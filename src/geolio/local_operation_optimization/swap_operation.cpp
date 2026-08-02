//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "swap_operation.h"
#include <cassert>
#include <utility>
#include <vector>
#include <geolio//mesh/tri_operations.h>
#include <geolio/mesh/mesh_operations.h>

#include "geolio/common/log.h"
#include "geolio/common/vecg.h"

namespace geolio
{
    SwapOperation::SwapOperation(
        MeshElementManager& mesh_element_manager
        ) : BaseOperation(mesh_element_manager)
    {}

    void SwapOperation::sweep_mesh(
        ) {
        mesh_f_timestamping_.fill(false);

        for (GEO::index_t f = 0, f_end = mesh_.facets.nb(); f < f_end; ++f) {
            if (mesh_f_timestamping_[f])
                continue;

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (!is_perform_valid(f, lv))
                    continue;

                const auto& nf = mesh_.facets.adjacent(f, lv);
                assert(nf != GEO::NO_FACET);

                perform(f, lv);

                post_process(f, lv, nf);

                assert(post_check());

                /* Label processed facets */
                mesh_f_timestamping_[f] = true;
                mesh_f_timestamping_[nf] = true;
            }
        }
    }

    void SwapOperation::run_iterative_loop(
        ) {
        mesh_f_timestamping_.fill(0); // as version timestamping

        std::vector<EdgeToCollapse> pq;

        /* Init vector */
        auto emplace_to_pq = [&](const GEO::index_t f, const GEO::index_t lv) {
            if (is_perform_valid(f, lv))
                pq.emplace_back(f, lv, 0);
        };
        for_each_edge(emplace_to_pq);

        /* Iteratively perform */
        while (!pq.empty()) {
            const auto edge = pq.back();
            pq.pop_back();

            /* Check validity */
            if (const auto& f_timestamping = mesh_f_timestamping_[edge.f];
                edge.timestamping < f_timestamping) { // This edge is not up-to-date. Push again.
                pq.emplace_back(edge.f, edge.lv, f_timestamping);
                continue;
            }

            if (!is_perform_valid(edge.f, edge.lv))
                continue;

            /* Swap */
            const auto nf = mesh_.facets.adjacent(edge.f, edge.lv);
            assert(nf != GEO::NO_FACET);
            const auto prev_f_timestamping = mesh_f_timestamping_[edge.f];
            const auto prev_nf_timestamping = mesh_f_timestamping_[nf];

            perform(edge.f, edge.lv);

            post_process(edge.f, edge.lv, nf);

            assert(post_check());

            /* Push new sub-edges */
            {
                auto& f_timestamping = mesh_f_timestamping_[edge.f];
                f_timestamping = prev_f_timestamping+1;
                pq.emplace_back(edge.f, edge.lv, f_timestamping);
                pq.emplace_back(edge.f, (edge.lv+2)%3, f_timestamping);
            }
            {
                auto& nf_timestamping = mesh_f_timestamping_[nf];
                nf_timestamping = prev_nf_timestamping+1;
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (mesh_.facets.adjacent(nf, lv) == edge.f)
                        continue;
                    pq.emplace_back(nf, lv, nf_timestamping);
                }
            }
        }
    }

    bool SwapOperation::is_perform_valid(
        const GEO::index_t f,
        const GEO::index_t lv
        ) const {
        assert(f < mesh_.facets.nb());
        assert(lv < 3);

        /* This facet should not yet exist. */
        if (!manager_.mesh_f_used[f])
            return false;

        /* Forbid swapping fixed edge. */
        if (const auto& fc = mesh_.facets.corner(f, lv);
            manager_.mesh_fc_fixed[fc])
            return false;

        /* Forbid swapping boundary edge. */
        const auto nf = mesh_.facets.adjacent(f, lv);
        if (nf == GEO::NO_FACET)
            return false;
        assert(nf != GEO::NO_FACET);
        assert(manager_.mesh_f_used[nf]);

        const auto v0 = mesh_.facets.vertex(f, lv);
        const auto lv1 = (lv+1)%3;
        const auto v1 = mesh_.facets.vertex(f, lv1);
        const auto lv2 = (lv+2)%3;
        const auto v2 = mesh_.facets.vertex(f, lv2);
        const auto nlv = (mesh_.facets.find_vertex(nf, v0)+1)%3;
        assert(nlv != GEO::NO_INDEX);
        const auto v3 = mesh_.facets.vertex(nf, nlv);

        /* It is prohibited to swap an edge that has a non-manifold vertex, because the vertex will not be changed. */
        if (manager_.mesh_v_non_manifold[v0] ||
            manager_.mesh_v_non_manifold[v1] ||
            manager_.mesh_v_non_manifold[v2] ||
            manager_.mesh_v_non_manifold[v3])
            return false;

        /* Swap operation is not valid. */
        if (!is_tri_edge_swap_valid(mesh_, f, lv))
            return false;

        /* Criterion */
        if (SWAP_CRITERION & SWAP_BASED_ON_VALENCE) { // The swap operation should help improve the overall valence.
            int valence0, valence1, valence2, valence3;
                std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
                {
                    get_vertex_incident_facets(mesh_, f, lv, ordered_f_and_lv);
                    valence0 = static_cast<int>(ordered_f_and_lv.size());
                }
                {
                    get_vertex_incident_facets(mesh_, f, lv1, ordered_f_and_lv);
                    valence1 = static_cast<int>(ordered_f_and_lv.size());
                }
                {
                    get_vertex_incident_facets(mesh_, f, lv2, ordered_f_and_lv);
                    valence2 = static_cast<int>(ordered_f_and_lv.size());
                }
                {
                    get_vertex_incident_facets(mesh_, nf, nlv, ordered_f_and_lv);
                    valence3 = static_cast<int>(ordered_f_and_lv.size());
                }
                constexpr int INTERIOR_IDEAL_VALENCE = 6;
                constexpr int BOUNDARY_IDEAL_VALENCE = 4;
                const auto v0_ideal_valence = manager_.mesh_v_boundary[v0] ? BOUNDARY_IDEAL_VALENCE : INTERIOR_IDEAL_VALENCE;
                const auto v1_ideal_valence = manager_.mesh_v_boundary[v1] ? BOUNDARY_IDEAL_VALENCE : INTERIOR_IDEAL_VALENCE;
                const auto v2_ideal_valence = manager_.mesh_v_boundary[v2] ? BOUNDARY_IDEAL_VALENCE : INTERIOR_IDEAL_VALENCE;
                const auto v3_ideal_valence = manager_.mesh_v_boundary[v3] ? BOUNDARY_IDEAL_VALENCE : INTERIOR_IDEAL_VALENCE;
                const auto prev_valence = std::pow(valence0-v0_ideal_valence, 2) +
                                          std::pow(valence1-v1_ideal_valence, 2) +
                                          std::pow(valence2-v2_ideal_valence, 2) +
                                          std::pow(valence3-v3_ideal_valence, 2);
                --valence0;
                --valence1;
                ++valence2;
                ++valence3;
                const auto post_valence = std::pow(valence0-v0_ideal_valence, 2) +
                                          std::pow(valence1-v1_ideal_valence, 2) +
                                          std::pow(valence2-v2_ideal_valence, 2) +
                                          std::pow(valence3-v3_ideal_valence, 2);
                if (post_valence >= prev_valence)
                    return false;
        }
        if (SWAP_CRITERION & SWAP_BASED_ON_DELAUNAY) {
            if (manager_.mesh_2d) {
                const auto& p0 = mesh_.vertices.point<2>(v0);
                const auto& p1 = mesh_.vertices.point<2>(v1);
                const auto& p2 = mesh_.vertices.point<2>(v2);
                const auto& p3 = mesh_.vertices.point<2>(v3);
                const auto p2p0 = p0-p2;
                const auto p2p1 = p1-p2;
                const auto p3p0 = p0-p3;
                const auto p3p1 = p1-p3;
                const auto cot_alpha = GEO::dot(p2p0, p2p1) / std::abs(geolio::cross(p2p0, p2p1));
                const auto cot_beta  = GEO::dot(p3p0, p3p1) / std::abs(geolio::cross(p3p0, p3p1));
                if (cot_alpha + cot_beta > -1e-5)
                    return false;
            }
            else {
                const auto& p0 = mesh_.vertices.point(v0);
                const auto& p1 = mesh_.vertices.point(v1);
                const auto& p2 = mesh_.vertices.point(v2);
                const auto& p3 = mesh_.vertices.point(v3);
                const auto p2p0 = p0-p2;
                const auto p2p1 = p1-p2;
                const auto p3p0 = p0-p3;
                const auto p3p1 = p1-p3;
                const auto cot_alpha = GEO::dot(p2p0, p2p1) / GEO::length(GEO::cross(p2p0, p2p1));
                const auto cot_beta = GEO::dot(p3p0, p3p1) / GEO::length(GEO::cross(p3p0, p3p1));
                if (cot_alpha + cot_beta > -1e-5)
                    return false;
            }
        }
        // if (SWAP_CRITERION & SWAP_BASED_ON_MAX_MIN_ANGLE) { // not robust
        //     if (manager_.mesh_2d) {
        //         const auto& p0 = mesh_.vertices.point<2>(v0);
        //         const auto& p1 = mesh_.vertices.point<2>(v1);
        //         const auto& p2 = mesh_.vertices.point<2>(v2);
        //         const auto& p3 = mesh_.vertices.point<2>(v3);
        //         const std::array<double, 6> prev_angles = {
        //             GEO::Geom::angle(p1-p0, p2-p0),
        //             GEO::Geom::angle(p0-p1, p2-p1),
        //             GEO::Geom::angle(p0-p2, p1-p2),
        //             GEO::Geom::angle(p1-p0, p3-p0),
        //             GEO::Geom::angle(p0-p1, p3-p1),
        //             GEO::Geom::angle(p0-p3, p1-p3),
        //         };
        //         const std::array<double, 6> post_angles = {
        //             GEO::Geom::angle(p2-p0, p3-p0),
        //             GEO::Geom::angle(p0-p2, p3-p2),
        //             GEO::Geom::angle(p0-p3, p2-p3),
        //             GEO::Geom::angle(p2-p1, p3-p1),
        //             GEO::Geom::angle(p1-p2, p3-p2),
        //             GEO::Geom::angle(p1-p3, p2-p3),
        //         };
        //         if (std::ranges::min(post_angles) < std::ranges::min(prev_angles) + 1e-5)
        //             return false;
        //     }
        //     else {
        //         const auto& p0 = mesh_.vertices.point(v0);
        //         const auto& p1 = mesh_.vertices.point(v1);
        //         const auto& p2 = mesh_.vertices.point(v2);
        //         const auto& p3 = mesh_.vertices.point(v3);
        //         const std::array<double, 6> prev_angles = {
        //             GEO::Geom::angle(p1-p0, p2-p0),
        //             GEO::Geom::angle(p0-p1, p2-p1),
        //             GEO::Geom::angle(p0-p2, p1-p2),
        //             GEO::Geom::angle(p1-p0, p3-p0),
        //             GEO::Geom::angle(p0-p1, p3-p1),
        //             GEO::Geom::angle(p0-p3, p1-p3),
        //         };
        //         const std::array<double, 6> post_angles = {
        //             GEO::Geom::angle(p2-p0, p3-p0),
        //             GEO::Geom::angle(p0-p2, p3-p2),
        //             GEO::Geom::angle(p0-p3, p2-p3),
        //             GEO::Geom::angle(p2-p1, p3-p1),
        //             GEO::Geom::angle(p1-p2, p3-p2),
        //             GEO::Geom::angle(p1-p3, p2-p3),
        //         };
        //         if (std::ranges::min(post_angles) < std::ranges::min(prev_angles) + 1e-5)
        //             return false;
        //     }
        // }

        return true;
    }

    void SwapOperation::perform(
        const GEO::index_t f,
        const GEO::index_t lv
        ) const {
        assert(f < mesh_.facets.nb());
        assert(lv < 3);

        tri_edge_swap(mesh_, f, lv);
    }

    void SwapOperation::post_process(
        const GEO::index_t f,
        const GEO::index_t lv,
        const GEO::index_t nf
        ) const {
        assert(f < mesh_.facets.nb());
        assert(lv < 3);
        assert(nf < mesh_.facets.nb());

        /* Reset facet attributes (will be restored after swap operation) */
        manager_.mesh_f_used[f] = true;
        manager_.mesh_f_used[nf] = true;
    }
}
