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
    template<GEO::index_t DIM>
    SwapOperation<DIM>::SwapOperation(
        MeshElementManager<DIM>& mesh_element_manager,
        const GEO::index_t swap_criterion
        ) : BaseOperation<DIM>(mesh_element_manager),
            SWAP_CRITERION_(swap_criterion)
    {
        /* Init timestamping */
        this->mesh_f_timestamping_.fill(0);

        /* Init vector */
        pq_.reserve(3*this->mesh_.facets.nb());
        auto emplace_to_pq = [&](const GEO::index_t f, const GEO::index_t lv) {
            if (is_perform_valid(f, lv))
                pq_.emplace_back(f, lv, 0);
        };
        this->for_each_edge(emplace_to_pq);
    }

    template<GEO::index_t DIM>
    bool SwapOperation<DIM>::do_once(
        const bool iteratively
        ) {
        if (pq_.empty())
            return false;

        const auto edge = pq_.back();
        pq_.pop_back();

        /* Check validity */
        if (const auto& f_timestamping = this->mesh_f_timestamping_[edge.f];
            edge.timestamping < f_timestamping
            ) { // This edge is not up-to-date. Push again.
            if (iteratively)
                pq_.emplace_back(edge.f, edge.lv, f_timestamping);
            return true;
        }

        if (!is_perform_valid(edge.f, edge.lv))
            return true;

        /* Swap */
        const auto nf = this->mesh_.facets.adjacent(edge.f, edge.lv);
        assert(nf != GEO::NO_FACET);
        const auto prev_f_timestamping = this->mesh_f_timestamping_[edge.f];
        const auto prev_nf_timestamping = this->mesh_f_timestamping_[nf];

        perform(edge.f, edge.lv);

        post_process(edge.f, edge.lv, nf);

        assert(this->post_check());

        /* Push new sub-edges */
        {
            auto& f_timestamping = this->mesh_f_timestamping_[edge.f];
            f_timestamping = prev_f_timestamping+1;

            if (iteratively) {
                pq_.emplace_back(edge.f, edge.lv, f_timestamping);
                pq_.emplace_back(edge.f, (edge.lv+2)%3, f_timestamping);
            }
        }
        {
            auto& nf_timestamping = this->mesh_f_timestamping_[nf];
            nf_timestamping = prev_nf_timestamping+1;

            if (iteratively) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (this->mesh_.facets.adjacent(nf, lv) == edge.f)
                        continue;
                    pq_.emplace_back(nf, lv, nf_timestamping);
                }
            }
        }

        return true;
    }

    template<GEO::index_t DIM>
    void SwapOperation<DIM>::run_through(
        const bool iteratively
        ) {
        while (do_once(iteratively)) {}
    }

    template<GEO::index_t DIM>
    bool SwapOperation<DIM>::is_perform_valid(
        const GEO::index_t f,
        const GEO::index_t lv
        ) const {
        assert(f < this->mesh_.facets.nb());
        assert(lv < 3);

        /* This facet should not yet exist. */
        if (!this->manager_.mesh_f_used[f])
            return false;

        /* Forbid swapping fixed edge. */
        if (const auto& fc = this->mesh_.facets.corner(f, lv);
            this->manager_.mesh_fc_fixed[fc])
            return false;

        /* Forbid swapping boundary edge. */
        const auto nf = this->mesh_.facets.adjacent(f, lv);
        if (nf == GEO::NO_FACET)
            return false;
        assert(nf != GEO::NO_FACET);
        assert(this->manager_.mesh_f_used[nf]);

        const auto v0 = this->mesh_.facets.vertex(f, lv);
        const auto lv1 = (lv+1)%3;
        const auto v1 = this->mesh_.facets.vertex(f, lv1);
        const auto lv2 = (lv+2)%3;
        const auto v2 = this->mesh_.facets.vertex(f, lv2);
        const auto nlv = (this->mesh_.facets.find_vertex(nf, v0)+1)%3;
        assert(nlv != GEO::NO_INDEX);
        const auto v3 = this->mesh_.facets.vertex(nf, nlv);

        /* It is prohibited to swap an edge that has a non-manifold vertex, because the vertex will not be changed. */
        if (this->manager_.mesh_v_non_manifold[v0] ||
            this->manager_.mesh_v_non_manifold[v1] ||
            this->manager_.mesh_v_non_manifold[v2] ||
            this->manager_.mesh_v_non_manifold[v3])
            return false;

        /* Swap operation is not valid. */
        if (!is_tri_edge_swap_valid(this->mesh_, f, lv))
            return false;

        /* Criterion */
        if (SWAP_CRITERION_ & SWAP_BASED_ON_VALENCE) { // The swap operation should help improve the overall valence.
            int valence0, valence1, valence2, valence3;
                std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
                {
                    get_vertex_incident_facets(this->mesh_, f, lv, ordered_f_and_lv);
                    valence0 = static_cast<int>(ordered_f_and_lv.size());
                }
                {
                    get_vertex_incident_facets(this->mesh_, f, lv1, ordered_f_and_lv);
                    valence1 = static_cast<int>(ordered_f_and_lv.size());
                }
                {
                    get_vertex_incident_facets(this->mesh_, f, lv2, ordered_f_and_lv);
                    valence2 = static_cast<int>(ordered_f_and_lv.size());
                }
                {
                    get_vertex_incident_facets(this->mesh_, nf, nlv, ordered_f_and_lv);
                    valence3 = static_cast<int>(ordered_f_and_lv.size());
                }
                constexpr int INTERIOR_IDEAL_VALENCE = 6;
                constexpr int BOUNDARY_IDEAL_VALENCE = 4;
                const auto v0_ideal_valence = this->manager_.mesh_v_boundary[v0] ? BOUNDARY_IDEAL_VALENCE : INTERIOR_IDEAL_VALENCE;
                const auto v1_ideal_valence = this->manager_.mesh_v_boundary[v1] ? BOUNDARY_IDEAL_VALENCE : INTERIOR_IDEAL_VALENCE;
                const auto v2_ideal_valence = this->manager_.mesh_v_boundary[v2] ? BOUNDARY_IDEAL_VALENCE : INTERIOR_IDEAL_VALENCE;
                const auto v3_ideal_valence = this->manager_.mesh_v_boundary[v3] ? BOUNDARY_IDEAL_VALENCE : INTERIOR_IDEAL_VALENCE;
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
        if (SWAP_CRITERION_ & SWAP_BASED_ON_DELAUNAY) {
                const auto& p0 = this->mesh_.vertices.template point<DIM>(v0);
                const auto& p1 = this->mesh_.vertices.template point<DIM>(v1);
                const auto& p2 = this->mesh_.vertices.template point<DIM>(v2);
                const auto& p3 = this->mesh_.vertices.template point<DIM>(v3);
                const auto p2p0 = p0-p2;
                const auto p2p1 = p1-p2;
                const auto p3p0 = p0-p3;
                const auto p3p1 = p1-p3;
            double cot_alpha, cot_beta;
            if constexpr (DIM == 2) {
                cot_alpha = GEO::dot(p2p0, p2p1) / std::abs(geolio::cross(p2p0, p2p1));
                cot_beta  = GEO::dot(p3p0, p3p1) / std::abs(geolio::cross(p3p0, p3p1));
            }
            else {
                cot_alpha = GEO::dot(p2p0, p2p1) / GEO::length(GEO::cross(p2p0, p2p1));
                cot_beta  = GEO::dot(p3p0, p3p1) / GEO::length(GEO::cross(p3p0, p3p1));
            }
            if (cot_alpha + cot_beta > -1e-5)
                return false;
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

    template<GEO::index_t DIM>
    void SwapOperation<DIM>::perform(
        const GEO::index_t f,
        const GEO::index_t lv
        ) const {
        assert(f < this->mesh_.facets.nb());
        assert(lv < 3);

        tri_edge_swap(this->mesh_, f, lv);
    }

    template<GEO::index_t DIM>
    void SwapOperation<DIM>::post_process(
        const GEO::index_t f,
        const GEO::index_t lv,
        const GEO::index_t nf
        ) const {
        assert(f < this->mesh_.facets.nb());
        assert(lv < 3);
        assert(nf < this->mesh_.facets.nb());

        /* Reset facet attributes (will be restored after swap operation) */
        this->manager_.mesh_f_used[f] = true;
        this->manager_.mesh_f_used[nf] = true;
    }

    template class SwapOperation<2>;
    template class SwapOperation<3>;
}
