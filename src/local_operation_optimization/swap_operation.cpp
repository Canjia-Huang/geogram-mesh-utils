//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "swap_operation.h"
#include <cassert>

#include "geolio/tri_operations.h"

namespace geolio
{
    SwapOperation::SwapOperation(
        MeshElementManager& mesh_element_manager
        ) : BaseOperation(mesh_element_manager)
    {}

    void SwapOperation::perform_one_pass(
        ) {
        mesh_f_processed_.fill(false);

        for (GEO::index_t f = 0, f_end = mesh_.facets.nb(); f < f_end; ++f) {
            if (mesh_f_processed_[f])
                continue;

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (!is_perform_valid(f, lv))
                    continue;

                const auto& nf = mesh_.facets.adjacent(f, lv);
                assert(nf != GEO::NO_FACET);

                perform(f, lv);

                post_process(f, lv);

                assert(post_check());

                /* Label processed facets */
                mesh_f_processed_[f] = true;
                mesh_f_processed_[nf] = true;
            }
        }
    }

    bool SwapOperation::is_perform_valid(
        const GEO::index_t f,
        const GEO::index_t lv
        ) const {
        assert(f < mesh_.facets.nb());
        assert(lv < 3);

        if (!manager_.mesh_f_used[f]) // This facet should not yet exist.
            return false;

        if (const auto& fc = mesh_.facets.corner(f, lv);
            manager_.mesh_fc_fixed[fc]) // Forbid swapping fixed edge.
            return false;

        const auto nf = mesh_.facets.adjacent(f, lv);
        if (nf == GEO::NO_FACET) // Forbid swapping boundary edge.
            return false;
        assert(nf != GEO::NO_FACET);
        assert(manager_.mesh_f_used[nf]);

        const auto v0 = mesh_.facets.vertex(f, lv);
        const auto lv1 = (lv+1)%3;
        const auto v1 = mesh_.facets.vertex(f, lv1);
        const auto lv2 = (lv+2)%3;
        const auto v2 = mesh_.facets.vertex(f, lv2);
        const auto nlv = (mesh_.facets.find_vertex(nf, v0) + 1)%3;
        assert(nlv != GEO::NO_INDEX);
        const auto v3 = mesh_.facets.vertex(nf, nlv);

        if (manager_.mesh_v_non_manifold[v0] ||
            manager_.mesh_v_non_manifold[v1] ||
            manager_.mesh_v_non_manifold[v2] ||
            manager_.mesh_v_non_manifold[v3])
            return false; // It is prohibited to swap an edge that has a non-manifold vertex, because the vertex will not be changed.

        if (!is_tri_edge_swap_valid(mesh_, f, lv)) // Swap operation is not valid.
            return false;

        { // The swap operation should help improve the overall valence.
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
                if (post_valence > prev_valence)
                    return false;
        }

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
        const GEO::index_t lv
        ) const {
        // do nothing
    }
}
