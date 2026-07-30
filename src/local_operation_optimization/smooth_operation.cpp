//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "smooth_operation.h"
#include <cassert>

namespace geolio
{
    SmoothOperation::SmoothOperation(
        MeshElementManager& mesh_element_manager
        ) : BaseOperation(mesh_element_manager)
    {
        if (!manager_.mesh_2d) {
            original_mesh_.copy(mesh_);
            original_mesh_facet_AABB_.initialize(original_mesh_);
        }
    }

    void SmoothOperation::perform_one_pass(
        const GEO::index_t iterations_nb
        ) const {
        /* Get adjacency */
        std::vector<std::vector<GEO::index_t>> mesh_v_adjacent_v(mesh_.vertices.nb());
        for (const auto& f : mesh_.facets) {
            if (!manager_.mesh_f_used[f])
                continue;

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                const auto& v = mesh_.facets.vertex(f, lv);
                assert(manager_.mesh_v_used[v]);
                const auto& nv = mesh_.facets.vertex(f, (lv+1)%3);
                assert(manager_.mesh_v_used[nv]);

                mesh_v_adjacent_v[v].push_back(nv);
                if (mesh_.facets.adjacent(f, lv) == GEO::NO_FACET)
                    mesh_v_adjacent_v[nv].push_back(v);
            }
        }

        if (manager_.mesh_2d) {
            std::vector<GEO::vec2> mesh_v_new_pos(mesh_.vertices.nb()); // pre-allocated
            for (GEO::index_t iter = 0; iter < iterations_nb; ++iter) {
                /* Compute average position */
                for (const auto& v : mesh_.vertices) {
                    if (!manager_.mesh_v_used[v])
                        continue;

                    assert(!mesh_v_adjacent_v[v].empty());
                    mesh_v_new_pos[v] = GEO::vec2(0, 0);
                    for (const auto& nv : mesh_v_adjacent_v[v])
                        mesh_v_new_pos[v] += mesh_.vertices.point<2>(nv);
                    mesh_v_new_pos[v] /= mesh_v_adjacent_v[v].size();
                }

                /* Update */
                if (manager_.ALLOW_SMOOTH_FIXED_EDGE_VERTICES) {
                    // TODO: Slide along the fixed edge.
                }
                for (const auto& v : mesh_.vertices) {
                    if (is_perform_valid(v))
                        mesh_.vertices.point<2>(v) = mesh_v_new_pos[v];
                }
            }
        }
        else {
            assert(mesh_.vertices.dimension() == 3);

            std::vector<GEO::vec3> mesh_v_new_pos(mesh_.vertices.nb()); // pre-allocated
            for (GEO::index_t iter = 0; iter < iterations_nb; ++iter) {
                /* Compute average position */
                for (const auto& v : mesh_.vertices) {
                    if (!manager_.mesh_v_used[v])
                        continue;

                    assert(!mesh_v_adjacent_v[v].empty());
                    mesh_v_new_pos[v] = GEO::vec3(0, 0, 0);
                    for (const auto& nv : mesh_v_adjacent_v[v])
                        mesh_v_new_pos[v] += mesh_.vertices.point(nv);
                    mesh_v_new_pos[v] /= mesh_v_adjacent_v[v].size();
                }

                /* Project to original mesh */
                for (const auto& v : mesh_.vertices) {
                    if (is_perform_valid(v)) {
                        GEO::vec3 nearest_pos;
                        double sq_dist;
                        original_mesh_facet_AABB_.nearest_facet(mesh_v_new_pos[v], nearest_pos, sq_dist);

                        mesh_v_new_pos[v] = nearest_pos;
                    }
                }

                /* Update */
                if (manager_.ALLOW_SMOOTH_FIXED_EDGE_VERTICES) {
                    // TODO: Slide along the fixed edge.
                }
                for (const auto& v : mesh_.vertices) {
                    if (is_perform_valid(v))
                        mesh_.vertices.point(v) = mesh_v_new_pos[v];
                }
            }
        }
    }

    bool SmoothOperation::is_perform_valid(
        const GEO::index_t v
        ) const {
        assert(v < mesh_.vertices.nb());

        if (!manager_.mesh_v_used[v]) // This vertex should not yet exist.
            return false;

        if (manager_.mesh_v_fixed[v]) // Cannot move fixed vertex.
            return false;

        if (!manager_.ALLOW_SMOOTH_FIXED_EDGE_VERTICES &&
            manager_.mesh_v_fixed[v]) // Sliding along the fixed edge is prohibited.
            return false;

        return true;
    }
}
