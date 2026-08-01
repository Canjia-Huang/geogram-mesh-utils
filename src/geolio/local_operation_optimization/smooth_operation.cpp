//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "smooth_operation.h"
#include <cassert>
#include <utility>
#include <vector>

namespace geolio
{
    /**
     * @brief Constructs a SmoothOperation for relaxing vertex positions.
     * @details Initializes the base operation. For 3D meshes it keeps a copy of the input
     *          mesh (original_mesh_) and builds a GEO::MeshFacetsAABB over it so smoothed
     *          vertices can be projected back onto the original surface. 2D meshes skip the
     *          copy because no projection is needed.
     * @param[in] mesh_element_manager The mesh element manager exposing the mesh and its
     *                                 usage/fixed element attributes.
     */
    SmoothOperation::SmoothOperation(
        MeshElementManager& mesh_element_manager
        ) : BaseOperation(mesh_element_manager)
    {
        if (!manager_.mesh_2d) {
            original_mesh_.copy(mesh_);
            original_mesh_facet_AABB_.initialize(original_mesh_);
        }
    }

    /**
     * @brief Runs a number of smoothing iterations over the mesh vertices.
     * @details Builds the one-ring adjacency of every used vertex from the used facets, and
     *          optionally the set of vertices/edges adjacent to fixed edges. For each
     *          iteration it computes each movable vertex's target position as the average of
     *          its neighbours, projects it onto the original surface for 3D meshes, slides
     *          it along adjacent fixed edges when allowed, and writes it back if
     *          is_perform_valid() passes.
     * @param[in] iterations_nb Number of smoothing iterations to execute.
     */
    void SmoothOperation::perform_one_pass(
        const GEO::index_t iterations_nb
        ) const {
        /* Build vertex adjacency */
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

        /* Build fixed edge adjacent edges */
        std::vector<char> mesh_v_on_fixed_edges;
        std::vector<std::pair<GEO::index_t, GEO::index_t>> mesh_fixed_edge_v_adjacent_v; // v -> adjacent vertices along adjacent fixed edges
        if (ALLOW_SMOOTH_FIXED_EDGE_VERTICES) {
            mesh_fixed_edge_v_adjacent_v.assign(mesh_.vertices.nb(), std::pair(GEO::NO_VERTEX, GEO::NO_VERTEX));

            for (const auto& f : mesh_.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (const auto& fc = mesh_.facets.corner(f, lv);
                        manager_.mesh_fc_fixed[fc]
                        ) {
                        const auto& ev0 = mesh_.facets.vertex(f, lv);
                        const auto& ev1 = mesh_.facets.vertex(f, (lv+1)%3);
                        {
                            if (auto& [adj_v0, adj_v1] = mesh_fixed_edge_v_adjacent_v[ev0];
                                adj_v0 == GEO::NO_VERTEX)
                                adj_v0 = ev1;
                            else if (adj_v1 == GEO::NO_VERTEX) {
                                if (adj_v0 != ev1)
                                    adj_v1 = ev1;
                            }
                        }
                        {
                            if (auto& [adj_v0, adj_v1] = mesh_fixed_edge_v_adjacent_v[ev1];
                                adj_v0 == GEO::NO_VERTEX)
                                adj_v0 = ev0;
                            else if (adj_v1 == GEO::NO_VERTEX) {
                                if (adj_v0 != ev0)
                                    adj_v1 = ev0;
                            }
                        }
                    }
                }
            }

            /* Fix fixed vertex */
            for (const auto& v : mesh_.vertices) {
                if (manager_.mesh_v_fixed[v])
                    mesh_fixed_edge_v_adjacent_v[v] = std::pair(GEO::NO_VERTEX, GEO::NO_VERTEX);
            }
        }
        else {
            mesh_v_on_fixed_edges.assign(mesh_.vertices.nb(), false);
            for (const auto& f : mesh_.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (const auto& fc = mesh_.facets.corner(f, lv);
                        manager_.mesh_fc_fixed[fc]
                        ) {
                        const auto& ev0 = mesh_.facets.vertex(f, lv);
                        const auto& ev1 = mesh_.facets.vertex(f, (lv+1)%3);
                        mesh_v_on_fixed_edges[ev0] = true;
                        mesh_v_on_fixed_edges[ev1] = true;
                    }
                }
            }
        }

        /* Smooth */
        if (manager_.mesh_2d) {
            std::vector<GEO::vec2> mesh_v_new_pos(mesh_.vertices.nb()); // pre-allocated
            for (GEO::index_t iter = 0; iter < iterations_nb; ++iter) {
                /* Compute average position */
                for (const auto& v : mesh_.vertices) {
                    if (!manager_.mesh_v_used[v])
                        continue;

                    auto& target_p = mesh_v_new_pos[v];
                    target_p = GEO::vec2(0, 0);
                    for (const auto& nv : mesh_v_adjacent_v[v])
                        target_p += mesh_.vertices.point<2>(nv);
                    assert(!mesh_v_adjacent_v[v].empty());
                    target_p /= mesh_v_adjacent_v[v].size();
                }

                /* Update */
                for (const auto& v : mesh_.vertices) {
                    if (is_perform_valid(v)) {
                        auto& target_p = mesh_v_new_pos[v];

                        /* Slide along fixed edge */
                        if (ALLOW_SMOOTH_FIXED_EDGE_VERTICES) {
                            assert(mesh_fixed_edge_v_adjacent_v.size() == mesh_.vertices.nb());
                            assert(!manager_.mesh_v_fixed[v]);
                            const auto& [v0, v1] = mesh_fixed_edge_v_adjacent_v[v];
                            assert(v0 != GEO::NO_VERTEX);
                            assert(v1 != GEO::NO_VERTEX);
                            const auto& p0 = mesh_.vertices.point<2>(v0);
                            const auto& p1 = mesh_.vertices.point<2>(v1);
                            const auto p1p0 = GEO::normalize(p1-p0);
                            target_p = p0 + GEO::dot(target_p-p0, p1p0) * p1p0;
                        }
                        else {
                            assert(mesh_v_on_fixed_edges.size() == mesh_.vertices.nb());
                            if (mesh_v_on_fixed_edges[v])
                                continue;
                        }

                        mesh_.vertices.point<2>(v) = target_p;
                    }
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

                    auto& target_p = mesh_v_new_pos[v];
                    target_p = GEO::vec3(0, 0, 0);
                    for (const auto& nv : mesh_v_adjacent_v[v])
                        target_p += mesh_.vertices.point(nv);
                    assert(!mesh_v_adjacent_v[v].empty());
                    target_p /= mesh_v_adjacent_v[v].size();
                }

                /* Update */
                for (const auto& v : mesh_.vertices) {
                    if (is_perform_valid(v)) {
                        auto& target_p = mesh_v_new_pos[v];

                        { /* Project to original mesh */
                            GEO::vec3 nearest_pos;
                            double sq_dist;
                            original_mesh_facet_AABB_.nearest_facet(target_p, nearest_pos, sq_dist);

                            target_p = nearest_pos;
                        }

                        /* Slide along fixed edge */
                        if (ALLOW_SMOOTH_FIXED_EDGE_VERTICES) {
                            assert(mesh_fixed_edge_v_adjacent_v.size() == mesh_.vertices.nb());
                            assert(!manager_.mesh_v_fixed[v]);
                            if (const auto& [v0, v1] = mesh_fixed_edge_v_adjacent_v[v];
                                v0 != GEO::NO_VERTEX && v1 != GEO::NO_VERTEX
                                ) {
                                const auto& p0 = mesh_.vertices.point(v0);
                                const auto& p1 = mesh_.vertices.point(v1);
                                const auto p1p0 = GEO::normalize(p1-p0);
                                target_p = p0 + GEO::dot(target_p-p0, p1p0) * p1p0;
                            }
                        }
                        else {
                            assert(mesh_v_on_fixed_edges.size() == mesh_.vertices.nb());
                            if (mesh_v_on_fixed_edges[v])
                                continue;
                        }

                        mesh_.vertices.point(v) = mesh_v_new_pos[v];
                    }
                }
            }
        }
    }

    /**
     * @brief Checks whether vertex @p v is allowed to move during smoothing.
     * @details Returns false when the vertex is no longer in use or is marked as fixed;
     *          returns true otherwise.
     * @param[in] v Index of the vertex to test.
     * @return true if the vertex may be moved; false otherwise.
     */
    bool SmoothOperation::is_perform_valid(
        const GEO::index_t v
        ) const {
        assert(v < mesh_.vertices.nb());

        if (!manager_.mesh_v_used[v]) // This vertex should not yet exist.
            return false;

        if (manager_.mesh_v_fixed[v]) // Cannot move fixed vertex.
            return false;

        return true;
    }
}
