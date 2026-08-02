//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "smooth_operation.h"
#include <cassert>
#include <utility>
#include <vector>

#include "geolio/common/log.h"

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
        /* Build vertex adjacency */
        std::vector<std::vector<GEO::index_t>> mesh_v_adjacent_v(mesh_.vertices.nb());
        build_vertex_adjacent_vertices(mesh_v_adjacent_v);

        /* Build fixed edge adjacent edges */
        std::vector<char> mesh_v_on_fixed_edges;
        std::vector<std::pair<GEO::index_t, GEO::index_t>> mesh_fixed_edge_v_adjacent_v; // v -> adjacent vertices along adjacent fixed edges
        if (ALLOW_SMOOTH_FIXED_EDGE_VERTICES)
            build_fixed_edge_adjacent_edges(mesh_fixed_edge_v_adjacent_v);
        else
            build_vertex_on_fixed_edges(mesh_v_on_fixed_edges);

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
                            if (v0 == GEO::NO_VERTEX || v1 == GEO::NO_VERTEX)
                                continue;
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
                            const auto& [v0, v1] = mesh_fixed_edge_v_adjacent_v[v];
                            if (v0 == GEO::NO_VERTEX || v1 == GEO::NO_VERTEX)
                                continue;
                            const auto& p0 = mesh_.vertices.point(v0);
                            const auto& p1 = mesh_.vertices.point(v1);
                            const auto p1p0 = GEO::normalize(p1-p0);
                            target_p = p0 + GEO::dot(target_p-p0, p1p0) * p1p0;
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

    void SmoothOperation::perform_iteratively(
        const double displacement_threshold
        ) {
        /* Build vertex adjacency */
        std::vector<std::vector<GEO::index_t>> mesh_v_adjacent_v(mesh_.vertices.nb());
        build_vertex_adjacent_vertices(mesh_v_adjacent_v);

        /* Build fixed edge adjacent edges */
        std::vector<char> mesh_v_on_fixed_edges;
        std::vector<std::pair<GEO::index_t, GEO::index_t>> mesh_fixed_edge_v_adjacent_v; // v -> adjacent vertices along adjacent fixed edges
        if (ALLOW_SMOOTH_FIXED_EDGE_VERTICES)
            build_fixed_edge_adjacent_edges(mesh_fixed_edge_v_adjacent_v);
        else
            build_vertex_on_fixed_edges(mesh_v_on_fixed_edges);

        /* Smooth */
        if (manager_.mesh_2d) {
            std::vector<GEO::vec2> mesh_v_new_pos(mesh_.vertices.nb()); // pre-allocated
            for (;;) {
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
                double max_displacement = -std::numeric_limits<double>::max();
                for (const auto& v : mesh_.vertices) {
                    if (is_perform_valid(v)) {
                        auto& target_p = mesh_v_new_pos[v];

                        /* Slide along fixed edge */
                        if (ALLOW_SMOOTH_FIXED_EDGE_VERTICES) {
                            assert(mesh_fixed_edge_v_adjacent_v.size() == mesh_.vertices.nb());
                            assert(!manager_.mesh_v_fixed[v]);
                            const auto& [v0, v1] = mesh_fixed_edge_v_adjacent_v[v];
                            if (v0 == GEO::NO_VERTEX || v1 == GEO::NO_VERTEX)
                                continue;
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

                        max_displacement = std::max(max_displacement, GEO::distance(mesh_.vertices.point<2>(v), target_p));
                        mesh_.vertices.point<2>(v) = target_p;
                    }
                }

                if (max_displacement < displacement_threshold)
                    break;
            }
        }
        else {
            assert(mesh_.vertices.dimension() == 3);

            std::vector<GEO::vec3> mesh_v_new_pos(mesh_.vertices.nb()); // pre-allocated
            for (;;) {
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
                double max_displacement = -std::numeric_limits<double>::max();
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
                            const auto& [v0, v1] = mesh_fixed_edge_v_adjacent_v[v];
                            if (v0 == GEO::NO_VERTEX || v1 == GEO::NO_VERTEX)
                                continue;
                            const auto& p0 = mesh_.vertices.point(v0);
                            const auto& p1 = mesh_.vertices.point(v1);
                            const auto p1p0 = GEO::normalize(p1-p0);
                            target_p = p0 + GEO::dot(target_p-p0, p1p0) * p1p0;
                        }
                        else {
                            assert(mesh_v_on_fixed_edges.size() == mesh_.vertices.nb());
                            if (mesh_v_on_fixed_edges[v])
                                continue;
                        }

                        max_displacement = std::max(max_displacement, GEO::distance(mesh_.vertices.point(v), target_p));
                        mesh_.vertices.point(v) = target_p;
                    }
                }

                LOG::DEBUG("max_displacement: {}/{}", max_displacement, displacement_threshold);
                if (max_displacement < displacement_threshold)
                    break;
            }
        }
    }

    void SmoothOperation::build_vertex_adjacent_vertices(
        std::vector<std::vector<GEO::index_t>>& mesh_v_adjacent_v
        ) const {
        mesh_v_adjacent_v.assign(mesh_.vertices.nb(), std::vector<GEO::index_t>());
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
    }

    void SmoothOperation::build_fixed_edge_adjacent_edges(
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& mesh_fixed_edge_v_adjacent_v
        ) const {
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

    void SmoothOperation::build_vertex_on_fixed_edges(
        std::vector<char>& mesh_v_on_fixed_edges
        ) const {
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
