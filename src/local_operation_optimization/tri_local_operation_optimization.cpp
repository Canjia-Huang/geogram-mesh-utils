//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "tri_local_operation_optimization.h"
#include <ranges>
#include <geogram/mesh/mesh_io.h>
#include "geolio/log.h"
#include "geolio/tri_operations.h"
#include "geolio/utils.h"
#include <unordered_set>
#include <geogram/numerics/exact_geometry.h>

#include "geolio/detect_mesh_defects.h"

namespace geolio
{
    TriLocalOperationOptimization::TriLocalOperationOptimization(
        GEO::Mesh& mesh
        ) : mesh_(mesh),
            manager_(mesh)
    {
        label_boundary_vertices();
        label_non_manifold_vertices();
    }

    void TriLocalOperationOptimization::optimize(
        GEO::index_t rounds_nb,
        double target_edge_length
        ) {
        LOG::TRACE(__FUNCTION__);

        /* Compute target edge length */
        if (target_edge_length < 0) {
            target_edge_length = manager_.compute_average_mesh_edge_length();
            LOG::DEBUG("Automatically set the target edge length to the average edge length {}.", target_edge_length);
        }
        assert(target_edge_length > 0);
        const double SPLIT_EDGE_LENGTH = 4.0/3.0 * target_edge_length;
        const double COLLAPSE_EDGE_LENGTH = 4.0/5.0 * target_edge_length;

        /* Init */
        SplitOperation split_operation(manager_, SPLIT_EDGE_LENGTH);
        CollapseOperation collapse_operation(manager_, COLLAPSE_EDGE_LENGTH);
        SwapOperation swap_operation(manager_);
        SmoothOperation smooth_operation(manager_);

        /* Let's go! */
        for (GEO::index_t round = 0; round < rounds_nb; ++round) {
            LOG::TRACE("round: {}/{}", round+1, rounds_nb);

            const auto PREV_VERTICES_NB = mesh_.vertices.nb();
            const auto PREV_FACETS_NB = mesh_.facets.nb();

            split_operation.perform_one_pass();
            // collapse_operation.perform_one_pass();
            // swap_operation.perform_one_pass();
            // smooth_operation.perform_one_pass(3);

            LOG::DEBUG("#V: {} -> {}, #F: {} -> {}", PREV_VERTICES_NB, mesh_.vertices.nb(), PREV_FACETS_NB, mesh_.facets.nb());
        }

        /* Make output mesh valid */
        manager_.clean_unused_elements(true);
        LOG::DEBUG("result mesh #V: {}, #F: {}", mesh_.vertices.nb(), mesh_.facets.nb());
    }

    void TriLocalOperationOptimization::fix_boundary_elements(
        ) {
        LOG::TRACE(__FUNCTION__);

        /* Fix edges */
        for (const auto& f : mesh_.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (manager_.mesh_fc_fixed[mesh_.facets.corner(f, lv)])
                    continue;

                if (mesh_.facets.adjacent(f, lv) == GEO::NO_FACET)
                    fix_edge(f, lv);
            }
        }

        fix_vertices_based_on_fixed_edges();
    }

    void TriLocalOperationOptimization::fix_sharp_elements(
        const double sharp_angle
        ) {
        LOG::TRACE(__FUNCTION__);
        assert(mesh_.vertices.dimension() == 3);

        /* Pre-compute facet normals */
        std::vector<GEO::vec3> mesh_f_normal(mesh_.facets.nb());
        for (const auto& f : mesh_.facets)
            mesh_f_normal[f] = GEO::Geom::triangle_normal(
                mesh_.facets.point(f, 0),
                mesh_.facets.point(f, 1),
                mesh_.facets.point(f, 2));

        /* Fix edges */
        for (const auto& f : mesh_.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (manager_.mesh_fc_fixed[mesh_.facets.corner(f, lv)])
                    continue;

                if (const auto& nf = mesh_.facets.adjacent(f, lv);
                    nf != GEO::NO_FACET &&
                    M_PI - GEO::Geom::angle(mesh_f_normal[f], mesh_f_normal[nf]) < sharp_angle)
                    fix_edge(f, lv);
            }
        }

        fix_vertices_based_on_fixed_edges();
    }

    void TriLocalOperationOptimization::label_boundary_vertices(
        ) {
        LOG::TRACE(__FUNCTION__);

        manager_.mesh_v_boundary.fill(false);
        for (const auto& f : mesh_.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (mesh_.facets.adjacent(f, lv) == GEO::NO_FACET) {
                    manager_.mesh_v_boundary[mesh_.facets.vertex(f, lv)] = true;
                    manager_.mesh_v_boundary[mesh_.facets.vertex(f, (lv+1)%3)] = true;
                }
            }
        }
    }

    void TriLocalOperationOptimization::label_non_manifold_vertices(
        ) {
        LOG::TRACE(__FUNCTION__);

        std::vector<GEO::index_t> non_manifold_vertices;
        const auto NON_MANIFOLD_VERTICES_NB = detect_non_manifold_vertices(mesh_, non_manifold_vertices);

        manager_.mesh_v_non_manifold.fill(false);
        for (const auto& v : non_manifold_vertices) {
            manager_.mesh_v_non_manifold[v] = true;
            fix_vertex(v); // fix it, prevent errors in collapse operations involving this vertex.
        }

        LOG::WARN("Detected {} non-manifold vertices in the input mesh; "
                  "they have been fixed to prevent unexpected errors.", NON_MANIFOLD_VERTICES_NB);
    }

    void TriLocalOperationOptimization::fix_vertices_based_on_fixed_edges(
        const double sharp_angle
        ) {
        std::vector<std::pair<GEO::index_t, GEO::index_t>> mesh_v_adjacent_v(
            mesh_.vertices.nb(), std::pair(GEO::NO_VERTEX, GEO::NO_VERTEX));

        for (const auto& f : mesh_.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (manager_.mesh_fc_fixed[mesh_.facets.corner(f, lv)]) {
                    const auto& ev0 = mesh_.facets.vertex(f, lv);
                    const auto& ev1 = mesh_.facets.vertex(f, (lv+1)%3);

                    if (auto& [adj_v0, adj_v1] = mesh_v_adjacent_v[ev0];
                        adj_v0 == GEO::NO_VERTEX)
                        adj_v0 = ev1;
                    else if (adj_v0 != ev1) {
                        if (adj_v1 == GEO::NO_VERTEX)
                            adj_v1 = ev1;
                        else if (adj_v1 != ev1)
                            manager_.mesh_v_fixed[ev0] = true; // fix vertex adjacent to more than 3 fixed edges
                    }

                    if (auto& [adj_v0, adj_v1] = mesh_v_adjacent_v[ev1];
                        adj_v0 == GEO::NO_VERTEX)
                        adj_v0 = ev0;
                    else if (adj_v0 != ev0) {
                        if (adj_v1 == GEO::NO_VERTEX)
                            adj_v1 = ev0;
                        else if (adj_v1 != ev0)
                            manager_.mesh_v_fixed[ev1] = true; // fix vertex adjacent to more than 3 fixed edges
                    }
                }
            }
        }

        for (const auto& v : mesh_.vertices) {
            if (manager_.mesh_v_fixed[v])
                continue;
            if (const auto& [adj_v0, adj_v1] = mesh_v_adjacent_v[v];
                adj_v0 != GEO::NO_VERTEX
                ) {
                if (adj_v1 == GEO::NO_VERTEX)
                    manager_.mesh_v_fixed[v] = true; // fix vertex that adjacent to only one fixed edge
                else {
                    const auto& p = mesh_.vertices.point(v);
                    const auto& p0 = mesh_.vertices.point(adj_v0);
                    const auto& p1 = mesh_.vertices.point(adj_v1);
                    if (GEO::Geom::angle(p0-p, p1-p) < sharp_angle)
                        manager_.mesh_v_fixed[v] = true; // fix vertex that adjacent fixed edges form a sharp angle
                }
            }
        }
    }
}
