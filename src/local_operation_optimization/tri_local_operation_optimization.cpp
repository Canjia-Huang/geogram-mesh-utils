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
        double target_edge_length,
        GEO::index_t rounds_nb
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
            collapse_operation.perform_one_pass();
            swap_operation.perform_one_pass();
            smooth_operation.perform_one_pass(5);

            LOG::DEBUG("#V: {} -> {}, #F: {} -> {}", PREV_VERTICES_NB, mesh_.vertices.nb(), PREV_FACETS_NB, mesh_.facets.nb());
        }

        /* Make output mesh valid */
        manager_.clean_unused_elements(true);
        LOG::DEBUG("result mesh #V: {}, #F: {}", mesh_.vertices.nb(), mesh_.facets.nb());
    }

    void TriLocalOperationOptimization::fix_boundary_elements(
        ) {
        LOG::TRACE(__FUNCTION__);

        for (const auto& f : mesh_.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (mesh_.facets.adjacent(f, lv) == GEO::NO_FACET) {
                    fix_vertex(mesh_.facets.vertex(f, lv));
                    fix_vertex(mesh_.facets.vertex(f, (lv+1)%3));
                    fix_edge(f, lv);
                }
            }
        }
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
}
