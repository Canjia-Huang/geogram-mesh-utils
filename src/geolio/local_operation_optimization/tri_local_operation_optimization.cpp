//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "tri_local_operation_optimization.h"
#include <unordered_set>
#include <utility>
#include <vector>
#include <geogram/mesh/mesh_io.h>
#include <geogram/numerics/exact_geometry.h>
#include <geolio/common//log.h>
#include <geolio/common/utils.h>
#include <geolio/mesh/detect_mesh_defects.h>
#include "smooth_operation.h"
#include "split_operation.h"
#include "swap_operation.h"

namespace
{
    constexpr GEO::index_t FIRST_ELEMENT_IDX = static_cast<GEO::index_t>(-2); /*
        Because the default value of GEO::index_t is 0, this would make it impossible to identify which one is the
        original first element after optimization; therefore, a special index is assigned to the first element. */
}

namespace geolio
{
    TriLocalOperationOptimization::TriLocalOperationOptimization(
        GEO::Mesh& mesh,
        GEO::Attribute<GEO::index_t>* mesh_v_original_idx,
        GEO::Attribute<GEO::index_t>* mesh_f_original_idx
        ) : mesh_(mesh),
            manager_(mesh),
            mesh_v_original_idx_(mesh_v_original_idx),
            mesh_f_original_idx_(mesh_f_original_idx)
    {
        assert(mesh_.vertices.nb() > 0);
        assert(mesh_.facets.nb() > 0);

        label_boundary_vertices();
        label_non_manifold_vertices();

        /* Init original idx attributes */
        if (mesh_v_original_idx_ != nullptr) {
            assert(mesh_v_original_idx_->is_bound());
            assert(mesh_v_original_idx_->size() == mesh_.vertices.nb());
            for (const auto& v : mesh_.vertices)
                (*mesh_v_original_idx_)[v] = v;
            (*mesh_v_original_idx_)[0] = FIRST_ELEMENT_IDX;
        }
        if (mesh_f_original_idx_ != nullptr) {
            assert(mesh_f_original_idx_->is_bound());
            assert(mesh_f_original_idx_->size() == mesh_.facets.nb());
            for (const auto& f : mesh_.facets)
                (*mesh_f_original_idx_)[f] = f;
            (*mesh_f_original_idx_)[0] = FIRST_ELEMENT_IDX;
        }
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

            split_operation.perform_iteratively();
            collapse_operation.perform_iteratively();
            swap_operation.perform_iteratively();
            smooth_operation.perform_iteratively();

            // split_operation.perform_one_pass();
            // collapse_operation.perform_one_pass();
            // swap_operation.perform_one_pass();
            // smooth_operation.perform_one_pass(3);

            LOG::DEBUG("#V: {} -> {}, #F: {} -> {}", PREV_VERTICES_NB, mesh_.vertices.nb(), PREV_FACETS_NB, mesh_.facets.nb());
        }

        /* Make output mesh valid */
        manager_.clean_unused_elements(true);
        LOG::DEBUG("result mesh #V: {}, #F: {}", mesh_.vertices.nb(), mesh_.facets.nb());

        /* Refactor original idx (if exist) */
        if (mesh_v_original_idx_ != nullptr) {
            for (const auto& v : mesh_.vertices) {
                if (auto& idx = (*mesh_v_original_idx_)[v];
                    idx == 0) // default, newly vertices
                    idx = GEO::NO_INDEX;
                else if (idx == FIRST_ELEMENT_IDX) // the idx of the first vertex of the original mesh
                    idx = 0;
            }
        }
        if (mesh_f_original_idx_ != nullptr) {
            for (const auto& f : mesh_.facets) {
                if (auto& idx = (*mesh_f_original_idx_)[f];
                    idx == 0) // default, newly vertices
                    idx = GEO::NO_INDEX;
                else if (idx == FIRST_ELEMENT_IDX) // the idx of the first vertex of the original mesh
                    idx = 0;
            }
        }
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
                    if (manager_.mesh_2d) {
                        const auto& p = mesh_.vertices.point<2>(v);
                        const auto& p0 = mesh_.vertices.point<2>(adj_v0);
                        const auto& p1 = mesh_.vertices.point<2>(adj_v1);
                        if (GEO::Geom::angle(p0-p, p1-p) < sharp_angle)
                            manager_.mesh_v_fixed[v] = true; // fix vertex that adjacent fixed edges form a sharp angle
                    }
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
}
