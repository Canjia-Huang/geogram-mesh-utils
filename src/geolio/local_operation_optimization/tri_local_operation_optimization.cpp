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

namespace geolio
{
    /**
     * @brief Constructs a TriLocalOperationOptimization over the given triangle mesh.
     * @details Stores the mesh reference, builds the internal MeshElementManager, and
     *          labels boundary and non-manifold vertices so subsequent operations can
     *          preserve or fix them.
     * @param[in] mesh The triangle mesh to optimize; only simplex facets are supported.
     */
    TriLocalOperationOptimization::TriLocalOperationOptimization(
        GEO::Mesh& mesh
        ) : mesh_(mesh),
            manager_(mesh)
    {
        label_boundary_vertices();
        label_non_manifold_vertices();
    }

    /**
     * @brief Runs the local optimization pipeline.
     * @details If target_edge_length is negative it is derived automatically from
     *          compute_average_mesh_edge_length(). The split and collapse thresholds are
     *          then set to 4/3 and 4/5 of that target. A SplitOperation, CollapseOperation,
     *          SwapOperation and SmoothOperation are created once, and for each round their
     *          perform_one_pass() methods are invoked in split, collapse, swap, smooth order
     *          (smooth runs 3 iterations). After all rounds, clean_unused_elements() removes
     *          the disused facets and isolated vertices.
     * @param[in] rounds_nb Number of optimization rounds to execute. Each round runs split,
     *                      collapse, swap and smooth passes in that order.
     * @param[in] target_edge_length Desired target edge length used to guide split/collapse
     *                               decisions. If negative, the optimizer computes an
     *                               automatic target via compute_average_mesh_edge_length().
     */
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
            collapse_operation.perform_one_pass();
            swap_operation.perform_one_pass();
            smooth_operation.perform_one_pass(3);

            LOG::DEBUG("#V: {} -> {}, #F: {} -> {}", PREV_VERTICES_NB, mesh_.vertices.nb(), PREV_FACETS_NB, mesh_.facets.nb());
        }

        /* Make output mesh valid */
        manager_.clean_unused_elements(true);
        LOG::DEBUG("result mesh #V: {}, #F: {}", mesh_.vertices.nb(), mesh_.facets.nb());
    }

    /**
     * @brief Fixes (locks) all boundary edges of the mesh.
     * @details Iterates over every facet corner and fixes each edge that has no adjacent
     *          facet (a boundary edge), skipping corners already fixed. Afterwards calls
     *          fix_vertices_based_on_fixed_edges() so that boundary vertices are fixed too.
     */
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

    /**
     * @brief Fixes (locks) edges whose dihedral angle is sharp.
     * @details Pre-computes the normal of every facet, then fixes each interior edge whose
     *          adjacent facets form a dihedral angle smaller than @p sharp_angle (measured
     *          as the angle between the facet normals). Finally calls
     *          fix_vertices_based_on_fixed_edges() to fix the associated vertices.
     * @param[in] sharp_angle Dihedral angle threshold in radians; edges sharper than this
     *                        value are fixed. Defaults to 0.75*M_PI (135 degrees).
     */
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

    /**
     * @brief Labels all vertices that lie on a boundary edge.
     * @details Resets the boundary flag on every vertex, then scans all facets and marks the
     *          two endpoints of each boundary (adjacent-free) edge as boundary vertices.
     */
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

    /**
     * @brief Labels non-manifold vertices and fixes them.
     * @details Runs detect_non_manifold_vertices() to collect all non-manifold vertices,
     *          marks them in the manager's non-manifold attribute, and calls fix_vertex()
     *          on each so that collapse operations touching them cannot corrupt the mesh.
     *          Logs a warning with the number of detected vertices.
     */
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

    /**
     * @brief Fixes vertices whose incident fixed edges make them unsafe to move.
     * @details Collects the (up to two) fixed-edge neighbours of every vertex. A vertex is
     *          fixed when it touches more than two fixed edges, touches exactly one fixed
     *          edge, or when its two fixed-edge neighbours form an angle smaller than
     *          @p sharp_angle (a sharp corner). Handles both 2D and 3D meshes.
     * @param[in] sharp_angle Angle threshold in radians used to detect sharp corners formed
     *                         by two fixed edges. Defaults to 0.75*M_PI (135 degrees).
     */
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
