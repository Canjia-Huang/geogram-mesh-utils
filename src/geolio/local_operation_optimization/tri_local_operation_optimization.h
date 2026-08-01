//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_TRI_LOCAL_OPERATION_OPTIMIZATION_H
#define GEOLIO_TRI_LOCAL_OPERATION_OPTIMIZATION_H

#include <cassert>
#include <geogram/mesh/mesh.h>
#include "collapse_operation.h"
#include "mesh_element_manager.h"

namespace geolio
{
    class TriLocalOperationOptimization {
    public:
        /**
         * @brief Constructs a TriLocalOperationOptimization over the given triangle mesh.
         * @details Stores the mesh reference, builds the internal MeshElementManager, and
         *          labels boundary and non-manifold vertices so subsequent operations can
         *          preserve or fix them.
         * @param[in] mesh The triangle mesh to optimize; only simplex facets are supported.
         */
        explicit TriLocalOperationOptimization(
            GEO::Mesh& mesh,
            GEO::Attribute<GEO::index_t>* mesh_v_original_idx = nullptr,
            GEO::Attribute<GEO::index_t>* mesh_f_original_idx = nullptr);


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
        void optimize(
            GEO::index_t rounds_nb = 5,
            double target_edge_length = -1);

        /**
         * @brief Marks a specific vertex as fixed.
         * @details Sets the manager's fixed flag for the vertex, so it is excluded from
         *          smoothing and from local operations that would move it. The caller must
         *          ensure @p v is a valid vertex index in mesh_.
         * @param[in] v Vertex index to lock.
         */
        void fix_vertex(
            const GEO::index_t v
            ) {
            assert(v < mesh_.vertices.nb());
            manager_.mesh_v_fixed[v] = true;
        }

        /**
         * @brief Fixes (locks) the mesh edge of facet @p f at local vertex @p lv.
         * @details Sets the fixed-corner flag on the facet corner of the edge, and, when the
         *          edge is interior, also sets the matching fixed-corner flag on the adjacent
         *          facet so the whole edge is locked from both sides.
         * @param[in] f Index of a facet adjacent to the edge to fix.
         * @param[in] lv Local vertex index identifying the oriented edge (lv -> lv+1).
         */
        void fix_edge(
            const GEO::index_t f,
            const GEO::index_t lv
            ) {
            assert(f < mesh_.facets.nb());
            assert(lv < 3);
            manager_.mesh_fc_fixed[mesh_.facets.corner(f, lv)] = true;
            if (const auto nf = mesh_.facets.adjacent(f, lv);
                nf != GEO::NO_FACET) {
                const auto v = mesh_.facets.vertex(f, lv);
                const auto nlv = mesh_.facets.find_vertex(nf, v);
                assert(nlv != GEO::NO_INDEX);
                manager_.mesh_fc_fixed[mesh_.facets.corner(nf, (nlv+2)%3)] = true;
            }
        }

        /**
         * @brief Fixes (locks) all boundary edges of the mesh.
         * @details Iterates over every facet corner and fixes each edge that has no adjacent
         *          facet (a boundary edge), skipping corners already fixed. Afterwards calls
         *          fix_vertices_based_on_fixed_edges() so that boundary vertices are fixed too.
         */
        void fix_boundary_elements();

        /**
         * @brief Fixes (locks) edges whose dihedral angle is sharp.
         * @details Pre-computes the normal of every facet, then fixes each interior edge whose
         *          adjacent facets form a dihedral angle smaller than @p sharp_angle (measured
         *          as the angle between the facet normals). Finally calls
         *          fix_vertices_based_on_fixed_edges() to fix the associated vertices.
         * @param[in] sharp_angle Dihedral angle threshold in radians; edges sharper than this
         *                        value are fixed. Defaults to 0.75*M_PI (135 degrees).
         */
        void fix_sharp_elements(double sharp_angle = 0.75*M_PI);

    protected:
        /**
         * @brief Labels all vertices that lie on a boundary edge.
         * @details Resets the boundary flag on every vertex, then scans all facets and marks the
         *          two endpoints of each boundary (adjacent-free) edge as boundary vertices.
         */
        void label_boundary_vertices();

        /**
         * @brief Labels non-manifold vertices and fixes them.
         * @details Runs detect_non_manifold_vertices() to collect all non-manifold vertices,
         *          marks them in the manager's non-manifold attribute, and calls fix_vertex()
         *          on each so that collapse operations touching them cannot corrupt the mesh.
         *          Logs a warning with the number of detected vertices.
         */
        void label_non_manifold_vertices();

        /**
         * @brief Fixes vertices whose incident fixed edges make them unsafe to move.
         * @details Collects the (up to two) fixed-edge neighbours of every vertex. A vertex is
         *          fixed when it touches more than two fixed edges, touches exactly one fixed
         *          edge, or when its two fixed-edge neighbours form an angle smaller than
         *          @p sharp_angle (a sharp corner). Handles both 2D and 3D meshes.
         * @param[in] sharp_angle Angle threshold in radians used to detect sharp corners formed
         *                         by two fixed edges. Defaults to 0.75*M_PI (135 degrees).
         */
        void fix_vertices_based_on_fixed_edges(double sharp_angle = 0.75*M_PI);

        GEO::Mesh& mesh_;
        MeshElementManager manager_;

        GEO::Attribute<GEO::index_t>* mesh_v_original_idx_ = nullptr;
        GEO::Attribute<GEO::index_t>* mesh_f_original_idx_ = nullptr;
    };
}

#endif //GEOLIO_TRI_LOCAL_OPERATION_OPTIMIZATION_H
