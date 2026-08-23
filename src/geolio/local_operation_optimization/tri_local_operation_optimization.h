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
#include "swap_operation.h"
#include "smooth_operation.h"

namespace geolio
{
    template <GEO::index_t DIM>
    class TriLocalOperationOptimization {
    public:
        /**
         * @brief Constructs a TriLocalOperationOptimization over the given triangle mesh.
         * @details Stores the mesh reference, builds the internal MeshElementManager, and
         *          labels boundary and non-manifold vertices so subsequent operations can
         *          preserve or fix them. When optional original-index attributes are provided,
         *          they are initialized so that every current vertex/facet maps to its own
         *          index; the first element receives a special marker so it can be recovered
         *          after optimization.
         * @param[in] mesh The triangle mesh to optimize; only simplex facets are supported.
         */
        explicit TriLocalOperationOptimization(GEO::Mesh& mesh);

        /**
         * @brief Runs the local optimization pipeline.
         * @details If target_edge_length is negative it is derived automatically from
         *          compute_average_mesh_edge_length(). The split and collapse thresholds are
         *          then set to 4/3 and 4/5 of that target, and fixed-edge operations are enabled
         *          for all four operations. For each round a SplitOperation, CollapseOperation,
         *          SwapOperation and SmoothOperation are created and their run_through() methods
         *          are invoked in split, collapse, swap, smooth order (smooth runs 3 iterations
         *          via run_nb_times()). After all rounds, clean_unused_elements() removes the
         *          disused facets and isolated vertices, and any optional original-index
         *          attributes are finalized (newly created elements are set to GEO::NO_INDEX).
         * @param[in] rounds_nb Number of optimization rounds to execute. Each round runs split,
         *                      collapse, swap and smooth passes in that order.
         * @param[in] target_edge_length Desired target edge length used to guide split/collapse
         *                               decisions. If negative, the optimizer computes an
         *                               automatic target via compute_average_mesh_edge_length().
         */
        void optimize(
            GEO::index_t rounds_nb = 5,
            double target_edge_length = -1);

        /** Enables split operations to act on edges that are marked as fixed. */
        bool allow_split_fixed_edges = true;

        /** Enables collapse operations to act on edges that are marked as fixed. */
        bool allow_collapse_fixed_edges = true;

        /**
         * Criterion used by swap operations to decide whether an edge should be flipped.
         * Possible values:
         * - SwapOperation<DIM>::SWAP_BASED_ON_VALENCE: accept swaps that improve vertex valence.
         * - SwapOperation<DIM>::SWAP_BASED_ON_DELAUNAY: accept swaps that make the edge locally Delaunay.
         * These are bit flags, so they can be combined with bitwise OR.
         */
        GEO::index_t swap_criterion = SwapOperation<DIM>::SWAP_BASED_ON_DELAUNAY;

        /**
         * Geometric constraint used by smoothing to project updated vertices.
         * Possible values:
         * - SmoothOperation<DIM>::NONE: plain Laplacian smoothing without extra geometric constraints.
         * - SmoothOperation<DIM>::PROJECT_TO_ORIGINAL_MESH: project the new position onto the original
         *   input surface (3D only).
         * - SmoothOperation<DIM>::TANGENTIAL_SMOOTHING: keep only the tangential displacement (3D only).
         */
        GEO::index_t smooth_geometric_constraint = SmoothOperation<DIM>::PROJECT_TO_ORIGINAL_MESH;

        /** Allows smoothing to move vertices that lie on fixed edges. */
        bool allow_smooth_fixed_edges_vertices = true;

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
            const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                mesh_.facets.vertex(f, lv),
                mesh_.facets.vertex(f, (lv+1)%3));
            manager_.fixed_edges_.insert(edge);
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

        /**
          * @brief Computes the average edge length of the current mesh.
          * @details Delegates to the internal MeshElementManager to accumulate edge lengths
          *          across the mesh and divide by the number of edges, yielding a global scale
          *          estimate used to initialize split/collapse targets in the optimization.
          * @return Mean length of all mesh edges in the current topology.
          */
        double compute_average_edge_length();

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
        MeshElementManager<DIM> manager_;
    };

    extern template class TriLocalOperationOptimization<2>;
    extern template class TriLocalOperationOptimization<3>;
}

#endif //GEOLIO_TRI_LOCAL_OPERATION_OPTIMIZATION_H
