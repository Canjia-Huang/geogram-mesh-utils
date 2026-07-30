//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_TRI_LOCAL_OPERATION_OPTIMIZATION_H
#define GEOLIO_TRI_LOCAL_OPERATION_OPTIMIZATION_H

#include <cassert>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_AABB.h>

namespace geolio
{
    /**
     * @brief Performs local optimization on a triangular mesh.
     *
     * The optimization pipeline adjusts mesh connectivity and vertex positions
     * by splitting, collapsing, swapping, and smoothing edges while preserving
     * the original surface when working in 3D.
     */
    /**
     * @class TriLocalOperationOptimization
     * @brief Performs local optimization on triangular meshes.
     *
     * @details
     * The optimizer implements a classical local operations pipeline used in
     * remeshing: edge splitting for long edges, edge collapsing for short
     * edges, edge swapping to improve valence, and vertex smoothing to
     * improve element quality. When operating on 3D surface meshes, vertex
     * smoothing projects updated positions back onto the original surface to
     * preserve geometry.
     */
    class TriLocalOperationOptimization {
    public:
        /**
         * @brief Constructs an optimizer bound to the given mesh.
         *
         * @param[in,out] mesh The mesh to be optimized in place. The optimizer
         * keeps a reference to the mesh and modifies it during optimization.
         *
         * @note The constructor copies the current mesh geometry to
         * `original_mesh_` to enable projection of smoothed vertices back onto
         * the initial surface for 3D meshes.
         */
        explicit TriLocalOperationOptimization(GEO::Mesh& mesh);

        /**
         * @brief Destroys the optimizer and releases bound temporary resources.
         *
         * @details
         * If temporary mesh attributes are still bound, they are released as
         * part of object teardown. The referenced mesh itself is not owned and
         * is not destroyed by this class.
         */
        ~TriLocalOperationOptimization();

        /**
         * @brief Runs the local optimization pipeline.
         *
         * @param[in] target_edge_length Desired target edge length used to
         * guide split/collapse decisions. If negative, the optimizer computes
         * an automatic target using compute_average_mesh_edge_length().
         * @param[in] rounds_nb Number of optimization rounds to execute. Each
         * round runs split, collapse, swap and smooth passes in that order.
         *
         * @details
         * The method performs the following steps per round:
         * - bind_attributes() to create temporary usage attributes
         * - split_edges(target_edge_length * 1.5) to refine long edges
         * - collapse_edges(target_edge_length * 0.5) to remove short edges
         * - swap_edges() to improve mesh valence
         * - smooth_vertices(1) to relax vertex positions
         * - clean_unused_elements() to remove released facets
         * After the requested rounds, unbind_attributes() is called to clean up.
         */
        void optimize(
            double target_edge_length = -1,
            GEO::index_t rounds_nb = 5);

        /**
         * @brief Marks a specific vertex as fixed.
         *
         * @param[in] v Vertex index to lock.
         *
         * @details
         * A fixed vertex is excluded from smoothing and from local operations
         * that would move it. The caller must ensure `v` is a valid vertex
         * index in `mesh_`.
         */
        void fix_vertex(
            const GEO::index_t v
            ) {
            assert(mesh_v_fixed_.is_bound());
            assert(v < mesh_.vertices.nb());
            mesh_v_fixed_[v] = true;
        }

        void fix_edge(
            const GEO::index_t f,
            const GEO::index_t lv
            ) {
            assert(f < mesh_.facets.nb());
            assert(lv < 3);
            mesh_fc_fixed_[mesh_.facets.corner(f, lv)] = true;
        }

        void fix_edge(
            const GEO::index_t fc
            ) {
            assert(fc < mesh_.facet_corners.nb());
            mesh_fc_fixed_[fc] = true;
        }

        void fix_boundary_edges();

    protected:
        /**
         * @brief Binds temporary usage attributes on mesh vertices and facets.
         *
         * @details
         * Creates boolean attributes `mesh_v_used_` and `mesh_f_used_` on the
         * mesh and initializes them to true for all existing vertices and
         * facets. These attributes are used to track which elements are
         * logically alive during partial element removal, while allowing the
         * mesh arrays to be reused.
         */
        void bind_attributes();

        /**
         * @brief Releases the temporary usage attributes from the mesh.
         *
         * @details
         * Removes the attributes created by bind_attributes() and clears the
         * `free_vertices_` and `free_facets_` lists. Should be called after
         * optimization to restore the mesh to a clean state.
         */
        void unbind_attributes();

        /**
         * @brief Detects and labels boundary vertices in the current mesh.
         *
         * @details
         * Updates `mesh_v_boundary_` so each vertex indicates whether it lies
         * on a boundary edge. This label is used by fixing and local operation
         * passes to preserve boundary shape and topology.
         */
        void label_boundary_vertices();

        void label_non_manifold_vertices();

        /**
         * @brief Compute the length of the edge opposite local vertex `lv` in facet `f`.
         *
         * @param[in] f Facet index containing the edge.
         * @param[in] lv Local index (0..2) of the vertex opposite the edge.
         * @return Edge length as Euclidean distance between the two edge vertices.
         *
         * @details
         * The function fetches the two vertex indices composing the requested
         * edge from the facet and computes the Euclidean distance in the
         * mesh coordinate space.
         */
        [[nodiscard]] double get_edge_length(
            GEO::index_t f,
            GEO::index_t lv) const;

        /**
         * @brief Computes the average edge length of the current mesh.
         *
         * @return The arithmetic mean of all triangle edge lengths.
         *
         * @details
         * Iterates over all facets, accumulates lengths of their three edges
         * (each edge counted once overall) and divides by the total number
         * of distinct edges. Used when the optimizer must infer a default
         * target edge length.
         */
        [[nodiscard]] double compute_average_mesh_edge_length() const;

        /**
         * @brief Acquires an available vertex slot, allocating more if needed.
         *
         * @return The index of a reusable or newly allocated vertex.
         *
         * @details
         * If `free_vertices_` is not empty, the last index is popped and
         * returned. Otherwise, allocate_new_vertices() is called to expand the
         * vertex buffer and populate the free list; a new index is then
         * returned. The returned vertex should be initialized by the caller.
         */
        [[nodiscard]] GEO::index_t require_a_new_vertex();

        void disuse_a_vertex(GEO::index_t v);

        /**
         * @brief Acquires an available facet slot, allocating more if needed.
         *
         * @return The index of a reusable or newly allocated facet.
         *
         * @details
         * Mirrors require_a_new_vertex() behavior for facets: reuse from
         * `free_facets_` if possible, otherwise allocate_new_facets() and
         * return a fresh index.
         */
        [[nodiscard]] GEO::index_t require_a_new_facet();

        void disuse_a_facet(GEO::index_t f);

        /**
         * @brief Expands the vertex buffer and records the new slots as free.
         *
         * @details
         * Allocates as many new vertices as the current vertex count, thereby
         * doubling the capacity. The indices of the newly created vertices
         * are pushed into `free_vertices_` and their `mesh_v_used_` entries
         * are set to false.
         */
        void allocate_new_vertices();

        /**
         * @brief Expands the facet buffer and records the new slots as free.
         *
         * @details
         * Allocates additional facet slots equal to the current number of
         * facets, pushes their indices into `free_facets_` and marks them
         * unused. This strategy amortizes allocation cost across operations.
         */
        void allocate_new_facets();

        /**
         * @brief Splits edges longer than the provided length threshold.
         *
         * @param[in] limit_edge_length Maximum edge length allowed before splitting.
         *
         * @details
         * The pass scans facets and identifies edges whose length exceeds
         * `limit_edge_length`. For each such edge, if splitting is topologically
         * valid (does not create degenerate elements or violate fixed-vertex
         * constraints), a new vertex is created at the midpoint, incident
         * facets are updated or duplicated, and the involved old facets are
         * marked as processed to avoid reprocessing in the same round.
         */
        void split_edges(double limit_edge_length);

        /**
         * @brief Collapses edges shorter than the provided length threshold.
         *
         * @param[in] limit_edge_length Minimum edge length allowed before collapse.
         *
         * @details
         * The pass identifies short edges and attempts to collapse them into
         * a single vertex when the collapse preserves mesh manifoldness and
         * element quality. On success, removed vertices/facets are released
         * via disuse_a_vertex()/disuse_a_facet() and recorded for later cleanup.
         */
        void collapse_edges(double limit_edge_length);

        /**
         * @brief Swaps edges that improve local valence quality.
         *
         * @details
         * Evaluates the local valence score before and after the potential
         * edge flip (typically sum of squared deviations from ideal valence)
         * and performs the flip only if the swap does not worsen the score.
         * Boundary and fixed-vertex constraints are respected.
         */
        void swap_edges();

        /**
         * @brief Smooths vertex positions by repeated neighborhood averaging.
         *
         * @param[in] iterations_nb Number of smoothing iterations to run.
         *
         * @details
         * Implements a Laplacian-like smoothing: each non-fixed vertex is
         * moved to the average of its neighboring vertices. For surface
         * (3D) meshes, the computed position is projected back onto the
         * original surface using `original_mesh_facet_AABB_` to find the
         * closest point on the initial facets. The method runs
         * `iterations_nb` passes of this relaxation.
         */
        void smooth_vertices(GEO::index_t iterations_nb) const;

        /**
         * @brief Deletes mesh elements that were released during optimization.
         *
         * @details
         * Iterates over `mesh_f_used_` and removes facets marked as unused
         * from the mesh data structures (calls mesh_.facets.delete_one() or
         * similar). Vertex storage is not physically shrunk here; freed
         * vertex indices remain in `free_vertices_` for reuse by future
         * operations.
         */
        void clean_unused_elements();

        GEO::Mesh& mesh_;
        const bool mesh_2d_; // mesh.vertices.dimension() == 2

        const std::string attribute_name_; // Prevent anyone from using these attributes externally (unsafety).
        GEO::Attribute<bool> mesh_v_boundary_; // v -> on boundary
        GEO::Attribute<bool> mesh_v_fixed_; // v -> fixed
        GEO::Attribute<bool> mesh_v_non_manifold_; // v -> non manifold
        GEO::Attribute<bool> mesh_v_used_; // v -> used
        GEO::Attribute<bool> mesh_f_processed_; // f -> processed (just pre-allocated)
        GEO::Attribute<bool> mesh_f_used_; // f -> used
        GEO::Attribute<bool> mesh_fc_fixed_; // fc (edge) -> fixed

        GEO::Mesh original_mesh_; // a copy of original input mesh
        GEO::MeshFacetsAABB original_mesh_facet_AABB_;

        std::vector<GEO::index_t> free_vertices_;
        std::vector<GEO::index_t> free_facets_;

        bool ALLOW_SPLIT_FIXED_EDGES_               = false;
        bool ALLOW_COLLAPSE_FIXED_EDGES_            = false;
        bool ALLOW_SMOOTH_FIXED_EDGE_VERTICES_      = false;
    };
}

#endif //GEOLIO_TRI_LOCAL_OPERATION_OPTIMIZATION_H
