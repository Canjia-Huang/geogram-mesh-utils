//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_TRI_LOCAL_OPERATION_OPTIMIZATION_H
#define GEOLIO_TRI_LOCAL_OPERATION_OPTIMIZATION_H

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
    class TriLocalOperationOptimization {
    public:
        /**
         * @brief Constructs an optimizer bound to the given mesh.
         *
         * @param[in, out] mesh The mesh to be optimized in place.
         */
        explicit TriLocalOperationOptimization(GEO::Mesh& mesh);

        /**
         * @brief Runs the local optimization pipeline.
         *
         * The method derives a target edge length when needed, prepares
         * temporary mesh attributes, then repeatedly splits, collapses,
         * swaps, and smooths edges before removing unused mesh elements.
         *
         * @param[in] target_edge_length Desired average edge length. If negative,
         * the value is computed from the current mesh.
         * @param[in] rounds_nb Number of optimization rounds to execute.
         */
        void optimize(
            double target_edge_length = -1,
            GEO::index_t rounds_nb = 5);

    private:
        /**
         * @brief Binds temporary usage attributes on mesh vertices and facets.
         *
         * The attributes are initialized to mark all elements as currently used.
         */
        void bind_attributes();

        /**
         * @brief Releases the temporary usage attributes from the mesh.
         *
         * This is called after optimization completes to clean up the bound
         * attributes.
         */
        void unbind_attributes();

        /**
         * @brief Computes the average edge length of the current mesh.
         *
         * @return The arithmetic mean of all triangle edge lengths.
         */
        [[nodiscard]] double compute_average_mesh_edge_length() const;

        /**
         * @brief Acquires an available vertex slot, allocating more if needed.
         *
         * @return The index of a reusable or newly allocated vertex.
         */
        [[nodiscard]] GEO::index_t require_a_new_vertex();

        /**
         * @brief Marks a vertex slot as unused so it can be reused later.
         *
         * @param[in] v The vertex index to release. GEO::NO_VERTEX is ignored.
         */
        void disuse_a_vertex(GEO::index_t v);

        /**
         * @brief Acquires an available facet slot, allocating more if needed.
         *
         * @return The index of a reusable or newly allocated facet.
         */
        [[nodiscard]] GEO::index_t require_a_new_facet();

        /**
         * @brief Marks a facet slot as unused so it can be reused later.
         *
         * @param[in] f The facet index to release. GEO::NO_FACET is ignored.
         */
        void disuse_a_facet(GEO::index_t f);

        /**
         * @brief Expands the vertex buffer and records the new slots as free.
         *
         * The method doubles the current vertex capacity by creating an equal
         * number of new vertices and pushing their indices into the free list.
         */
        void allocate_new_vertices();

        /**
         * @brief Expands the facet buffer and records the new slots as free.
         *
         * The method doubles the current facet capacity by creating an equal
         * number of new triangles and pushing their indices into the free list.
         */
        void allocate_new_facets();

        /**
         * @brief Splits edges longer than the provided length threshold.
         *
         * Each eligible long edge is split once per pass, and the affected
         * facets are marked so they are not processed again in the same round.
         *
         * @param[in] limit_edge_length Maximum edge length allowed before splitting.
         */
        void split_edges(double limit_edge_length);

        /**
         * @brief Collapses edges shorter than the provided length threshold.
         *
         * Each eligible short edge is collapsed when the collapse is valid,
         * then released vertices and facets are returned to the free lists.
         *
         * @param[in] limit_edge_length Minimum edge length allowed before collapse.
         */
        void collapse_edges(double limit_edge_length);

        /**
         * @brief Swaps edges that improve local valence quality.
         *
         * The method evaluates each valid swap and performs it only when the
         * valence score after swapping is no worse than before.
         */
        void swap_edges() const;

        /**
         * @brief Smooths vertex positions by repeated neighborhood averaging.
         *
         * In 2D, the averaged positions are applied directly. In 3D, the
         * averaged positions are projected back onto the original surface.
         *
         * @param[in] iterations_nb Number of smoothing iterations to run.
         */
        void smooth_vertices(GEO::index_t iterations_nb) const;

        /**
         * @brief Deletes mesh elements that were released during optimization.
         *
         * Only facets are explicitly removed here; unused vertices remain as
         * reusable storage for future operations.
         */
        void clean_unused_elements() const;

        GEO::Mesh& mesh_;
        GEO::Attribute<bool> mesh_v_used_;
        GEO::Attribute<bool> mesh_f_used_;

        GEO::Mesh original_mesh_;
        GEO::MeshFacetsAABB original_mesh_facet_AABB_;

        std::vector<GEO::index_t> free_vertices_;
        std::vector<GEO::index_t> free_facets_;
    };
}

#endif //GEOLIO_TRI_LOCAL_OPERATION_OPTIMIZATION_H
