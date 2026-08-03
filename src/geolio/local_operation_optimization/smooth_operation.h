//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_SMOOTH_OPERATION_H
#define GEOLIO_SMOOTH_OPERATION_H

#include "base_operation.h"
#include <geogram/mesh/mesh_AABB.h>

namespace geolio
{
    template<GEO::index_t DIM>
    class SmoothOperation : public BaseOperation<DIM> {
    public:
        /** @brief Controls how smoothed vertex positions are constrained.
         *         @details NONE applies plain Laplacian smoothing; PROJECT_TO_ORIGINAL_MESH
         *                  snaps each new position onto the original input surface (3D only);
         *                  TANGENTIAL_SMOOTHING keeps only the tangential displacement of each
         *                  vertex (3D only). */
        enum SmoothGeometricConstraint {
            NONE,
            PROJECT_TO_ORIGINAL_MESH,
            TANGENTIAL_SMOOTHING
        };

        /**
         * @brief Constructs a SmoothOperation for relaxing vertex positions.
         * @details Initializes the base operation, builds the one-ring vertex adjacency from
         *          the used facets, and prepares the fixed-edge data: either the vertices lying
         *          on fixed edges (when sliding is disallowed) or, per vertex, the two neighbours
         *          along adjacent fixed edges (when sliding is allowed). Pre-determines which
         *          vertices are eligible to move via is_perform_valid(). For 3D meshes with
         *          PROJECT_TO_ORIGINAL_MESH, it keeps a copy of the used input surface and
         *          builds a GEO::MeshFacetsAABB over it so smoothed vertices can be projected
         *          back onto the original surface.
         * @param[in] mesh_element_manager The mesh element manager exposing the mesh and its
         *                                 usage/fixed element attributes.
         * @param[in] geometric_constraint One of the SmoothGeometricConstraint values selecting
         *                                 how new positions are constrained. Defaults to NONE.
         * @param[in] allow_smooth_fixed_edge_vertices When true, vertices incident to fixed
         *                                              edges are smoothed by sliding along those
         *                                              edges; when false they are kept fixed.
         *                                              Defaults to true.
         */
        explicit SmoothOperation(
            MeshElementManager<DIM>& mesh_element_manager,
            GEO::index_t geometric_constraint = PROJECT_TO_ORIGINAL_MESH,
            bool allow_smooth_fixed_edge_vertices = true);

        /**
         * @brief Performs a single smoothing iteration over every movable vertex.
         * @details Computes each vertex's target position as the average of its neighbours, then
         *          applies the configured geometric constraints (sliding along fixed edges,
         *          projection onto the original mesh, or tangential smoothing) and a damping
         *          factor, and finally commits the new positions. Always returns true.
         * @return true after a full smoothing iteration.
         */
        double do_once();

        /**
         * @brief Runs a fixed number of smoothing iterations over the mesh vertices.
         * @param[in] iterations_nb Number of do_once() iterations to execute.
         */
        void run_nb_times(GEO::index_t iterations_nb);

        /**
         * @brief Runs smoothing iterations until the per-iteration displacement falls below a
         *        threshold.
         * @details Repeatedly calls do_once() while the displacement it returns exceeds
         *          @p displacement_threshold.
         * @param[in] displacement_threshold Maximum acceptable per-iteration vertex
         *                                   displacement. Defaults to 0.1.
         */
        void run_until(double displacement_threshold = 0.1);

    private:
        /**
         * @brief Checks whether vertex @p v is allowed to move during smoothing.
         * @details Returns false when the vertex is no longer in use or is marked as fixed;
         *          returns true otherwise.
         * @param[in] v Index of the vertex to test.
         * @return true if the vertex may be moved; false otherwise.
         */
        [[nodiscard]] bool is_perform_valid(GEO::index_t v) const;

        const GEO::index_t GEOMETRIC_CONSTRAINT_;
        GEO::Mesh original_mesh_; // a copy of original input mesh, only used in 3D mesh
        GEO::MeshFacetsAABB original_mesh_facet_AABB_; // only used in 3D mesh

        const bool ALLOW_SMOOTH_FIXED_EDGE_VERTICES_; // When true, vertices lying on fixed edges are smoothed by sliding along those edges.
        std::vector<char> mesh_v_on_fixed_edges_; // used when ALLOW_SMOOTH_FIXED_EDGE_VERTICES_ == false
        std::vector<std::pair<GEO::index_t, GEO::index_t>> mesh_fixed_edge_v_adjacent_v; /*
            used when ALLOW_SMOOTH_FIXED_EDGE_VERTICES_ == true
            v -> adjacent vertices along adjacent fixed edges
        */

        double damping_factor_ = 0.5;

        std::vector<char> mesh_v_perform_; // v -> to process
        std::vector<std::vector<GEO::index_t>> mesh_v_adjacent_v; // v -> adjacent vertices

        std::vector<GEO::vecng<DIM, GEO::Numeric::float64>> mesh_v_new_pos; // just pre-allocated
    };

    extern template class SmoothOperation<2>;
    extern template class SmoothOperation<3>;
}

#endif //GEOLIO_SMOOTH_OPERATION_H
