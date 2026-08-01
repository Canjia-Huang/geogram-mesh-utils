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
    class SmoothOperation : public BaseOperation {
    public:
        /**
         * @brief Constructs a SmoothOperation for relaxing vertex positions.
         * @details Initializes the base operation. For 3D meshes it keeps a copy of the input
         *          mesh (original_mesh_) and builds a GEO::MeshFacetsAABB over it so smoothed
         *          vertices can be projected back onto the original surface. 2D meshes skip the
         *          copy because no projection is needed.
         * @param[in] mesh_element_manager The mesh element manager exposing the mesh and its
         *                                 usage/fixed element attributes.
         */
        explicit SmoothOperation(MeshElementManager& mesh_element_manager);

        /**
         * @brief Runs a number of smoothing iterations over the mesh vertices.
         * @details Builds the one-ring adjacency of every used vertex from the used facets, and
         *          optionally the set of vertices/edges adjacent to fixed edges. For each
         *          iteration it computes each movable vertex's target position as the average of
         *          its neighbours, projects it onto the original surface for 3D meshes, slides
         *          it along adjacent fixed edges when allowed, and writes it back if
         *          is_perform_valid() passes.
         * @param[in] iterations_nb Number of smoothing iterations to execute.
         */
        void perform_one_pass(GEO::index_t iterations_nb) const;

        /** @brief When true, vertices lying on fixed edges are smoothed by sliding along those edges. */
        bool ALLOW_SMOOTH_FIXED_EDGE_VERTICES = false;

    private:
        /**
         * @brief Checks whether vertex @p v is allowed to move during smoothing.
         * @details Returns false when the vertex is no longer in use or is marked as fixed;
         *          returns true otherwise.
         * @param[in] v Index of the vertex to test.
         * @return true if the vertex may be moved; false otherwise.
         */
        [[nodiscard]] bool is_perform_valid(GEO::index_t v) const;

        GEO::Mesh original_mesh_; // a copy of original input mesh
        GEO::MeshFacetsAABB original_mesh_facet_AABB_;
    };
}

#endif //GEOLIO_SMOOTH_OPERATION_H
