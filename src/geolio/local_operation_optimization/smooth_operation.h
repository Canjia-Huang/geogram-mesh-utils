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
        /**
         * @brief Constructs a SmoothOperation for relaxing vertex positions.
         * @details Initializes the base operation. For 3D meshes it keeps a copy of the input
         *          mesh (original_mesh_) and builds a GEO::MeshFacetsAABB over it so smoothed
         *          vertices can be projected back onto the original surface. 2D meshes skip the
         *          copy because no projection is needed.
         * @param[in] mesh_element_manager The mesh element manager exposing the mesh and its
         *                                 usage/fixed element attributes.
         */
        explicit SmoothOperation(
            MeshElementManager<DIM>& mesh_element_manager,
            bool project_to_original_mesh = true,
            bool allow_smooth_fixed_edge_vertices = false);

        double do_once();

        void run_nb_times(GEO::index_t iterations_nb);

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

        bool PROJECT_TO_ORIGINAL_MESH_;
        GEO::Mesh original_mesh_; // a copy of original input mesh, only used in 3D mesh
        GEO::MeshFacetsAABB original_mesh_facet_AABB_; // only used in 3D mesh

        bool ALLOW_SMOOTH_FIXED_EDGE_VERTICES_; // When true, vertices lying on fixed edges are smoothed by sliding along those edges.
        std::vector<char> mesh_v_on_fixed_edges_; // used when ALLOW_SMOOTH_FIXED_EDGE_VERTICES_ == false
        std::vector<std::pair<GEO::index_t, GEO::index_t>> mesh_fixed_edge_v_adjacent_v; /*
            used when ALLOW_SMOOTH_FIXED_EDGE_VERTICES_ == true
            v -> adjacent vertices along adjacent fixed edges
        */

        std::vector<std::vector<GEO::index_t>> mesh_v_adjacent_v; // v -> adjacent vertices

        std::vector<GEO::vecng<DIM, GEO::Numeric::float64>> mesh_v_new_pos; // just pre-allocated
    };

    extern template class SmoothOperation<2>;
    extern template class SmoothOperation<3>;
}

#endif //GEOLIO_SMOOTH_OPERATION_H
