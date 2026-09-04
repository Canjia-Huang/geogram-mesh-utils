//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/4.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef HOSM_QUAD_CONTROL_GRID_H
#define HOSM_QUAD_CONTROL_GRID_H
#include "surf_control_grid.h"

namespace geolio
{
    template<GEO::index_t DIM>
    class QuadControlGrid : public SurfaceControlGrid<DIM> {
    public:
        QuadControlGrid(const GEO::Mesh& mesh, GEO::index_t order);

        void append_discretized_high_order_facets(
            GEO::Mesh& mesh_out,
            GEO::index_t resolution = 10,
            GEO::Attribute<GEO::index_t>* mesh_out_v_facet = nullptr,
            GEO::Attribute<GEO::vec2>* mesh_out_v_uv = nullptr,
            GEO::Attribute<GEO::index_t>* mesh_out_f_facet = nullptr) const;

    protected:
        /**
             * @brief Initialize local indexing/layout rules for hexahedral control nodes.
             */
        void initialize_nodes_arrangement() override;

        /**
         * @brief Build global control-node coordinates and cell-to-control-node connectivity.
         */
        void initialize_control_nodes() override;
    };

    extern template class QuadControlGrid<2>;
    extern template class QuadControlGrid<3>;
}

#endif //HOSM_QUAD_CONTROL_GRID_H
