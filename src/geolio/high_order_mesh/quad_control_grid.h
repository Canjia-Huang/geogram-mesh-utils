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
