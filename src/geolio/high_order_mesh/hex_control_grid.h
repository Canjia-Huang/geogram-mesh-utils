//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_HEX_CONTROL_GRID_H
#define GEOLIO_HEX_CONTROL_GRID_H
#include "control_grid.h"

namespace geolio
{
    class HexControlGrid : public VolumeControlGrid {
    public:
        HexControlGrid(const GEO::Mesh& mesh, GEO::index_t order);

    protected:
        void initialize_nodes_arrangement() override;

        void initialize_control_nodes() override;
    };
}

#endif //GEOLIO_HEX_CONTROL_GRID_H
