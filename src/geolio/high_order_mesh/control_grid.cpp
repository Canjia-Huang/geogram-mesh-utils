//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "control_grid.h"
#include "node_positions.h"
#include <cassert>

namespace geolio
{
    ControlGrid::ControlGrid(
        const GEO::Mesh& mesh,
        const GEO::index_t order
        ) : mesh_(mesh), ORDER_(order)
    {
        assert(ORDER_ > 0);

        initialize_node_positions_1D();
    }

    void ControlGrid::set_nodes_type(
        const NodesType nodes_type
        ) {
        if (nodes_type != nodes_type_)
            initialize_node_positions_1D();
    }

    void ControlGrid::initialize_node_positions_1D(
        ) {
        switch (nodes_type_) {
            case NodesType::EQUALLY_SPACED_NODES:
                compute_equally_spaced_nodes(ORDER_, node_positions_1D_);
                break;
            case NodesType::CHEBYSHEV_GAUSS:
                compute_Chebyshev_Gauss_nodes(ORDER_, node_positions_1D_);
                break;
            case NodesType::CHEBYSHEV_GAUSS_LOBATTO:
                compute_Chebyshev_Gauss_Lobatto_nodes(ORDER_, node_positions_1D_);
                break;
            case NodesType::LEGENDRE_GAUSS_LOBATTO:
                compute_Legendre_Gauss_Lobatto_nodes(ORDER_, node_positions_1D_);
                break;
            default:
                assert(0);
        }
    }
}