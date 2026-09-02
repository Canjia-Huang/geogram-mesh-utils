//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef HOSM_SURF_CONTROL_GRID_H
#define HOSM_SURF_CONTROL_GRID_H
#include "control_grid.h"

namespace geolio
{
    class SurfaceControlGrid : public ControlGrid {
    public:
        /**
         * @brief Construct a surface control grid.
         * @param[in] mesh Input surface mesh.
         * @param[in] order Polynomial order of the high-order surface representation.
         */
        SurfaceControlGrid(const GEO::Mesh& mesh, const GEO::index_t order) : ControlGrid(mesh, order) {}

    protected:
        /**
         * @brief Initialize local indexing/layout rules for surface control nodes.
         */
        virtual void initialize_nodes_arrangement() = 0;
        GEO::index_t CONTROL_POINTS_NB_PER_EDGE = GEO::NO_INDEX;
        GEO::index_t CONTROL_POINTS_NB_PER_FACET = GEO::NO_INDEX;
        GEO::index_t INTERNAL_CONTROL_POINTS_NB_PER_EDGE = GEO::NO_INDEX;
        GEO::index_t INTERNAL_CONTROL_POINTS_NB_PER_FACET = GEO::NO_INDEX;
        std::vector<GEO::index_t> VERTEX_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<GEO::index_t> EDGE_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<int>          EDGE_CONTROL_POINTS_NEXT_IDX_STEP_{};
        std::vector<GEO::index_t> EDGE_INTERNAL_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<int>          EDGE_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP_{};
        std::vector<GEO::index_t> FACET_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<int>          FACET_CONTROL_POINTS_NEXT_IDX_STEP0_{};
        std::vector<int>          FACET_CONTROL_POINTS_NEXT_IDX_STEP1_{};
        std::vector<GEO::index_t> FACET_INTERNAL_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<int>          FACET_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP0_{};
        std::vector<int>          FACET_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP1_{};
    };
}

#endif //HOSM_SURF_CONTROL_GRID_H
