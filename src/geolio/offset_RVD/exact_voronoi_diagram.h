//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_EXACT_VORONOI_DIAGRAM_H
#define GEOLIO_EXACT_VORONOI_DIAGRAM_H
#include <geogram/basic/numeric.h>
#include "exact_voronoi_cell.h"
#include <geogram/mesh/mesh.h>

namespace geolio
{
    class ExactVoronoiDiagram {
    public:
        ExactVoronoiDiagram() = default;

        void create_voronoi_cells(
            const double* sites, GEO::index_t sites_nb,
            double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);

        void append_to_mesh(GEO::Mesh& mesh);

    private:
        std::vector<ExactVoronoiCell> voronoi_cells_;
    };
}

#endif //GEOLIO_EXACT_VORONOI_DIAGRAM_H
