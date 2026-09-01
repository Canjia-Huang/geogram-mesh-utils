//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "exact_voronoi_cell.h"
#include <utility>

namespace geolio
{
    ExactVoronoiCell::ExactVoronoiCell(
        Polyhedron  polyhedron
        ) : polyhedron_(std::move(polyhedron))
    {}
}

