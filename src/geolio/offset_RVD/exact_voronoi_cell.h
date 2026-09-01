//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_EXACT_VORONOI_CELL_H
#define GEOLIO_EXACT_VORONOI_CELL_H
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
typedef CGAL::Exact_predicates_exact_constructions_kernel  Kernel;
typedef CGAL::Polyhedron_3<Kernel>                         Polyhedron;

namespace geolio
{
    class ExactVoronoiCell {
    public:
        explicit ExactVoronoiCell(Polyhedron  polyhedron);

        auto& polyhedron() { return polyhedron_; }

        const auto& polyhedron() const { return polyhedron_; }

    private:
        Polyhedron polyhedron_;
    };
}

#endif //GEOLIO_EXACT_VORONOI_CELL_H
