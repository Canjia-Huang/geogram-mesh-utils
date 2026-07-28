//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/12.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_UTILS_H
#define GEOLIO_UTILS_H

#include <gtest/gtest.h>
#include <string>
#include <geogram/delaunay/CDT_2d.h>
#include "geolio/CDT_2d.h"
#include "geolio/periodic_delaunay_3d.h"
#include <geogram/delaunay/periodic_delaunay_3d.h>

namespace geolio::test
{
    inline void generate_random_CDT2d_mesh(
        GEO::Mesh& mesh,
        const GEO::index_t samples_nb = 20,
        const double w = 10
        ) {
        GEO::CDT2d CDT;
        CDT.create_enclosing_quad(GEO::vec2(-w, -w), GEO::vec2(w, -w), GEO::vec2(w, w), GEO::vec2(-w, w));
        for (GEO::index_t i = 0; i < samples_nb; ++i)
            CDT.insert(GEO::vec2(-w+1.8*w*GEO::Numeric::random_float32(), -w+1.8*w*GEO::Numeric::random_float32()));

        append_CDT2d_to_mesh(CDT, mesh);
        mesh.facets.connect();
    }

    inline void generate_random_delaunay3d_mesh (
        GEO::Mesh& mesh,
        const GEO::index_t samples_nb = 20,
        const double w = 10
        ) {
        std::vector<double> points;
        points.reserve(samples_nb*3);
        for (GEO::index_t i = 0, i_end = samples_nb*3; i < i_end; ++i)
            points.push_back(-w*1.8*w*GEO::Numeric::random_float32());

        GEO::SmartPointer<GEO::PeriodicDelaunay3d> delaunay = new GEO::PeriodicDelaunay3d(false);
        delaunay->set_vertices(samples_nb, points.data());
        delaunay->compute();

        append_delaunay_to_mesh(*delaunay, mesh);
        mesh.cells.connect();
    }

    inline std::string get_current_test_name(){
        const testing::TestInfo* const current_test_info = testing::UnitTest::GetInstance()->current_test_info();
        return std::string("test")
                + "_"
                + std::string(current_test_info->test_case_name())
                + "_"
                + std::string(current_test_info->name());
    }
}


#endif //GEOLIO_UTILS_H