//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <geogram/mesh/mesh_io.h>
#include <gtest/gtest.h>
#include <mach/task_info.h>
#include "geolio/tri_local_operation_optimization.h"
#include "utils.h"
#include "geogram/delaunay/CDT_2d.h"
#include "geolio/CDT_2d.h"

namespace geolio::test
{
    TEST(TriLocalOperationOptimizationTest, two_d_model) {
        GEO::CDT2d CDT;
        constexpr double w = 10;
        CDT.create_enclosing_quad(GEO::vec2(-w, -w), GEO::vec2(w, -w), GEO::vec2(w, w), GEO::vec2(-w, w));
        for (GEO::index_t i = 0; i < 20; ++i)
            CDT.insert(GEO::vec2(-w+1.8*w*GEO::Numeric::random_float32(), -w+1.8*w*GEO::Numeric::random_float32()));

        GEO::Mesh mesh(2);
        append_CDT2d_to_mesh(CDT, mesh);
        mesh.facets.connect();
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        TriLocalOperationOptimization TLO_opt(mesh);
        TLO_opt.optimize(1);
        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");
    }

    TEST(TriLocalOperationOptimizationTest, three_d_model) {
        GEO::Mesh mesh;
        GEO::mesh_load(std::string(TEST_DATA_PATH)+"bunny.obj", mesh);

        TriLocalOperationOptimization TLO_opt(mesh);
        TLO_opt.optimize();
        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");
    }
}