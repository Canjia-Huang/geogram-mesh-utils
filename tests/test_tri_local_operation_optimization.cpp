//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <geogram/mesh/mesh_io.h>
#include <gtest/gtest.h>
#include <geolio/local_operation_optimization/tri_local_operation_optimization.h>
#include "utils.h"
#include "geogram/delaunay/CDT_2d.h"

namespace geolio::test
{
    TEST(TriLocalOperationOptimizationTest, two_d_model) {
        GEO::Mesh mesh(2);
        generate_random_CDT2d_mesh(mesh, 20, 10);
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        {
            TriLocalOperationOptimization TLOO(mesh);
            TLOO.fix_boundary_elements();
            TLOO.optimize(10, 1);
        }
        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");
    }

    TEST(TriLocalOperationOptimizationTest, three_d_model) {
        GEO::Mesh mesh;
        GEO::mesh_load(std::string(TEST_DATA_PATH)+"bunny.obj", mesh);

        {
            TriLocalOperationOptimization TLOO(mesh);
            TLOO.fix_sharp_elements();
            GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

            TLOO.optimize();
            GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");
        }
    }

    TEST(TriLocalOperationOptimizationTest, three_d_model_sharp) {
        GEO::Mesh mesh;
        GEO::mesh_load(std::string(TEST_DATA_PATH)+"fandisk.obj", mesh);

        {
            TriLocalOperationOptimization TLOO(mesh);
            TLOO.fix_sharp_elements();
            GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

            TLOO.optimize();
            GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");
        }

    }

    TEST(TriLocalOperationOptimizationTest, three_d_model_boundary) {
        GEO::Mesh mesh;
        GEO::mesh_load(std::string(TEST_DATA_PATH)+"beetle.obj", mesh);
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        {
            TriLocalOperationOptimization TLOO(mesh);
            TLOO.fix_boundary_elements();
            TLOO.optimize();
            GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");
        }

    }
}