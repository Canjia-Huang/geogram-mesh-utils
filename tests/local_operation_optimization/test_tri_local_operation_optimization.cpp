//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <string>
#include <geogram/mesh/mesh_io.h>
#include <gtest/gtest.h>
#include <geolio/local_operation_optimization/tri_local_operation_optimization.h>
#include "../utils.h"
#include "geogram/delaunay/CDT_2d.h"

namespace geolio::test
{
    TEST(TriLocalOperationOptimizationTest, cdt_2d) {
        GEO::Mesh mesh(2);
        generate_random_CDT2d_mesh(mesh, 20, 10);
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        {
            TriLocalOperationOptimization<2> TLOO(mesh);
            TLOO.fix_boundary_elements();
            TLOO.optimize(5, 1);
        }

        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");
    }

    TEST(TriLocalOperationOptimizationTest, cdt_2d_not_allow_split_fix_edges) {
        GEO::Mesh mesh(2);
        generate_random_CDT2d_mesh(mesh, 20, 10);
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        {
            TriLocalOperationOptimization<2> TLOO(mesh);
            TLOO.fix_boundary_elements();
            TLOO.allow_split_fixed_edges = false;
            TLOO.optimize(5, 1);
        }

        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");
    }

    TEST(TriLocalOperationOptimizationTest, bunny) {
        GEO::Mesh mesh;
        GEO::mesh_load(std::string(TEST_DATA_PATH)+"bunny.obj", mesh);

        {
            TriLocalOperationOptimization<3> TLOO(mesh);
            GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");
            TLOO.optimize(3, 1.5);
        }

        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");
    }

    TEST(TriLocalOperationOptimizationTest, fandisk_sharp) {
        GEO::Mesh mesh;
        GEO::mesh_load(std::string(TEST_DATA_PATH)+"fandisk.obj", mesh);

        {
            TriLocalOperationOptimization<3> TLOO(mesh);
            TLOO.fix_sharp_elements();
            GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");
            TLOO.optimize();
        }

        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");
    }

    TEST(TriLocalOperationOptimizationTest, beetle_boundary) {
        GEO::Mesh mesh;
        GEO::mesh_load(std::string(TEST_DATA_PATH)+"beetle.obj", mesh);
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        {
            TriLocalOperationOptimization<3> TLOO(mesh);
            TLOO.fix_boundary_elements();
            TLOO.optimize(5, 0.01);
        }

        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");
    }
}