//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <geogram/mesh/mesh_io.h>
#include <gtest/gtest.h>
#include <mach/task_info.h>

#include "geolio/tri_local_operation_optimization.h"

namespace geolio::test
{
    TEST(TriLocalOperationOptimizationTest, test) {
        GEO::Mesh mesh;
        GEO::mesh_load(std::string(TEST_DATA_PATH)+"bunny.obj", mesh);

        TriLocalOperationOptimization TLO_opt(mesh);
        TLO_opt.optimize();
        GEO::mesh_save(mesh, "debug.geogram");
    }
}