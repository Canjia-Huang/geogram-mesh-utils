//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/6.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio/motorcycle/quad_motorcycle_graph.h>
#include "../utils.h"

namespace geolio::test
{
    class QuadMotorCycleGraphTest : public ::testing::Test {
    protected:
        void SetUp() override {
            ASSERT_TRUE(quad_mesh.load(std::string(TEST_DATA_PATH) + "botijo_out_quad_7.geogram"));
            MC = std::make_unique<QuadMotorCycleGraph>(quad_mesh);
        }

        GEO::Mesh quad_mesh;
        std::unique_ptr<QuadMotorCycleGraph> MC;
    };

    TEST_F(QuadMotorCycleGraphTest, base_complex) {
        MC->compute(QuadMotorCycleGraph::BASE_COMPLEX);
    }
}