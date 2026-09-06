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

        void save_results() {
            GEO::Attribute<GEO::index_t> quad_f_block(quad_mesh.facets.attributes(), "block");
            MC->label_blocks(quad_f_block);

            quad_mesh.save(get_current_test_name()+"_block.geogram");
        }

        GEO::Mesh quad_mesh;
        std::unique_ptr<QuadMotorCycleGraph> MC;
    };

    TEST_F(QuadMotorCycleGraphTest, base_complex) {
        MC->compute(QuadMotorCycleGraph::BASE_COMPLEX);
        save_results();
    }

    TEST_F(QuadMotorCycleGraphTest, motorcycle_complex) {
        MC->compute(QuadMotorCycleGraph::MOTORCYCLE_COMPLEX);
        save_results();
    }
}