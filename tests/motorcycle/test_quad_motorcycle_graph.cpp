//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/6.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <unordered_set>
#include <gtest/gtest.h>
#include <geolio/motorcycle/quad_motorcycle_graph.h>
#include "../utils.h"
#include <geolio/common/pair_hash.h>

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

            std::unordered_set<std::pair<GEO::index_t, GEO::index_t>, PairHash> edge_to_create;
            for (const auto& f : quad_mesh.facets) {
                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    if (const auto& nf = quad_mesh.facets.adjacent(f, lv);
                        nf == GEO::NO_FACET || quad_f_block[f] != quad_f_block[nf]
                        ) {
                        const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                            quad_mesh.facets.vertex(f, lv),
                            quad_mesh.facets.vertex(f, (lv+1)%4));
                        edge_to_create.insert(edge);
                    }
                }
            }
            GEO::index_t new_e = quad_mesh.edges.create_edges(edge_to_create.size());
            for (const auto& [ev0, ev1] : edge_to_create) {
                quad_mesh.edges.set_vertex(new_e, 0, ev0);
                quad_mesh.edges.set_vertex(new_e, 1, ev1);
                ++new_e;
            }

            GEO::Mesh coarse_mesh;
            std::vector<GEO::index_t> old_fc_to_new_fc;
            MC->create_coarse_mesh(coarse_mesh, &old_fc_to_new_fc);
            ASSERT_EQ(old_fc_to_new_fc.size(), quad_mesh.facet_corners.nb());

            GEO::Attribute<GEO::index_t> quad_mesh_fc_to_new_fc(quad_mesh.facet_corners.attributes(), "new_fc");
            for (const auto& fc : quad_mesh.facet_corners)
                quad_mesh_fc_to_new_fc[fc] = old_fc_to_new_fc[fc];

            quad_mesh.save(get_current_test_name()+"_block.geogram");
            coarse_mesh.save(get_current_test_name()+"_coarse.geogram");
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