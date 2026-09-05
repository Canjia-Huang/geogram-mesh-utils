//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio/motorcycle/hex_motorcycle_complex.h>
#include "../utils.h"
#include "geolio/mesh/hex_operations.h"

namespace geolio::test
{
    class HexMotorcycleComplexTest : public ::testing::Test {
    protected:
        void SetUp() override {
            ASSERT_TRUE(hex_mesh.load(std::string(TEST_DATA_PATH) + "i01c_m1_hex.ovm"));
            hex_mesh.edges.clear();
            hex_mesh.facets.clear();
            MC = std::make_unique<HexMotorCycleComplex>(hex_mesh);
        }

        void save_results() {
            GEO::Attribute<GEO::index_t> hex_c_block(hex_mesh.cells.attributes(), "block");
            MC->label_blocks(hex_c_block);

            std::vector<std::pair<GEO::index_t, GEO::index_t>> cf_to_create;
            std::vector<bool> processed_cf(8*hex_mesh.cells.nb(), false);
            for (const auto& c : hex_mesh.cells) {
                for (GEO::index_t lf = 0; lf < 6; ++lf) {
                    if (processed_cf[8*c+lf])
                        continue;

                    if (const auto nc = hex_mesh.cells.adjacent(c, lf);
                        nc == GEO::NO_CELL || hex_c_block[c] != hex_c_block[nc]
                        ) {
                        cf_to_create.emplace_back(c, lf);

                        processed_cf[8*c+lf] = true;
                        if (nc != GEO::NO_CELL) {
                            const auto nlf = find_hex_facet(
                                hex_mesh,
                                nc,
                                hex_mesh.cells.facet_vertex(c, lf, 2),
                                hex_mesh.cells.facet_vertex(c, lf, 1),
                                hex_mesh.cells.facet_vertex(c, lf, 0));
                            assert(nlf != GEO::NO_INDEX);
                            processed_cf[8*nc+nlf] = true;
                        }
                    }
                }
            }
            GEO::index_t new_f = hex_mesh.facets.create_quads(cf_to_create.size());
            for (const auto& [c, lf] : cf_to_create) {
                for (GEO::index_t lv = 0; lv < 4; ++lv)
                    hex_mesh.facets.set_vertex(new_f, lv, hex_mesh.cells.facet_vertex(c, lf, lv));
                ++new_f;
            }

            GEO::Mesh coarse_mesh;
            std::vector<GEO::index_t> old_cf_to_new_cf;
            MC->create_coarse_mesh(coarse_mesh, &old_cf_to_new_cf);
            ASSERT_EQ(old_cf_to_new_cf.size(), 8*hex_mesh.cells.nb());

            GEO::Attribute<GEO::index_t> hex_mesh_cf_to_new_cf(hex_mesh.cell_facets.attributes(), "new_cf");
            for (const auto& cf : hex_mesh.cell_facets)
                hex_mesh_cf_to_new_cf[cf] = old_cf_to_new_cf[cf];

            hex_mesh.save(get_current_test_name()+"_block.geogram");
            coarse_mesh.save(get_current_test_name()+"_coarse.geogram");
        }

        GEO::Mesh hex_mesh;
        std::unique_ptr<HexMotorCycleComplex> MC;
    };

    TEST_F(HexMotorcycleComplexTest, base_complex) {
        MC->compute(HexMotorCycleComplex::BASE_COMPLEX);
        save_results();
    }

    TEST_F(HexMotorcycleComplexTest, motorcycle_complex) {
        MC->compute(HexMotorCycleComplex::MOTORCYCLE_COMPLEX);
        save_results();
    }
}
