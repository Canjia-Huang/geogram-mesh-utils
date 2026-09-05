//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio/motorcycle/hex_motorcycle_complex.h>
#include "../utils.h"

namespace geolio::test
{
    TEST(HexMotorcycleComplexTest, base_complex) {
        GEO::Mesh hex_mesh;
        ASSERT_TRUE(hex_mesh.load(std::string(TEST_DATA_PATH) + "i01c_m1_hex.ovm"));

        HexMotorCycleComplex MC(hex_mesh);
        MC.compute(HexMotorCycleComplex::BASE_COMPLEX);

        GEO::Mesh coarse_mesh;
        MC.create_coarse_mesh(coarse_mesh);
        coarse_mesh.save(get_current_test_name()+".geogram");
    }
}
