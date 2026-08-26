//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio/io/io.h>

#include "utils.h"

namespace geolio::test
{
    TEST(AEDTPLT_IOTest, load) {
        GEO::Mesh mesh;
        mesh.load("/Users/canjia/Downloads/HFSS_results/t1_v161_phi_mesh.aedtplt");
        mesh.save(get_current_test_name()+".geogram");
    }
}
