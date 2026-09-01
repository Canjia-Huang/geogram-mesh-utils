//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/25.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <gtest/gtest.h>
#include <geolio//mesh/tri_metrics.h>

namespace geolio::test
{
    TEST(TriMetricsTest, get_triangle_minimum_angle_45) {
        GEO::Mesh M;
        M.vertices.create_vertices(3);
        M.vertices.point(0) = GEO::vec3(0, 0, 0);
        M.vertices.point(1) = GEO::vec3(1, 0, 0);
        M.vertices.point(2) = GEO::vec3(0, 1, 0);
        M.facets.create_triangle(0, 1, 2);

        EXPECT_NEAR(get_triangle_minimum_angle(M, 0), M_PI/4, 1e-10);
    }

    TEST(TriMetricsTest, get_triangle_minimum_angle_30) {
        GEO::Mesh M;
        M.vertices.create_vertices(3);
        M.vertices.point(0) = GEO::vec3(0, 0, std::sqrt(3));
        M.vertices.point(1) = GEO::vec3(0, 0, 0);
        M.vertices.point(2) = GEO::vec3(0, 1, 0);
        M.facets.create_triangle(0, 1, 2);

        EXPECT_NEAR(get_triangle_minimum_angle(M, 0), M_PI/6, 1e-10);
    }
}