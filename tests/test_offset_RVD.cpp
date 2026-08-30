//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <geolio/offset_RVD/offset_restricted_voronoi_diagram.h>
#include <gtest/gtest.h>
#include <geolio/offset_RVD/mesh_distance_field.h>

namespace geolio::test
{
    TEST(OffsetRVDTest, test) {
        GEO::Mesh mesh;
        EXPECT_TRUE(mesh.load(std::string(TEST_DATA_PATH) + "fandisk.obj"));

        std::vector<double> sites;
        constexpr double d = 1;
        {
            sites.reserve(3*mesh.facets.nb());
            for (const auto& f : mesh.facets) {
                const auto& center = (mesh.facets.point(f, 0)+mesh.facets.point(f, 1)+mesh.facets.point(f, 2))/3;
                const auto& normal = GEO::Geom::triangle_normal(mesh.facets.point(f, 0), mesh.facets.point(f, 1), mesh.facets.point(f, 2));
                const auto& p = center + d*normal;
                sites.push_back(p.x);
                sites.push_back(p.y);
                sites.push_back(p.z);
            }
        }

        const auto distance_field = std::make_shared<MeshDistanceField>(mesh);

        OffsetRestrictedVoronoiDiagram RVD(distance_field);
        RVD.set_sites(sites.data(), sites.size()/3);
        RVD.compute(d);
    }
}
