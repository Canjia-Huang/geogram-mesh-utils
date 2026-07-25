//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/25.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <ranges>
#include <unordered_set>
#include <geogram/mesh/mesh_io.h>
#include <gtest/gtest.h>
#include "geolio/mesh_operations.h"

namespace geolio::test
{
    TEST(MeshOperationsTest, get_vertex_incident_facets) {
        GEO::Mesh M;
        ASSERT_TRUE(GEO::mesh_load(std::string(TEST_DATA_PATH) + "polygonal_disk.geogram", M));

        std::vector<std::unordered_set<GEO::index_t>> M_v_adjacent_f(M.vertices.nb());
        std::vector<bool> M_v_border(M.vertices.nb(), false);
        for (const auto& f : M.facets) {
            for (GEO::index_t lv = 0, lv_end = M.facets.nb_vertices(f); lv < lv_end; ++lv) {
                const bool FIRST_INSERT = M_v_adjacent_f[M.facets.vertex(f, lv)].insert(f).second;
                ASSERT_TRUE(FIRST_INSERT);

                if (M.facets.adjacent(f, lv) == GEO::NO_FACET) {
                    M_v_border[M.facets.vertex(f, lv)] = true;
                    M_v_border[M.facets.vertex(f, (lv+1)%lv_end)] = true;
                }
            }
        }

        for (const auto& f : M.facets) {
            for (GEO::index_t lv = 0, lv_end = M.facets.nb_vertices(f); lv < lv_end; ++lv) {
                const auto& v = M.facets.vertex(f, lv);

                std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
                const bool ON_BORDER = get_vertex_incident_facets(M, f, lv, ordered_f_and_lv);

                EXPECT_EQ(ON_BORDER, M_v_border[v]);

                /* Verify integrity */
                const auto& adjacent_facets = M_v_adjacent_f[v];
                EXPECT_EQ(ordered_f_and_lv.size(), adjacent_facets.size());
                for (const auto& nf: ordered_f_and_lv | std::views::keys)
                    EXPECT_TRUE(adjacent_facets.contains(nf));

                /* Check loop */
                for (GEO::index_t i = 0, i_end = ordered_f_and_lv.size(); i < i_end; ++i) {
                    const auto& [f0, lv0] = ordered_f_and_lv[i];
                    if (const auto& nf = M.facets.adjacent(f0, lv0);
                        nf == GEO::NO_FACET)
                        EXPECT_EQ(i, i_end-1);
                    else
                        EXPECT_EQ(nf, ordered_f_and_lv[(i+1)%i_end].first);
                }
            }
        }
    }

    TEST(MeshOperationsTest, get_edge_incident_cells) {
        // TODO
    }
}