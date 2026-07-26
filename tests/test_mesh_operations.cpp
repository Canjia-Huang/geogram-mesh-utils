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


//     class GetVertexIncidentTetrahedraTest : public TetrahedronOperationsTest {
//     public:
//         bool compute(
//             const GEO::index_t _c,
//             const GEO::index_t _lv
//             ) {
//             // return get_vertex_incident_tetrahedra(M, _c, _lv, c_and_lv);
//         }
//
//         void check_incident(
//             const GEO::index_t v
//             ) {
//             for (const auto& [c, lv] : c_and_lv)
//                 EXPECT_EQ(M.cells.vertex(c, lv), v);
//         }
//
//         void check_complete(
//             const GEO::index_t v
//             ) const {
//             std::unordered_map<std::pair<GEO::index_t, GEO::index_t>, bool, PairHash> incident_cells; // (c, lv) -> found
//             for (const auto& c : M.cells) {
//                 for (GEO::index_t lv = 0; lv < 4; ++lv) {
//                     if (M.cells.vertex(c, lv) == v) {
//                         incident_cells.emplace(std::pair(c, lv), false);
//                         break;
//                     }
//                 }
//             }
//
//             for (const auto& c_lv : c_and_lv) {
//                 auto it = incident_cells.find(c_lv);
//                 ASSERT_FALSE(it == incident_cells.end());
//                 EXPECT_FALSE(it->second);
//                 it->second = true;
//             }
//
//             for (const auto &found: incident_cells | std::views::values)
//                 EXPECT_TRUE(found);
//         }
//
//         std::vector<std::pair<GEO::index_t, GEO::index_t>> c_and_lv;
//     };
//
//     class GetInteriorVertexIncidentTetrahedraTest : public GetVertexIncidentTetrahedraTest {};
//
//     TEST_P(GetInteriorVertexIncidentTetrahedraTest, each_vertex) {
//         auto [c, lv, _] = GetParam();
//
//         EXPECT_FALSE(compute(c, lv));
//         check_incident(M.cells.vertex(c, lv));
//         check_complete(M.cells.vertex(c, lv));
//     }
//
//     INSTANTIATE_TEST_SUITE_P(
//         TetrahedronOperationsTest,
//         GetInteriorVertexIncidentTetrahedraTest,
//         ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_VERTEX_C_LV)));
//
//     class GetBorderVertexIncidentTetrahedraTest : public GetVertexIncidentTetrahedraTest {};
//
//     TEST_P(GetBorderVertexIncidentTetrahedraTest, each_vertex) {
//         auto [c, lv, _] = GetParam();
//
//         EXPECT_TRUE(compute(c, lv));
//         check_incident(M.cells.vertex(c, lv));
//         check_complete(M.cells.vertex(c, lv));
//     }
//
//     INSTANTIATE_TEST_SUITE_P(
//         TetrahedronOperationsTest,
//         GetBorderVertexIncidentTetrahedraTest,
//         ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_VERTEX_C_LV)));
// }
//
// /* == GetEdgeIncidentTetrahedraTest ================================================================================ */
//
// namespace geolio::test
// {
//     class GetEdgeIncidentTetrahedraTest : public TetrahedronOperationsTest {
//     public:
//         bool compute(
//             const GEO::index_t _c,
//             const GEO::index_t _lf,
//             const GEO::index_t _lv
//             ) {
//             // return get_edge_incident_tetrahedra(M, _c, _lf, _lv, ordered_c_and_lf);
//         }
//
//         bool compute(
//             const GEO::index_t _c,
//             const GEO::index_t _le
//             ) {
//             // return get_edge_incident_tetrahedra(M, _c, _le, ordered_c_and_lf);
//         }
//
//         void check_incident(
//             const GEO::index_t ev0,
//             const GEO::index_t ev1
//             ) {
//             for (const auto& [c, lf] : ordered_c_and_lf) {
//                 bool incident = false;
//                 for (GEO::index_t lv = 0; lv < 3; ++lv) {
//                     const GEO::index_t cev0 = M.cells.facet_vertex(c, lf, lv);
//                     const GEO::index_t cev1 = M.cells.facet_vertex(c, lf, (lv+1)%3);
//                     if ((cev0 == ev0 && cev1 == ev1) ||
//                         (cev0 == ev1 && cev1 == ev0)
//                         ) {
//                         incident = true;
//                         break;
//                         }
//                 }
//                 EXPECT_TRUE(incident);
//             }
//         }
//
//         void check_complete(
//             const GEO::index_t ev0,
//             const GEO::index_t ev1
//             ) {
//             std::unordered_map<GEO::index_t, bool> incident_cells; // (cell, found)
//             for (const auto& c : M.cells) {
//                 for (GEO::index_t le = 0; le < 6; ++le) {
//                     const GEO::index_t cev0 = M.cells.edge_vertex(c, le, 0);
//                     const GEO::index_t cev1 = M.cells.edge_vertex(c, le, 1);
//                     if ((cev0 == ev0 && cev1 == ev1) || (cev0 == ev1 && cev1 == ev0)) {
//                         incident_cells.emplace(c, false);
//                         break;
//                     }
//                 }
//             }
//
//             for (const auto &c: ordered_c_and_lf | std::views::keys) {
//                 auto it = incident_cells.find(c);
//                 EXPECT_FALSE(it == incident_cells.end());
//                 it->second = true;
//             }
//
//             for (const auto &found: incident_cells | std::views::values)
//                 EXPECT_TRUE(found);
//         }
//
//         virtual void check_loop() = 0;
//
//         std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_c_and_lf;
//     };
//
//     class GetInteriorEdgeIncidentTetrahedraTest : public GetEdgeIncidentTetrahedraTest {
//     public:
//         void check_loop() override {
//             for (GEO::index_t i = 0, i_end = ordered_c_and_lf.size(); i < i_end; ++i) {
//                 const auto& [c, lf] = ordered_c_and_lf[i];
//                 EXPECT_EQ(M.cells.adjacent(c, lf), ordered_c_and_lf[(i+1)%i_end].first);
//             }
//         }
//     };
//
//     class GetInteriorEdgeIncidentTetrahedraTest_c_lf_lv : public GetInteriorEdgeIncidentTetrahedraTest {};
//
//     TEST_P(GetInteriorEdgeIncidentTetrahedraTest_c_lf_lv, each_edge) {
//         auto [c, lf, lv] = GetParam();
//
//         EXPECT_FALSE(compute(c, lf, lv));
//         check_incident(M.cells.facet_vertex(c, lf, lv), M.cells.facet_vertex(c, lf, (lv+1)%3));
//         check_complete(M.cells.facet_vertex(c, lf, lv), M.cells.facet_vertex(c, lf, (lv+1)%3));
//         check_loop();
//     }
//
//     INSTANTIATE_TEST_SUITE_P(
//         TetrahedronOperationsTest,
//         GetInteriorEdgeIncidentTetrahedraTest_c_lf_lv,
//         ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_EDGE_C_LF_LV)));
//
//     class GetInteriorEdgeIncidentTetrahedraTest_c_le : public GetInteriorEdgeIncidentTetrahedraTest {};
//
//     TEST_P(GetInteriorEdgeIncidentTetrahedraTest_c_le, each_edge) {
//         auto [c, le, _] = GetParam();
//
//         EXPECT_FALSE(compute(c, le));
//         check_incident(M.cells.edge_vertex(c, le, 0), M.cells.edge_vertex(c, le, 1));
//         check_complete(M.cells.edge_vertex(c, le, 0), M.cells.edge_vertex(c, le, 1));
//         check_loop();
//     }
//
//     INSTANTIATE_TEST_SUITE_P(
//         TetrahedronOperationsTest,
//         GetInteriorEdgeIncidentTetrahedraTest_c_le,
//         ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_EDGE_C_LE)));
//
//     class GetBorderEdgeIncidentTetrahedraTest : public GetEdgeIncidentTetrahedraTest {
//     public:
//         void check_loop() override {
//             for (GEO::index_t i = 0, i_end = ordered_c_and_lf.size(); i < i_end; ++i) {
//                 const auto& [c, lf] = ordered_c_and_lf[i];
//                 if (i == i_end-1)
//                     EXPECT_EQ(M.cells.adjacent(c, lf), GEO::NO_CELL);
//                 else
//                     EXPECT_EQ(M.cells.adjacent(c, lf), ordered_c_and_lf[i+1].first);
//             }
//         }
//     };
//
//     class GetBorderEdgeIncidentTetrahedraTest_c_lf_lv : public GetBorderEdgeIncidentTetrahedraTest {};
//
//     TEST_P(GetBorderEdgeIncidentTetrahedraTest_c_lf_lv, each_c_lf_lv) {
//         auto [c, lf, lv] = GetParam();
//
//         EXPECT_TRUE(compute(c, lf, lv));
//         check_incident(M.cells.facet_vertex(c, lf, lv), M.cells.facet_vertex(c, lf, (lv+1)%3));
//         check_complete(M.cells.facet_vertex(c, lf, lv), M.cells.facet_vertex(c, lf, (lv+1)%3));
//         check_loop();
//     }
//
//     INSTANTIATE_TEST_SUITE_P(
//         TetrahedronOperationsTest,
//         GetBorderEdgeIncidentTetrahedraTest_c_lf_lv,
//         ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_EDGE_C_LF_LV)));
//
//     class GetBorderEdgeIncidentTetrahedraTest_c_le : public GetBorderEdgeIncidentTetrahedraTest {};
//
//     TEST_P(GetBorderEdgeIncidentTetrahedraTest_c_le, each_c_le) {
//         auto [c, le, _] = GetParam();
//
//         EXPECT_TRUE(compute(c, le));
//         check_incident(M.cells.edge_vertex(c, le, 0), M.cells.edge_vertex(c, le, 1));
//         check_complete(M.cells.edge_vertex(c, le, 0), M.cells.edge_vertex(c, le, 1));
//         check_loop();
//     }
//
//     INSTANTIATE_TEST_SUITE_P(
//         TetrahedronOperationsTest,
//         GetBorderEdgeIncidentTetrahedraTest_c_le,
//         ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_EDGE_C_LE)));
// }