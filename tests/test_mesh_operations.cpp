//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/25.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <ranges>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <geogram/mesh/mesh_io.h>
#include <geolio/common/pair_hash.h>
#include <geolio/mesh/mesh_operations.h>
#include <geolio/mesh/tet_descriptor.h>
#include <gtest/gtest.h>
#include "utils.h"

namespace geolio::test
{
    class SurfaceMeshOperationsTest : public ::testing::Test {
    protected:
        void SetUp() override {
            ASSERT_TRUE(GEO::mesh_load(std::string(TEST_DATA_PATH) + "polygonal_disk.geogram", M));

            M_v_adjacent_f.resize(M.vertices.nb());
            M_v_border.assign(M.vertices.nb(), false);
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
        }

        GEO::Mesh M;
        std::vector<std::unordered_set<GEO::index_t>> M_v_adjacent_f;
        std::vector<bool> M_v_border;
    };

    TEST_F(SurfaceMeshOperationsTest, get_vertex_incident_facets) {
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

    /* ============================================================================================================= */

    class TetMeshOperationsTest : public ::testing::Test {
    protected:
        void SetUp() override {
            generate_random_delaunay3d_mesh(mesh, 20, 10);

            mesh_v_adjacent_c.resize(mesh.vertices.nb());
            mesh_v_border.assign(mesh.vertices.nb(), false);
            for (const auto& c : mesh.cells) {
                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    const auto& v = mesh.cells.vertex(c, lv);
                    mesh_v_adjacent_c[v].insert(c);
                }
                for (GEO::index_t lf = 0; lf < 4; ++lf) {
                    if (mesh.cells.adjacent(c, lf) == GEO::NO_CELL) {
                        for (GEO::index_t lv = 0; lv < 3; ++lv) {
                            const auto& v = mesh.cells.facet_vertex(c, lf, lv);
                            mesh_v_border[v] = true;
                        }
                    }
                }
            }

            for (const auto& c : mesh.cells) {
                for (GEO::index_t le = 0; le < 6; ++le) {
                    const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                        mesh.cells.edge_vertex(c, le, 0), mesh.cells.edge_vertex(c, le, 1));
                    if (auto it = mesh_e_adjacent_c.find(edge);
                        it != mesh_e_adjacent_c.end())
                        it->second.insert(c);
                    else {
                        std::unordered_set<GEO::index_t> adj_c;
                        adj_c.insert(c);
                        mesh_e_adjacent_c.emplace(edge, adj_c);
                    }
                }

                for (GEO::index_t lf = 0; lf < 4; ++lf) {
                    if (mesh.cells.adjacent(c, lf) == GEO::NO_CELL) {
                        for (const auto& le : TET_LF_INCIDENT_LE[lf]) {
                            const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                                mesh.cells.edge_vertex(c, le, 0), mesh.cells.edge_vertex(c, le, 1));
                            mesh_e_border.insert(edge);
                        }
                    }
                }
            }
        }

        GEO::Mesh mesh;
        std::vector<std::unordered_set<GEO::index_t>> mesh_v_adjacent_c;
        std::vector<bool> mesh_v_border;
        std::unordered_map<std::pair<GEO::index_t, GEO::index_t>, std::unordered_set<GEO::index_t>, PairHash> mesh_e_adjacent_c;
        std::unordered_set<std::pair<GEO::index_t, GEO::index_t>, PairHash> mesh_e_border;
    };

    TEST_F(TetMeshOperationsTest, get_vertex_incident_cells) {
        for (const auto& c : mesh.cells) {
            for (GEO::index_t lv = 0; lv < 4; ++lv) {
                const auto& v = mesh.cells.vertex(c, lv);

                std::vector<std::pair<GEO::index_t, GEO::index_t>> c_and_lv;
                const bool ON_BORDER = get_vertex_incident_cells(mesh, c, lv, c_and_lv);

                EXPECT_EQ(ON_BORDER, mesh_v_border[v]);

                /* Verify integrity */
                const auto& adjacent_cells = mesh_v_adjacent_c[v];
                EXPECT_EQ(c_and_lv.size(), adjacent_cells.size());
                for (const auto& nc: c_and_lv | std::views::keys)
                    EXPECT_TRUE(adjacent_cells.contains(nc));

                /* Check lv */
                for (const auto& [nc, nlv] : c_and_lv)
                    EXPECT_EQ(mesh.cells.vertex(nc, nlv), v);
            }
        }
    }

    TEST_F(TetMeshOperationsTest, get_edge_incident_cells) {
        for (const auto& c : mesh.cells) {
            for (GEO::index_t le = 0; le < 6; ++le) {
                const auto& ev0 = mesh.cells.edge_vertex(c, le, 0);
                const auto& ev1 = mesh.cells.edge_vertex(c, le, 1);
                const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(ev0, ev1);

                std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> ordered_c_le_lf;
                const bool ON_BORDER = get_edge_incident_cells(mesh, c, le, ordered_c_le_lf);

                EXPECT_EQ(ON_BORDER, mesh_e_border.contains(edge));

                /* Verify integrity */
                ASSERT_TRUE(mesh_e_adjacent_c.contains(edge));
                const auto& adjacent_cells = mesh_e_adjacent_c.at(edge);
                EXPECT_EQ(ordered_c_le_lf.size(), adjacent_cells.size());
                for (const auto& nc: ordered_c_le_lf | std::views::keys)
                    EXPECT_TRUE(adjacent_cells.contains(nc));

                /* Check le */
                for (const auto& [nc, nle, _] : ordered_c_le_lf) {
                    const auto& nev0 = mesh.cells.edge_vertex(nc, nle, 0);
                    const auto& nev1 = mesh.cells.edge_vertex(nc, nle, 1);
                    const std::pair<GEO::index_t, GEO::index_t> nedge = std::minmax(nev0, nev1);
                    EXPECT_EQ(edge.first, nedge.first);
                    EXPECT_EQ(edge.second, nedge.second);
                }

                /* Check loop */
                for (GEO::index_t i = 0, i_end = ordered_c_le_lf.size(); i < i_end; ++i) {
                    const auto& [nc, _, nlf] = ordered_c_le_lf[i];
                    if (const auto& nnc = mesh.cells.adjacent(nc, nlf);
                        nnc == GEO::NO_CELL)
                        EXPECT_EQ(i, i_end-1);
                    else
                        EXPECT_EQ(nnc, get<0>(ordered_c_le_lf[(i+1)%i_end]));
                }
            }
        }
    }

    TEST_F(TetMeshOperationsTest, get_edge_incident_cells_2) {
        for (const auto& c : mesh.cells) {
            for (GEO::index_t lf = 0; lf < 4; ++lf) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    const auto& ev0 = mesh.cells.facet_vertex(c, lf, lv);
                    const auto& ev1 = mesh.cells.facet_vertex(c, lf, (lv+1)%3);
                    const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(ev0, ev1);

                    std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> ordered_c_le_lf;
                    const bool ON_BORDER = get_edge_incident_cells(mesh, c, lf, lv, ordered_c_le_lf);

                    EXPECT_EQ(ON_BORDER, mesh_e_border.contains(edge));

                    /* Verify integrity */
                    ASSERT_TRUE(mesh_e_adjacent_c.contains(edge));
                    const auto& adjacent_cells = mesh_e_adjacent_c.at(edge);
                    EXPECT_EQ(ordered_c_le_lf.size(), adjacent_cells.size());
                    for (const auto& nc: ordered_c_le_lf | std::views::keys)
                        EXPECT_TRUE(adjacent_cells.contains(nc));

                    /* Check le */
                    for (const auto& [nc, nle, _] : ordered_c_le_lf) {
                        const auto& nev0 = mesh.cells.edge_vertex(nc, nle, 0);
                        const auto& nev1 = mesh.cells.edge_vertex(nc, nle, 1);
                        const std::pair<GEO::index_t, GEO::index_t> nedge = std::minmax(nev0, nev1);
                        EXPECT_EQ(edge.first, nedge.first);
                        EXPECT_EQ(edge.second, nedge.second);
                    }

                    /* Check loop */
                    for (GEO::index_t i = 0, i_end = ordered_c_le_lf.size(); i < i_end; ++i) {
                        const auto& [nc, _, nlf] = ordered_c_le_lf[i];
                        if (const auto& nnc = mesh.cells.adjacent(nc, nlf);
                            nnc == GEO::NO_CELL)
                            EXPECT_EQ(i, i_end-1);
                        else
                            EXPECT_EQ(nnc, get<0>(ordered_c_le_lf[(i+1)%i_end]));
                    }
                }
            }
        }
    }
}