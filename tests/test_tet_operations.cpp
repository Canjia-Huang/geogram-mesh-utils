//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <gtest/gtest.h>
#include <numeric>
#include <ranges>
#include <unordered_set>
#include <geogram/mesh/mesh_io.h>
#include "geolio/mesh_operations.h"
#include "geolio/tet_operations.h"

namespace geolio::test
{
    class SingleTetOperationsTest : public ::testing::Test {
    protected:
        void SetUp() override {
            M.vertices.create_vertices(4);
            M.vertices.point(0) = GEO::vec3(0,0,0);
            M.vertices.point(1) = GEO::vec3(1,0,0);
            M.vertices.point(2) = GEO::vec3(0,1,0);
            M.vertices.point(3) = GEO::vec3(0,0,1);
            M.cells.create_tet(0,1,2,3);

            ASSERT_GT(GEO::Geom::tetra_signed_volume(
                M.vertices.point(0),
                M.vertices.point(1),
                M.vertices.point(2),
                M.vertices.point(3)), 0);
        }

    public:
        GEO::Mesh M;
        const GEO::index_t c = 0;
    };

    TEST_F(SingleTetOperationsTest, find_tet_edge_from_local_vertices) {
        for (GEO::index_t le = 0; le < M.cells.nb_edges(c); ++le) {
            const auto& lv0 = TET_LE_INCIDENT_LV[le][0];
            const auto& lv1 = TET_LE_INCIDENT_LV[le][1];
            EXPECT_EQ(find_tet_edge_from_local_vertices(lv0, lv1), le);
            EXPECT_EQ(find_tet_edge_from_local_vertices(lv1, lv0), le);
        }
    }

    TEST_F(SingleTetOperationsTest, find_tet_edge) {
        for (GEO::index_t le = 0; le < M.cells.nb_edges(c); ++le) {
            const auto& ev0 = M.cells.edge_vertex(c, le, 0);
            const auto& ev1 = M.cells.edge_vertex(c, le, 1);
            EXPECT_EQ(find_tet_edge(M, c, ev0, ev1), le);
            EXPECT_EQ(find_tet_edge(M, c, ev1, ev0), le);
        }
    }

    TEST_F(SingleTetOperationsTest, get_tet_facet_another_vertex) {
        for (GEO::index_t lf = 0; lf < M.cells.nb_facets(c); ++lf) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                const auto& v0 = M.cells.facet_vertex(c, lf, lv);
                const auto& v1 = M.cells.facet_vertex(c, lf, (lv+1)%3);
                const auto& v2 = M.cells.facet_vertex(c, lf, (lv+2)%3);
                EXPECT_EQ(get_tet_facet_another_vertex(M, c, lf, v0, v1), v2);
            }
        }
    }

    /* ============================================================================================================= */

    class TetOperationsTest : public ::testing::Test {
    protected:
        void SetUp() override {
            ASSERT_TRUE(GEO::mesh_load(std::string(TEST_DATA_PATH) + "delaunay3d_random_20.geogram", mesh));
            mesh.cells.connect();

            original_mesh.copy(mesh);
        }

        void for_each_c() {
            for (const auto& c : original_mesh.cells) {
                mesh.copy(original_mesh);

                /* Compute */
                perform_operation(c);

                /* Eval */
                check_connections();
            }
        }
        virtual void perform_operation(GEO::index_t c) {}

        void for_each_c_lf() {
            for (const auto& c : original_mesh.cells) {
                for (GEO::index_t lf = 0, lf_end = original_mesh.cells.nb_facets(c); lf < lf_end; ++lf) {
                    mesh.copy(original_mesh);

                    /* Compute */
                    perform_operation(c, lf);

                    /* Eval */
                    check_connections();
                }
            }
        }
        virtual void perform_operation(GEO::index_t c, GEO::index_t lf_or_le) {}

        void for_each_c_le() {
            for (const auto& c : original_mesh.cells) {
                for (GEO::index_t le = 0, le_end = original_mesh.cells.nb_edges(c); le < le_end; ++le) {
                    mesh.copy(original_mesh);

                    /* Compute */
                    perform_operation(c, le);

                    /* Eval */
                    check_connections();
                }
            }
        }

        /**
         * Verify that reconnecting the mesh preserves the current cell adjacency layout.
         */
        void check_connections() {
            std::vector<GEO::index_t> current_connections(4*mesh.cells.nb(), GEO::NO_CELL);
            for (const auto& c : mesh.cells) {
                for (GEO::index_t lf = 0; lf < 4; ++lf)
                    current_connections[4*c+lf] = mesh.cells.adjacent(c, lf);
            }

            mesh.cells.connect();
            for (const auto& c : mesh.cells) {
                for (GEO::index_t lf = 0; lf < 4; ++lf)
                    EXPECT_EQ(current_connections[4*c+lf], mesh.cells.adjacent(c, lf));
            }

            /* Rollback adjacency */
            for (const auto& c : mesh.cells) {
                for (GEO::index_t lf = 0; lf < 4; ++lf)
                    mesh.cells.set_adjacent(c, lf, current_connections[4*c+lf]);
            }
        }

        GEO::Mesh mesh;
        GEO::Mesh original_mesh;
    };

    /* ============================================================================================================= */

    class TetSplitTest : public TetOperationsTest {
    protected:
        void perform_operation(const GEO::index_t c) override {
            const GEO::index_t new_v = mesh.vertices.create_vertices(1);
            const GEO::index_t new_c = mesh.cells.create_tets(3);
            tet_split(mesh, c, new_v, new_c, new_c+1, new_c+2);
        }
    };

    TEST_F(TetSplitTest, tet_split) {
        for_each_c();
    }

    /* ============================================================================================================= */

    class TetFacetSplitTest : public TetOperationsTest {
    protected:
        void perform_operation(const GEO::index_t c, const GEO::index_t lf) override {
            const auto CELL_FACET_ON_BORDER = original_mesh.cells.adjacent(c, lf) == GEO::NO_CELL;

            const GEO::index_t new_v = mesh.vertices.create_vertices(1);
            GEO::index_t new_c0 = GEO::NO_CELL;
            GEO::index_t new_c1 = GEO::NO_CELL;
            GEO::index_t new_c2 = GEO::NO_CELL;
            GEO::index_t new_c3 = GEO::NO_CELL;
            if (CELL_FACET_ON_BORDER) {
                new_c0 = mesh.cells.create_tets(2);
                new_c1 = new_c0+1;
            }
            else {
                new_c0 = mesh.cells.create_tets(4);
                new_c1 = new_c0+1;
                new_c2 = new_c0+2;
                new_c3 = new_c0+3;
            }
            tet_facet_split(mesh, c, lf, new_v, new_c0, new_c1, new_c2, new_c3);
        }
    };

    TEST_F(TetFacetSplitTest, tet_facet_split) {
        for_each_c_lf();
    }

    /* ============================================================================================================= */

    class TetEdgeSplitTest : public TetOperationsTest {
    protected:
        void perform_operation(const GEO::index_t c, const GEO::index_t le) override {
            GEO::index_t EDGE_INCIDENT_TETS_NB = 0;
            const GEO::index_t ev0 = mesh.cells.edge_vertex(c, le, 0);
            const GEO::index_t ev1 = mesh.cells.edge_vertex(c, le, 1);
            for (const auto& cc : mesh.cells) {
                for (GEO::index_t lle = 0; lle < 6; ++lle) {
                    const GEO::index_t eev0 = mesh.cells.edge_vertex(cc, lle, 0);
                    const GEO::index_t eev1 = mesh.cells.edge_vertex(cc, lle, 1);
                    if ((eev0 == ev0 && eev1 == ev1) || (eev0 == ev1 && eev1 == ev0))
                        ++EDGE_INCIDENT_TETS_NB;
                }
            }

            const GEO::index_t new_v = mesh.vertices.create_vertices(1);
            const GEO::index_t new_c = mesh.cells.create_tets(EDGE_INCIDENT_TETS_NB);
            std::vector<GEO::index_t> new_cells(EDGE_INCIDENT_TETS_NB);
            std::iota(new_cells.begin(), new_cells.end(), new_c);

            tet_edge_split(mesh, c, le, new_v, &(new_cells[0]), GEO::Numeric::random_float32());

            for (const auto& status : new_cells)
                EXPECT_EQ(status, GEO::NO_CELL);
        }
    };

    TEST_F(TetEdgeSplitTest, tet_edge_split) {
        for_each_c_le();
    }

    /* ============================================================================================================= */

    class TetEdgeCollapseTest : public TetOperationsTest {
    protected:
        void perform_operation(const GEO::index_t c, const GEO::index_t le) override {
            const double r = GEO::Numeric::random_float32();
            if (!is_tet_edge_collapse_valid(mesh, c, le, r))
                return;

            GEO::index_t disuse_v;
            std::vector<GEO::index_t> disuse_cs;
            tet_edge_collapse(mesh, c, le, disuse_v, disuse_cs, r);

            /* Clean disuse vertices and cells */
            GEO::vector<GEO::index_t> cells_to_delete(mesh.cells.nb(), 0);
            for (const auto& cc : disuse_cs)
                cells_to_delete[cc] = 1;
            mesh.cells.delete_elements(cells_to_delete);
        }
    };

    TEST_F(TetEdgeCollapseTest, tet_edge_collapse) {
        for_each_c_le();
    }

    /* ============================================================================================================= */

    class TetEdgeSwap23Test : public TetOperationsTest {
    protected:
        void perform_operation(const GEO::index_t c, const GEO::index_t lf) override {
            if (!is_tet_edge_swap_2_3_valid(mesh, c, lf))
                return;

            const GEO::index_t new_c = mesh.cells.create_tets(1);
            tet_edge_swap_2_3(mesh, c, lf, new_c);
        }
    };

    TEST_F(TetEdgeSwap23Test, tet_edge_swap) {
        for_each_c_lf();
    }

    /* ============================================================================================================= */

    class TetEdgeSwap32Test : public TetOperationsTest {
    protected:
        void perform_operation(const GEO::index_t c, const GEO::index_t le) override {
            GEO::index_t disuse_c;

            if (const bool processed = tet_edge_swap_3_2(mesh, c, le, disuse_c)) {
                /* Clean disuse vertices and cells */
                ASSERT_LT(disuse_c, mesh.cells.nb());
                GEO::vector<GEO::index_t> cells_to_delete(mesh.cells.nb(), 0);
                cells_to_delete[disuse_c] = 1;
                mesh.cells.delete_elements(cells_to_delete);
            }
        }
    };

    TEST_F(TetEdgeSwap32Test, tet_edge_swap) {
        for_each_c_le();
    }
}