//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <ranges>
#include <unordered_set>
#include "test_tet_opeartions.h"
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

        /**
         * Verify that reconnecting the mesh preserves the current cell adjacency layout.
         */
        void check_connections() {
            // clean_inverse_tets();

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
}

    /* ============================================================================================================= */










/* == CellSplitTest ================================================================================================ */

namespace geolio::test
{
    class CellSplitTest : public TetrahedronOperationsTest {
    public:
        void compute(
            const GEO::index_t c
            ) {
            const GEO::index_t new_v = M.vertices.create_vertices(1);
            const GEO::index_t new_c = M.cells.create_tets(3);

            M_c_affected[c] = 1;
            M_c_affected[new_c] = 1;
            M_c_affected[new_c+1] = 1;
            M_c_affected[new_c+2] = 1;

            tet_split(
                M,
                c,
                new_v,
                new_c, new_c+1, new_c+2);
        }
    };

    class InteriorCellSplitTest : public CellSplitTest {};

    TEST_P(InteriorCellSplitTest, each_cell) {
        auto [c, _, __] = GetParam();

        compute(c);
        check_connections();
        save_results_c(c);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        InteriorCellSplitTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_FACET_C_LF)));

    class BorderCellSplitTest : public CellSplitTest {};

    TEST_P(BorderCellSplitTest, each_cell) {
        auto [c, _, __] = GetParam();

        compute(c);
        check_connections();
        save_results_c(c);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        BorderCellSplitTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_FACET_C_LF)));
}

/* == CellFacetSplit =============================================================================================== */

namespace geolio::test
{
    class CellFacetSplitTest : public TetrahedronOperationsTest {};

    class InteriorCellFacetSplitTest : public CellFacetSplitTest {
    public:
        void compute(
            const GEO::index_t c,
            const GEO::index_t lf
            ) {
            ASSERT_FALSE(M.cells.adjacent(c, lf) == GEO::NO_CELL);
            const GEO::index_t new_v = M.vertices.create_vertices(1);
            const GEO::index_t new_c = M.cells.create_tets(4);

            M_c_affected[c] = 1;
            M_c_affected[M.cells.adjacent(c, lf)] = 1;
            M_c_affected[new_c] = 1;
            M_c_affected[new_c+1] = 1;
            M_c_affected[new_c+2] = 1;
            M_c_affected[new_c+3] = 1;

            tet_facet_split(M, c, lf, new_v, new_c, new_c+1, new_c+2, new_c+3);
        }
    };

    TEST_P(InteriorCellFacetSplitTest, each_facet) {
        auto [c, lf, __] = GetParam();

        compute(c, lf);
        check_connections();
        save_results_c_lf(c, lf);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        InteriorCellFacetSplitTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_FACET_C_LF)));

    class BorderCellFacetSplitTest : public CellFacetSplitTest {
    public:
        void compute(
            const GEO::index_t c,
            const GEO::index_t lf
            ) {
            ASSERT_TRUE(M.cells.adjacent(c, lf) == GEO::NO_CELL);
            const GEO::index_t new_v = M.vertices.create_vertices(1);
            const GEO::index_t new_c = M.cells.create_tets(2);

            M_c_affected[c] = 1;
            M_c_affected[new_c] = 1;
            M_c_affected[new_c+1] = 1;

            tet_facet_split(M, c, lf, new_v, new_c, new_c+1);
        }
    };

    TEST_P(BorderCellFacetSplitTest, each_facet) {
        auto [c, lf, __] = GetParam();

        compute(c, lf);
        check_connections();
        save_results_c_lf(c, lf);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        BorderCellFacetSplitTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_FACET_C_LF)));
}

/* == CellEdgeSplitTest ============================================================================================ */

namespace geolio::test
{
    class CellEdgeSplitTest : public TetrahedronOperationsTest {
    public:
        void compute(
            const GEO::index_t c,
            const GEO::index_t le
            ) {
            const GEO::index_t ev0 = M.cells.edge_vertex(c, le, 0);
            const GEO::index_t ev1 = M.cells.edge_vertex(c, le, 1);
            for (const auto& cc : M.cells) {
                for (GEO::index_t lle = 0; lle < 6; ++lle) {
                    const GEO::index_t eev0 = M.cells.edge_vertex(cc, lle, 0);
                    const GEO::index_t eev1 = M.cells.edge_vertex(cc, lle, 1);
                    if ((eev0 == ev0 && eev1 == ev1) || (eev0 == ev1 && eev1 == ev0)) {
                        M_c_affected[cc] = 1;
                        break;
                    }
                }
            }

            const GEO::index_t new_v = M.vertices.create_vertices(1);
            GEO::index_t new_c = M.cells.create_tets(std::ceil(10.0*GEO::Numeric::random_float32()));

            tet_edge_split(M, c, le, new_v, new_c, GEO::Numeric::random_float32());

            /* Delete unuse cells */
            GEO::vector<GEO::index_t> cells_to_delete(M.cells.nb(), 0);
            for (GEO::index_t cc = new_c; cc < M.cells.nb(); ++cc)
                cells_to_delete[cc] = 1;
            M.cells.delete_elements(cells_to_delete);
        }
    };

    class InteriorCellEdgeSplitTest : public CellEdgeSplitTest {};

    TEST_P(InteriorCellEdgeSplitTest, each_edge) {
        auto [c, le, __] = GetParam();

        compute(c, le);
        check_connections();
        save_results_c_le(c, le);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        InteriorCellEdgeSplitTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_EDGE_C_LE)));

    class BorderCellEdgeSplitTest : public CellEdgeSplitTest {};

    TEST_P(BorderCellEdgeSplitTest, each_edge) {
        auto [c, le, __] = GetParam();

        compute(c, le);
        check_connections();
        save_results_c_le(c, le);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        BorderCellEdgeSplitTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_EDGE_C_LE)));
}

/* == CellEdgeCollapseTest ========================================================================================= */

namespace geolio::test
{
    class CellEdgeCollapseTest : public TetrahedronOperationsTest {
    public:
        void compute(
            const GEO::index_t c,
            const GEO::index_t le,
            const double r
            ) {
            GEO::index_t disuse_v;
            std::vector<GEO::index_t> disuse_cs;

            const GEO::index_t ev0 = M.cells.edge_vertex(c, le, 0);

            tet_edge_collapse(
                M,
                c,
                le,
                disuse_v,
                disuse_cs,
                r);

            EXPECT_NE(disuse_v, ev0);
            for (const auto& cc : M.cells) {
                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    if (M.cells.vertex(cc, lv) == ev0) {
                        M_c_affected[cc] = 1;
                        break;
                    }
                }
            }

            /* Clean disuse vertices and cells */
            GEO::vector<GEO::index_t> cells_to_delete(M.cells.nb(), 0);
            for (const auto& cc : disuse_cs)
                cells_to_delete[cc] = 1;
            M.cells.delete_elements(cells_to_delete);
        }
    };

    class InteriorCellEdgeCollapseTest : public CellEdgeCollapseTest {};

    TEST_P(InteriorCellEdgeCollapseTest, each_edge) {
        auto [c, le, _] = GetParam();

        compute(c, le, GEO::Numeric::random_float32());
        check_connections();
        save_results_c_le(c, le);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        InteriorCellEdgeCollapseTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_EDGE_C_LE)));

    class BorderCellEdgeCollapseTest : public CellEdgeCollapseTest {};

    TEST_P(BorderCellEdgeCollapseTest, each_edge) {
        auto [c, le, _] = GetParam();

        compute(c, le, GEO::Numeric::random_float32());
        check_connections();
        save_results_c_le(c, le);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        BorderCellEdgeCollapseTest,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_EDGE_C_LE)));
}

/* == CellEdgeSwap23Test =========================================================================================== */

namespace geolio::test
{
    class CellEdgeSwap23Test : public TetrahedronOperationsTest {};

    class InteriorCellEdgeSwap23Test : public CellEdgeSwap23Test {
    public:
        bool compute(
            const GEO::index_t c,
            const GEO::index_t lf
            ) {
            const GEO::index_t new_c = M.cells.create_tets(1);

            M_c_affected[c] = 1;
            M_c_affected[M.cells.adjacent(c, lf)] = 1;
            M_c_affected[new_c] = 1;

            return tet_edge_swap_2_3(
                M,
                c, lf,
                new_c);
        }
    };

    TEST_P(InteriorCellEdgeSwap23Test, each_facet) {
        auto [c, lf, _] = GetParam();

        EXPECT_TRUE(compute(c, lf));
        check_connections();
        save_results_c_lf(c, lf);
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        InteriorCellEdgeSwap23Test,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_FACET_C_LF)));

    class BorderCellEdgeSwap23Test : public CellEdgeSwap23Test {
    public:
        bool compute(
            const GEO::index_t c,
            const GEO::index_t lf
            ) {
            M_c_affected[c] = 1;

            return tet_edge_swap_2_3(
                M,
                c, lf,
                GEO::NO_CELL);
        }
    };

    TEST_P(BorderCellEdgeSwap23Test, each_facet) {
        auto [c, lf, _] = GetParam();

        EXPECT_FALSE(compute(c, lf));
        check_connections();
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        BorderCellEdgeSwap23Test,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(BORDER_FACET_C_LF)));
}

/* == CellEdgeSwap32Test =========================================================================================== */

namespace geolio::test
{
    class CellEdgeSwap32Test : public TetrahedronOperationsTest {
    public:
        void compute(
            const GEO::index_t c,
            const GEO::index_t le,
            bool& processed
            ) {
            std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_c_and_lf;
            // get_edge_incident_tetrahedra(M, c, le, ordered_c_and_lf);
            for (const auto &cc: ordered_c_and_lf | std::views::keys)
                M_c_affected[cc] = 1;

            GEO::index_t disuse_c = GEO::NO_CELL;

            processed = tet_edge_swap_3_2(
                M,
                c,
                le,
                disuse_c);

            if (processed) {
                /* Clean disuse vertices and cells */
                ASSERT_LT(disuse_c, M.cells.nb());
                GEO::vector<GEO::index_t> cells_to_delete(M.cells.nb(), 0);
                cells_to_delete[disuse_c] = 1;
                M.cells.delete_elements(cells_to_delete);
            }
        }
    };

    TEST_P(CellEdgeSwap32Test, each_edge) {
        auto [c, le, _] = GetParam();

        bool processed;
        compute(c, le, processed);

        if (processed) {
            check_connections();
            save_results_c_le(c, le);
        }
    }

    INSTANTIATE_TEST_SUITE_P(
        TetrahedronOperationsTest,
        CellEdgeSwap32Test,
        ::testing::ValuesIn(TETRAHEDRON_MESH_GET_TEST_PARAMS(INTERIOR_EDGE_C_LE)));
}