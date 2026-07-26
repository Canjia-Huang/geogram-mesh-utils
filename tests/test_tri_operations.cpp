//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <random>
#include <ranges>
#include <geogram/mesh/mesh_io.h>
#include <gtest/gtest.h>
#include "geolio/tri_operations.h"

namespace geolio::test
{
    class TriOperationsTest : public ::testing::Test {
    protected:
        void SetUp() override {
            ASSERT_TRUE(GEO::mesh_load(std::string(TEST_DATA_PATH) + "CDT_random_10.geogram", mesh));
            mesh.facets.connect();

            original_mesh.copy(mesh);
        }

        void compute() {
            for (const auto& f : original_mesh.facets) {
                for (GEO::index_t lv = 0, lv_end = original_mesh.facets.nb_vertices(f); lv < lv_end; ++lv) {
                    mesh.copy(original_mesh);

                    /* Compute */
                    perform_operation(f, lv);

                    /* Eval */
                    check_connections();
                }
            }
        }

        virtual void perform_operation(
            GEO::index_t f,
            GEO::index_t lv) = 0;

        /**
         * Verify that reconnecting the mesh preserves the current adjacency layout.
         */
        void check_connections() {
            std::vector<GEO::index_t> current_connections(3*mesh.facets.nb(), GEO::NO_FACET);
            for (const auto& f : mesh.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    current_connections[3*f+lv] = mesh.facets.adjacent(f, lv);
            }

            mesh.facets.connect();
            if constexpr (true) {
                for (const auto& f : mesh.facets) {
                    for (GEO::index_t lv = 0; lv < 3; ++lv)
                        if (current_connections[3*f+lv] != mesh.facets.adjacent(f, lv)) {
                            GEO::mesh_save(mesh, "debug.geogram");
                            ASSERT_TRUE(0);
                    }
                }
            }
            for (const auto& f : mesh.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    EXPECT_EQ(current_connections[3*f+lv], mesh.facets.adjacent(f, lv));
            }

            /* Rollback adjacency */
            for (const auto& f : mesh.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    mesh.facets.set_adjacent(f, lv, current_connections[3*f+lv]);
            }
        }

    public:
        GEO::Mesh mesh;
        GEO::Mesh original_mesh;
    };

    /* ============================================================================================================= */

    class TriEdgeSplitTest : public TriOperationsTest {
    protected:
        void perform_operation(
            const GEO::index_t f,
            const GEO::index_t lv
            ) override {
            const bool EDGE_ON_BORDER = original_mesh.facets.adjacent(f, lv) == GEO::NO_FACET;

            const GEO::index_t new_v = mesh.vertices.create_vertices(1);
            GEO::index_t new_f0 = GEO::NO_FACET;
            GEO::index_t new_f1 = GEO::NO_FACET;
            if (EDGE_ON_BORDER)
                new_f0 = mesh.facets.create_triangles(1);
            else {
                new_f0 = mesh.facets.create_triangles(2);
                new_f1 = new_f0+1;
            }
            tri_edge_split(mesh, f, lv, new_v, new_f0, new_f1, GEO::Numeric::random_float32());
        }
    };

    TEST_F(TriEdgeSplitTest, tri_edge_split) {
        compute();
    }

    /* ============================================================================================================= */

    class TriEdgeCollapseTest : public TriOperationsTest {
    protected:
        void perform_operation(
            const GEO::index_t f,
            const GEO::index_t lv
            ) override {
            const bool EDGE_ON_BORDER = original_mesh.facets.adjacent(f, lv) == GEO::NO_FACET;

            GEO::index_t disuse_v, disuse_f0, disuse_f1;
            tri_edge_collapse(mesh, f, lv, disuse_v, disuse_f0, disuse_f1, GEO::Numeric::random_float32());

            /* Clean disuse vertices and facets */
            GEO::vector<GEO::index_t> facets_to_delete(mesh.facets.nb(), 0);
            facets_to_delete[disuse_f0] = 1;
            if (EDGE_ON_BORDER)
                EXPECT_EQ(disuse_f1, GEO::NO_FACET);
            else
                EXPECT_NE(disuse_f1, GEO::NO_FACET);
            facets_to_delete[disuse_f1] = 1;
            mesh.facets.delete_elements(facets_to_delete);
        }
    };

    TEST_F(TriEdgeCollapseTest, tri_edge_collapse) {
        compute();
    }

    /* ============================================================================================================= */

    class TriEdgeSwapTest : public TriOperationsTest {
    protected:
        void perform_operation(
            const GEO::index_t f,
            const GEO::index_t lv
            ) override {
            if (is_tri_edge_swap_valid(mesh, f, lv))
                tri_edge_swap(mesh, f, lv);
        }
    };

    TEST_F(TriEdgeSwapTest, tri_edge_swap) {
        compute();
    }
}
