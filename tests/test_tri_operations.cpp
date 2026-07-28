//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include <numbers>
#include <random>
#include <ranges>
#include <geogram/mesh/mesh_io.h>
#include <gtest/gtest.h>

#include "utils.h"
#include "geolio/tri_operations.h"
#include "geolio/log.h"

namespace geolio::test
{
    class TriOperationsTest : public ::testing::Test {
    protected:
        void SetUp() override {
            generate_random_CDT2d_mesh(mesh, 20, 10);

            original_mesh.copy(mesh);
        }

        void for_each_f_lv() {
            for (const auto& f : original_mesh.facets) {
                for (GEO::index_t lv = 0, lv_end = original_mesh.facets.nb_vertices(f); lv < lv_end; ++lv) {
                    LOG::TRACE("f: {}/{}, lv: {}/{}", f, original_mesh.facets.nb(), lv, lv_end);

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
            GEO::Attribute<bool> mesh_fc_adj_error(mesh.facet_corners.attributes(), "adj_error");
            mesh_fc_adj_error.fill(false);
            for (const auto& f : mesh.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    EXPECT_EQ(current_connections[3*f+lv], mesh.facets.adjacent(f, lv));
                    mesh_fc_adj_error[mesh.facets.corner(f, lv)] = current_connections[3*f+lv] != mesh.facets.adjacent(f, lv);
                }
            }

            /* Rollback adjacency */
            for (const auto& f : mesh.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv)
                    mesh.facets.set_adjacent(f, lv, current_connections[3*f+lv]);
            }
        }

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
        for_each_f_lv();
    }

    /* ============================================================================================================= */

    class TriEdgeCollapseTest : public TriOperationsTest {
    protected:
        void perform_operation(
            const GEO::index_t f,
            const GEO::index_t lv
            ) override {
            const double r = GEO::Numeric::random_float32();

            if (!is_tri_edge_collapse_valid(mesh, f, lv))
                return;

            const bool EDGE_ON_BORDER = original_mesh.facets.adjacent(f, lv) == GEO::NO_FACET;

            GEO::index_t disuse_v, disuse_f0, disuse_f1;
            tri_edge_collapse(mesh, f, lv, disuse_v, disuse_f0, disuse_f1, r);

            /* Clean disuse vertices and facets */
            GEO::vector<GEO::index_t> facets_to_delete(mesh.facets.nb(), 0);
            facets_to_delete[disuse_f0] = 1;
            if (EDGE_ON_BORDER)
                EXPECT_EQ(disuse_f1, GEO::NO_FACET);
            else {
                EXPECT_NE(disuse_f1, GEO::NO_FACET);
                facets_to_delete[disuse_f1] = 1;
            }
            mesh.facets.delete_elements(facets_to_delete);
        }
    };

    TEST_F(TriEdgeCollapseTest, tri_edge_collapse) {
        for_each_f_lv();
    }

    TEST_F(TriEdgeCollapseTest, degenerate_2d) {
        const std::array<GEO::vec2, 13> vertices = {
            GEO::vec2(0, 0), GEO::vec2(1, 0), GEO::vec2(2, 0),
            GEO::vec2(0, 1), GEO::vec2(1, 1), GEO::vec2(2, 1),
            GEO::vec2(0, 2), GEO::vec2(1, 2), GEO::vec2(2, 2),
            GEO::vec2(0, 3), GEO::vec2(1, 3), GEO::vec2(2, 3),
            GEO::vec2(1.2, 1.8)
        };
        const std::array<GEO::index_t, 14*3> facets = {
            0, 1, 4, 0, 4, 3, 1, 2, 5, 1, 5, 4,
            3, 4, 7, 3, 7, 6, 4, 12, 7, 4, 5, 12, 5, 8, 12, 7, 12, 8,
            6, 7, 10, 6, 10, 9, 7, 8, 11, 7, 11, 10
        };
        mesh.clear();
        mesh.vertices.set_dimension(2);
        mesh.vertices.create_vertices(vertices.size());
        for (const auto& v : mesh.vertices)
            mesh.vertices.point<2>(v) = vertices[v];
        mesh.facets.create_triangles(facets.size()/3);
        for (const auto& f : mesh.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv)
                mesh.facets.set_vertex(f, lv, facets[3*f+lv]);
        }
        mesh.facets.connect();
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        /* Collapse */
        constexpr GEO::index_t f = 4;
        constexpr GEO::index_t lv = 1;

        GEO::index_t disuse_v, disuse_f0, disuse_f1;
        tri_edge_collapse(mesh, f, lv, disuse_v, disuse_f0, disuse_f1);
        { // clean up
            GEO::vector<GEO::index_t> facets_to_delete(mesh.facets.nb(), 0);
            ASSERT_NE(disuse_f0, GEO::NO_FACET);
            facets_to_delete[disuse_f0] = 1;
            if (disuse_f1 != GEO::NO_FACET)
                facets_to_delete[disuse_f1] = 1;
            mesh.facets.delete_elements(facets_to_delete);
        }
        check_connections();

        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");
    }

    TEST_F(TriEdgeCollapseTest, degenerate_2d_progressive) {
        const std::array<GEO::vec2, 13> vertices = {
            GEO::vec2(0, 0), GEO::vec2(1, 0), GEO::vec2(2, 0),
            GEO::vec2(0, 1), GEO::vec2(1, 1), GEO::vec2(2, 1),
            GEO::vec2(0, 2), GEO::vec2(1, 2), GEO::vec2(2, 2),
            GEO::vec2(0, 3), GEO::vec2(1, 3), GEO::vec2(2, 3),
            GEO::vec2(1.2, 1.8)
        };
        const std::array<GEO::index_t, 14*3> facets = {
            0, 1, 4, 0, 4, 3, 1, 2, 5, 1, 5, 4,
            3, 4, 7, 3, 7, 6, 4, 12, 7, 4, 5, 12, 5, 8, 12, 7, 12, 8,
            6, 7, 10, 6, 10, 9, 7, 8, 11, 7, 11, 10
        };
        mesh.clear();
        mesh.vertices.set_dimension(2);
        mesh.vertices.create_vertices(vertices.size());
        for (const auto& v : mesh.vertices)
            mesh.vertices.point<2>(v) = vertices[v];
        mesh.facets.create_triangles(facets.size()/3);
        for (const auto& f : mesh.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv)
                mesh.facets.set_vertex(f, lv, facets[3*f+lv]);
        }
        mesh.facets.connect();
        // GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        /* Collapse */
        GEO::index_t cnt = 1;
        while (mesh.facets.nb() > 0) {
            bool collapse = false;
            for (const auto& f : mesh.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (!is_tri_edge_collapse_valid(mesh, f, lv))
                        continue;

                    GEO::index_t disuse_v, disuse_f0, disuse_f1;
                    tri_edge_collapse(mesh, f, lv, disuse_v, disuse_f0, disuse_f1);
                    { // clean up
                        GEO::vector<GEO::index_t> facets_to_delete(mesh.facets.nb(), 0);
                        ASSERT_NE(disuse_f0, GEO::NO_FACET);
                        facets_to_delete[disuse_f0] = 1;
                        if (disuse_f1 != GEO::NO_FACET)
                            facets_to_delete[disuse_f1] = 1;
                        mesh.facets.delete_elements(facets_to_delete);
                    }
                    check_connections();

                    // GEO::mesh_save(mesh, get_current_test_name()+"_"+std::to_string(cnt++)+".geogram");

                    collapse = true;
                    break;
                }
                if (collapse)
                    break;
            }
        }
        EXPECT_EQ(mesh.facets.nb(), 0);
    }

    TEST_F(TriEdgeCollapseTest, degenerate_3d) {
        const std::array<GEO::vec3, 13> vertices = {
            GEO::vec3(0, 0, GEO::Numeric::random_float32()), GEO::vec3(1, 0, GEO::Numeric::random_float32()), GEO::vec3(2, 0, GEO::Numeric::random_float32()),
            GEO::vec3(0, 1, GEO::Numeric::random_float32()), GEO::vec3(1, 1, GEO::Numeric::random_float32()), GEO::vec3(2, 1, GEO::Numeric::random_float32()),
            GEO::vec3(0, 2, GEO::Numeric::random_float32()), GEO::vec3(1, 2, GEO::Numeric::random_float32()), GEO::vec3(2, 2, GEO::Numeric::random_float32()),
            GEO::vec3(0, 3, GEO::Numeric::random_float32()), GEO::vec3(1, 3, GEO::Numeric::random_float32()), GEO::vec3(2, 3, GEO::Numeric::random_float32()),
            GEO::vec3(1.2, 1.8, GEO::Numeric::random_float32())
        };
        const std::array<GEO::index_t, 14*3> facets = {
            0, 1, 4, 0, 4, 3, 1, 2, 5, 1, 5, 4,
            3, 4, 7, 3, 7, 6, 4, 12, 7, 4, 5, 12, 5, 8, 12, 7, 12, 8,
            6, 7, 10, 6, 10, 9, 7, 8, 11, 7, 11, 10
        };
        mesh.clear();
        mesh.vertices.create_vertices(vertices.size());
        for (const auto& v : mesh.vertices)
            mesh.vertices.point(v) = vertices[v];
        mesh.facets.create_triangles(facets.size()/3);
        for (const auto& f : mesh.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv)
                mesh.facets.set_vertex(f, lv, facets[3*f+lv]);
        }
        mesh.facets.connect();
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        /* Collapse */
        constexpr GEO::index_t f = 4;
        constexpr GEO::index_t lv = 1;

        GEO::index_t disuse_v, disuse_f0, disuse_f1;
        tri_edge_collapse(mesh, f, lv, disuse_v, disuse_f0, disuse_f1);
        { // clean up
            GEO::vector<GEO::index_t> facets_to_delete(mesh.facets.nb(), 0);
            ASSERT_NE(disuse_f0, GEO::NO_FACET);
            facets_to_delete[disuse_f0] = 1;
            if (disuse_f1 != GEO::NO_FACET)
                facets_to_delete[disuse_f1] = 1;
            mesh.facets.delete_elements(facets_to_delete);
        }
        check_connections();

        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");
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
        for_each_f_lv();
    }

    TEST_F(TriEdgeSwapTest, degenerate_2d) {
        const std::array<GEO::vec2, 7> vertices = {
            GEO::vec2(0, 0), GEO::vec2(1, 0), GEO::vec2(2, 0),
            GEO::vec2(1, 1),
            GEO::vec2(0, 2), GEO::vec2(1, 2), GEO::vec2(2, 2),
        };
        const std::array<GEO::index_t, 6*3> facets = {
            0, 3, 5, 5, 3, 2,
            // 0, 2, 3,
            0, 1, 3, 1, 2, 3,
            4, 0, 5, 5, 2, 6
        };
        mesh.clear();
        mesh.vertices.set_dimension(2);
        mesh.vertices.create_vertices(vertices.size());
        for (const auto& v : mesh.vertices)
            mesh.vertices.point<2>(v) = vertices[v];
        mesh.facets.create_triangles(facets.size()/3);
        for (const auto& f : mesh.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv)
                mesh.facets.set_vertex(f, lv, facets[3*f+lv]);
        }
        mesh.facets.connect();
        GEO::mesh_save(mesh, get_current_test_name()+"_0.geogram");

        /* Collapse */
        constexpr GEO::index_t f = 0;
        constexpr GEO::index_t lv = 1;

        tri_edge_swap(mesh, f, lv);
        check_connections();

        GEO::mesh_save(mesh, get_current_test_name()+"_1.geogram");
    }
}
