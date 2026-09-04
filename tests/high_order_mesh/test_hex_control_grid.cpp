//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/4.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include <geolio/high_order_mesh/hex_control_grid.h>
#include <gtest/gtest.h>

#include "../utils.h"

namespace geolio::test
{
    class HexControlGridTest : public ::testing::Test {
    public:
        void save_high_order_mesh(const std::string& filepath) {
            ASSERT_FALSE(control_grid == nullptr);

            GEO::Mesh mesh_out;
            GEO::Attribute<GEO::index_t> mesh_out_v_cell(mesh_out.vertices.attributes(), "cell");
            GEO::Attribute<GEO::vec3> mesh_out_v_uvw(mesh_out.vertices.attributes(), "uvw");
            GEO::Attribute<GEO::index_t> mesh_out_f_cell(mesh_out.facets.attributes(), "cell");

            control_grid->append_discretized_high_order_cells_facets(
                mesh_out,
                20,
                &mesh_out_v_cell,
                &mesh_out_v_uvw,
                &mesh_out_f_cell);

            EXPECT_TRUE(mesh_out.save(filepath));
        }

    protected:
        GEO::Mesh mesh;
        std::unique_ptr<HexControlGrid> control_grid;
    };

    class SingleHexControlGridTest : public HexControlGridTest {
    protected:
        void SetUp() override {
            mesh.vertices.create_vertices(8);
            mesh.vertices.point(0) = GEO::vec3(0, 0, 0);
            mesh.vertices.point(1) = GEO::vec3(1, 0, 0);
            mesh.vertices.point(2) = GEO::vec3(0, 1, 0);
            mesh.vertices.point(3) = GEO::vec3(1, 1, 0);
            mesh.vertices.point(4) = GEO::vec3(0, 0, 1);
            mesh.vertices.point(5) = GEO::vec3(1, 0, 1);
            mesh.vertices.point(6) = GEO::vec3(0, 1, 1);
            mesh.vertices.point(7) = GEO::vec3(1, 1, 1);
            mesh.cells.create_hex(0, 1, 2, 3, 4, 5, 6, 7);

            constexpr GEO::index_t order = 5;
            control_grid = std::make_unique<HexControlGrid>(mesh, order);
        }
    };

    TEST_F(SingleHexControlGridTest, test) {
        save_high_order_mesh(get_current_test_name()+".geogram");
    }
}
