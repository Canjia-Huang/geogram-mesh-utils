//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
// #include <geogram/mesh/mesh_io.h>
// #include <geolio/offset_RVD/offset_restricted_voronoi_diagram.h>
// #include <gtest/gtest.h>
// #include <geolio/offset_RVD/mesh_distance_field.h>
// #include <geolio/offset_RVD/exact_voronoi_diagram.h>
//
// namespace geolio::test
// {
//     TEST(OffsetRVDTest, test) {
//         GEO::Mesh mesh;
//         EXPECT_TRUE(mesh.load(std::string(TEST_DATA_PATH) + "fandisk.obj"));
//
//         std::vector<double> sites;
//         constexpr double d = 0;
//         {
//             sites.reserve(3*mesh.facets.nb());
//             for (const auto& f : mesh.facets) {
//                 const auto& center = (mesh.facets.point(f, 0)+mesh.facets.point(f, 1)+mesh.facets.point(f, 2))/3;
//                 const auto& normal = GEO::Geom::triangle_normal(mesh.facets.point(f, 0), mesh.facets.point(f, 1), mesh.facets.point(f, 2));
//                 const auto& p = center + d*normal;
//                 sites.push_back(p.x);
//                 sites.push_back(p.y);
//                 sites.push_back(p.z);
//             }
//         }
//
//         const auto distance_field = std::make_shared<MeshDistanceField>(mesh);
//
//         // OffsetRestrictedVoronoiDiagram RVD(distance_field);
//         // RVD.set_sites(sites.data(), sites.size()/3);
//         // RVD.compute(d);
//
//         ExactVoronoiDiagram EVD;
//         EVD.create_voronoi_cells(
//             sites.data(), sites.size()/3,
//             -1, 5, 11, 19, -3, 1);
//
//         {
//             GEO::Mesh mesh;
//             EVD.append_to_mesh(mesh);
//             mesh.save("debug.geogram");
//         }
//     }
// }

#include <gtest/gtest.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Isosurfacing_3/Value_function_3.h>
// CGAL 6.1+ 3D Isosurfacing 模块头文件
#include <CGAL/Isosurfacing_3/Cartesian_grid_3.h>
#include <CGAL/Isosurfacing_3/Marching_cubes_domain_3.h>
#include <CGAL/Isosurfacing_3/marching_cubes_3.h>
#include <CGAL/IO/polygon_soup_io.h>
#include <iostream>
#include <vector>
#include <geogram/basic/algorithm.h>

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using FT = Kernel::FT;
using Point_3 = Kernel::Point_3;
using Mesh = CGAL::Surface_mesh<Point_3>;

// AABB 树加速结构
using Primitive = CGAL::AABB_face_graph_triangle_primitive<Mesh>;
using Traits = CGAL::AABB_traits<Kernel, Primitive>;
using Tree = CGAL::AABB_tree<Traits>;

// 1. 距离计算 Oracle
class MeshOffsetOracle {
    const Mesh& mesh;
    bool is_closed;
    Tree tree;
    CGAL::Side_of_triangle_mesh<Mesh, Kernel> sotm;

public:
    MeshOffsetOracle(const Mesh& m)
        : mesh(m),
          is_closed(CGAL::is_closed(m)),
          tree(faces(m).first, faces(m).second, m),
          sotm(m) {}

    FT signed_distance(const Point_3& p) const {
        Point_3 cp = tree.closest_point(p);
        FT d = std::sqrt(CGAL::to_double((p - cp).squared_length()));
        // 如果内部为负，外部为正
        if (is_closed && sotm(p) == CGAL::ON_BOUNDED_SIDE) {
            d = -d;
        }
        return d;
    }
};

TEST(mc, test) {
    // 1. 读取输入网格
    Mesh mesh;
    if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh("/Users/canjia/Downloads/siggraph_open_fold_input_mesh.obj", mesh)) {
        std::cerr << "无法读取模型文件" << std::endl;
        return;
    }

    const FT offset_distance = 4; // 偏置距离（正数表示向外膨胀，负数表示向内收缩）

    // 2. 初始化距离 Oracle
    MeshOffsetOracle oracle(mesh);

    // 3. 计算 Grid 包围盒（输入网格 BBox 向外扩展安全边界）
    CGAL::Bbox_3 bbox = CGAL::Polygon_mesh_processing::bbox(mesh);
    double padding = std::abs(CGAL::to_double(offset_distance)) * 2.0;
    CGAL::Bbox_3 grid_bbox(
        bbox.xmin() - padding, bbox.ymin() - padding, bbox.zmin() - padding,
        bbox.xmax() + padding, bbox.ymax() + padding, bbox.zmax() + padding
    );

    // 划分 3D 笛卡尔网格（分辨率按需调整）
    using Grid = CGAL::Isosurfacing::Cartesian_grid_3<Kernel>;
    std::size_t resolution = 64;
    Grid grid(grid_bbox, CGAL::make_array(resolution, resolution, resolution));

    // 4. 建立 Marching Cubes 域
    auto distance_fn = [&oracle](const Point_3& p) {
        return oracle.signed_distance(p);
    };

    using Values = CGAL::Isosurfacing::Value_function_3<Grid>;
    Values values(distance_fn, grid);

    auto domain = CGAL::Isosurfacing::create_marching_cubes_domain_3(grid, values);

    // 5. 执行 Marching Cubes 抽取偏置等值面
    std::vector<Point_3> out_points;
    std::vector<std::vector<std::size_t>> out_triangles;

    std::cout << "正在运行 Marching Cubes..." << std::endl;
    CGAL::Isosurfacing::marching_cubes<CGAL::Parallel_if_available_tag>(
        domain,
        offset_distance, // 等值面阈值即为偏置距离
        out_points,
        out_triangles,
        CGAL::parameters::use_topologically_correct_marching_cubes(true) // 可选：拓扑正确配置
    );

    // 6. 保存输出模型
    CGAL::IO::write_polygon_soup("offset_mc.obj", out_points, out_triangles);
    std::cout << "完成！输出顶点数: " << out_points.size()
              << ", 三角面数: " << out_triangles.size() << std::endl;
}

#include <geogram/mesh/mesh.h>
TEST(direct, test) {
    GEO::Mesh mesh;
    mesh.load("/Users/canjia/Downloads/siggraph_open_fold_input_mesh.obj");

    GEO::Attribute<GEO::vec3> mesh_v_normal(mesh.vertices.attributes(), "normal");
    mesh_v_normal.fill(GEO::vec3(0, 0, 0));
    for (const auto& f : mesh.facets) {
        const auto normal = GEO::normalize(GEO::Geom::triangle_normal(mesh.facets.point(f, 0), mesh.facets.point(f, 1), mesh.facets.point(f, 2)));
        for (GEO::index_t lv = 0; lv < 3; ++lv)
            mesh_v_normal[mesh.facets.vertex(f, lv)] += normal;
    }
    for (const auto& v : mesh.vertices)
        mesh_v_normal[v] = GEO::normalize(mesh_v_normal[v]);

    constexpr double d = -4;
    GEO::Mesh mesh_out;
    GEO::index_t new_v = mesh_out.vertices.create_vertices(3*mesh.facets.nb());
    GEO::index_t new_f = mesh_out.facets.create_triangles(mesh.facets.nb());
    for (const auto& f : mesh.facets) {
        const auto normal = GEO::normalize(GEO::Geom::triangle_normal(mesh.facets.point(f, 0), mesh.facets.point(f, 1), mesh.facets.point(f, 2)));
        mesh_out.vertices.point(new_v) = mesh.facets.point(f, 0) + normal * d;
        mesh_out.vertices.point(new_v+1) = mesh.facets.point(f, 1) + normal * d;
        mesh_out.vertices.point(new_v+2) = mesh.facets.point(f, 2) + normal * d;
        mesh_out.facets.set_vertex(new_f, 0, new_v);
        mesh_out.facets.set_vertex(new_f, 1, new_v+1);
        mesh_out.facets.set_vertex(new_f, 2, new_v+2);
        new_v += 3;
        ++new_f;
    }
    // mesh_out.copy(mesh);
    // for (const auto& v : mesh.vertices)
        // mesh_out.vertices.point(v) += mesh_v_normal[v]*d;

    // mesh_out.save("debug.geogram");

    /* Random map */
    std::vector<GEO::index_t> region_to_new_region(mesh_out.facets.nb());
    GEO::index_t cnt = 0;
    std::ranges::generate(region_to_new_region,
                          [&cnt]() { return cnt++; });
    GEO::random_shuffle(region_to_new_region.begin(), region_to_new_region.end());

    /* Output */
    {
        std::ofstream out(R"(direct_offset.obj)");
        for (const auto& p : mesh_out.vertices.points())
            out << "v " << p.x << " " << p.y << " " << p.z << std::endl;

        GEO::index_t vt_nb = 0;
        for (const auto& f : mesh_out.facets) {
            const double r = static_cast<double>(region_to_new_region[f]) / mesh_out.facets.nb();
            for (GEO::index_t lv = 0; lv < mesh_out.facets.nb_vertices(f); ++lv)
                out << "vt " << r << " " << 0 << std::endl;
            out << "f";
            for (GEO::index_t lv = 0; lv < mesh_out.facets.nb_vertices(f); ++lv)
                out << " " << mesh_out.facets.vertex(f, lv)+1 << "/" << vt_nb+lv+1;
            out << std::endl;

            vt_nb += mesh_out.facets.nb_vertices(f);
        }
        out.close();
    }
}