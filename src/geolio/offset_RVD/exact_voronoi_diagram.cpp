//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "exact_voronoi_diagram.h"
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <geolio/common/log.h>

typedef Kernel::Point_3                                    Point_3;
typedef CGAL::Delaunay_triangulation_3<Kernel>                  Delaunay_triangulation;
typedef Delaunay_triangulation::Vertex_handle              DT_Vertex_handle;
typedef Delaunay_triangulation::Cell_handle                DT_Cell_handle;
typedef Polyhedron::Vertex_const_handle                    Vertex_const_handle;
typedef Polyhedron::Vertex_const_iterator                  Vertex_const_iterator;
typedef Polyhedron::Facet_const_iterator                   Facet_const_iterator;
typedef Polyhedron::Halfedge_around_facet_const_circulator Halfedge_around_facet_const_circulator;

namespace geolio
{
    void ExactVoronoiDiagram::create_voronoi_cells(
        const double* sites,
        const GEO::index_t sites_nb,
        double x_min,
        double x_max,
        double y_min,
        double y_max,
        double z_min,
        double z_max
        ) {
        LOG::TRACE(__FUNCTION__);

        /* Prepare points */
        std::vector<Point_3> points;
        points.reserve(sites_nb);
        for (GEO::index_t i = 0; i < sites_nb; ++i)
            points.emplace_back(sites[3*i], sites[3*i+1], sites[3*i+2]);

        /* Prepare bounding box */
        Polyhedron bbox_mesh;
        CGAL::make_hexahedron(
            Point_3(x_min, y_min, z_min), Point_3(x_max, y_min, z_min), Point_3(x_max, y_max, z_min), Point_3(x_min, y_max, z_min),
            Point_3(x_min, y_min, z_max), Point_3(x_max, y_min, z_max), Point_3(x_max, y_max, z_max), Point_3(x_min, y_max, z_max),
            bbox_mesh);
        CGAL::Polygon_mesh_processing::triangulate_faces(bbox_mesh);

        /* Compute Delaunay triangulation */
        Delaunay_triangulation DT(points.begin(), points.end());

        /* Initialize Voronoi cells */
        std::vector<DT_Vertex_handle> vertex_handles;
        vertex_handles.reserve(sites_nb);
        voronoi_cells_.clear();
        voronoi_cells_.reserve(sites_nb);
        for (auto vit = DT.finite_vertices_begin(); vit != DT.finite_vertices_end(); ++vit) {
            vertex_handles.push_back(vit);

            Polyhedron poly_cell = bbox_mesh;
            voronoi_cells_.emplace_back(poly_cell);
        }

        /* Clip Voronoi cells */
        auto clip_voronoi_cell = [&](const GEO::index_t v) {
            if (v % 1000 == 0)
                LOG::TRACE("{}", v);

            assert(v < sites_nb);
            const auto& vh = vertex_handles[v];
            auto& poly = voronoi_cells_[v].polyhedron();

            std::vector<DT_Vertex_handle> neighbors;
            DT.incident_vertices(vh, std::back_inserter(neighbors));

            for (const auto n_it : neighbors) {
                if (DT.is_infinite(n_it))
                    continue;

                const Point_3 p = n_it->point();
                const Point_3 q = vh->point();
                const Kernel::Plane_3 clipping_plane = CGAL::bisector(p, q);

                CGAL::Polygon_mesh_processing::clip(
                    poly,
                    clipping_plane,
                    CGAL::parameters::clip_volume(true));
            }
        };
        GEO::parallel_for(0, sites_nb, clip_voronoi_cell, 2, true);
    }

    void ExactVoronoiDiagram::append_to_mesh(
        GEO::Mesh& mesh
        ) {
        LOG::TRACE(__FUNCTION__);

        while (voronoi_cells_.size() > 1)
            voronoi_cells_.pop_back();

        /* Count elements nb */
        GEO::index_t vertices_nb = 0;
        GEO::index_t facets_nb = 0;
        for (const auto& VC : voronoi_cells_) {
            const auto& poly = VC.polyhedron();
            vertices_nb += poly.size_of_vertices();
            facets_nb += poly.size_of_facets();
        }
        LOG::DEBUG("{}, {}", vertices_nb, facets_nb);

        /* Create elements */
        GEO::index_t new_v = mesh.vertices.create_vertices(vertices_nb);
        GEO::index_t new_f = mesh.facets.create_triangles(facets_nb);
        for (const auto& VC : voronoi_cells_) {
            const auto& poly = VC.polyhedron();

            CGAL::Unique_hash_map<Vertex_const_handle, GEO::index_t> vh_to_v;
            for (Vertex_const_iterator vit = poly.vertices_begin(); vit != poly.vertices_end(); ++vit) {
                vh_to_v[vit] = new_v;

                const Point_3& p = vit->point();
                mesh.vertices.point(new_v++) = GEO::vec3(CGAL::to_double(p.x()), CGAL::to_double(p.y()), CGAL::to_double(p.z()));
            }

            for (Facet_const_iterator fit = poly.facets_begin(); fit != poly.facets_end(); ++fit) {
                Halfedge_around_facet_const_circulator hfc = fit->facet_begin();
                const Halfedge_around_facet_const_circulator end = hfc;

                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    assert(hfc != end);
                    Vertex_const_handle vh = hfc->vertex();
                    mesh.facets.set_vertex(new_f, lv, vh_to_v[vh]);
                    ++hfc;
                }

                ++new_f;
            }
        }
    }
}