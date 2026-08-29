//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/29.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "hexex_io.h"
#include "line_stream.h"
#include <geolio/common/log.h>

namespace geolio
{
    bool HEXEX_IOHandler::load(
        const std::string& filename,
        GEO::Mesh& M,
        const GEO::MeshIOFlags& ioflags
        ) {
        M.clear();
        M.vertices.set_dimension(3);

        LineInput in(filename);
        if (!in.OK())
            return false;

        try {
            /* Load vertices */
            {
                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 1)
                    throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Expect vertices nb!");

                const GEO::index_t nb_vertices = in.field_as_uint(0);
                M.vertices.create_vertices(nb_vertices);

                for (GEO::index_t v = 0; v < nb_vertices; ++v) {
                    in.get_line();
                    in.get_fields();
                    if (in.nb_fields() != 3)
                        throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Invalid vertex, expected 3 coordinates!");

                    M.vertices.point(v).x = in.field_as_double(0);
                    M.vertices.point(v).y = in.field_as_double(1);
                    M.vertices.point(v).z = in.field_as_double(2);
                }
            }


            /* Load tetrahedra */
            {
                in.get_line();
                in.get_fields();
                if (in.nb_fields() != 1)
                    throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Expect tetrahedra nb!");

                const GEO::index_t nb_cells = in.field_as_uint(0);
                M.cells.create_tets(nb_cells);
                GEO::Attribute<GEO::vec3> cc_uvw(M.cell_corners.attributes(), "uvw");

                for (GEO::index_t c = 0; c < nb_cells; ++c) {
                    in.get_line();
                    in.get_fields();
                    if(in.nb_fields() != 16)
                        throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Invalid parameterized cell, expected 4 int + 12 double");

                    for (GEO::index_t lv = 0; lv < 4; ++lv)
                        M.cells.set_vertex(c, lv, in.field_as_uint(lv));
                    for (GEO::index_t lv = 0; lv < 4; ++lv) {
                        cc_uvw[M.cells.corner(c, lv)].x = in.field_as_appro_double(in, 4+3*lv);
                        cc_uvw[M.cells.corner(c, lv)].y = field_as_appro_double(in, 4+3*lv+1);
                        cc_uvw[M.cells.corner(c, lv)].z = field_as_appro_double(in, 4+3*lv+2);
                    }
                }
            }

            /* For MC3D-samples (https://github.com/HendrikBrueckler/MC3D-samples) */
            // if (in.get_line()) {
            //     std::vector<std::array<GEO::index_t, 3>> feature_facets;
            //
            //     in.get_fields();
            //
            //     if (in.nb_fields() == 1) { /* Load wall triangles */
            //         const GEO::index_t nb_wall_triangles = in.field_as_uint(0);
            //         M.facets.create_triangles(nb_wall_triangles);
            //         GEO::Attribute<double> f_dist_to_origin(M.facets.attributes(), "dist_to_origin");
            //
            //         for (GEO::index_t f = 0; f < nb_wall_triangles; ++f) {
            //             in.get_line();
            //             in.get_fields();
            //             if (in.nb_fields() != 4)
            //                 throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Invalid wall triangles, expected 3 coordinates + 1 double");
            //
            //             M.facets.set_vertex(f, 0, in.field_as_uint(0));
            //             M.facets.set_vertex(f, 1, in.field_as_uint(1));
            //             M.facets.set_vertex(f, 2, in.field_as_uint(2));
            //             f_dist_to_origin[f] = in.field_as_double(3);
            //         }
            //     }
            //     else if (in.nb_fields() == 3) { /* Load features */
            //         nb_feature_vertices = in.field_as_uint(0);
            //         nb_feature_edges = in.field_as_uint(1);
            //         nb_feature_facets = in.field_as_uint(2);
            //
            //         /* Load feature vertices */
            //         GEO::Attribute<bool> v_feature(M.vertices.attributes(), "feature");
            //         v_feature.fill(false);
            //         for (GEO::index_t v = 0; v < nb_feature_vertices; ++v) {
            //             in.get_line();
            //             in.get_fields();
            //             if (in.nb_fields() != 1)
            //                 throw("Line: " + String::to_string(in.line_number()) + ":Invalid feature vertex index");
            //             v_feature[in.field_as_uint(0)] = true;
            //         }
            //
            //         /* Load feature edges */
            //         M.edges.create_edges(nb_feature_edges);
            //         for (GEO::index_t e = 0; e < nb_feature_edges; ++e) {
            //             in.get_line();
            //             in.get_fields();
            //             if (in.nb_fields() != 2)
            //                 throw("Line: " + String::to_string(in.line_number()) + ":Invalid features edge vertices");
            //             M.edges.set_vertex(e, 0, in.field_as_uint(0));
            //             M.edges.set_vertex(e, 1, in.field_as_uint(1));
            //         }
            //
            //         /* Load feature facets */
            //         feature_facets.reserve(nb_feature_facets);
            //         for (GEO::index_t f = 0; f < nb_feature_facets; ++f) {
            //             in.get_line();
            //             in.get_fields();
            //             if (in.nb_fields() != 3)
            //                 throw("Line: " + String::to_string(in.line_number()) + ":Invalid features facet vertices");
            //             std::array<GEO::index_t, 3> fvs{
            //                 in.field_as_uint(0), in.field_as_uint(1), in.field_as_uint(2)
            //             };
            //             feature_facets.push_back(fvs);
            //         }
            //     }
            //     else
            //         throw("Line: " + String::to_string(in.line_number()) + ":Invalid keyword");
            //
            //     /* Label feature facets */
            //     if (!feature_facets.empty()) {
            //         GEO::Attribute<bool> f_feature(M.facets.attributes(), "feature");
            //         f_feature.fill(false);
            //
            //         if (M.facets.nb() == 0) { // not wall facets, create feature facets
            //             GEO::index_t new_f = M.facets.create_triangles(feature_facets.size());
            //             for (const auto& fvs : feature_facets) {
            //                 M.facets.set_vertex(new_f, 0, fvs[0]);
            //                 M.facets.set_vertex(new_f, 1, fvs[1]);
            //                 M.facets.set_vertex(new_f, 2, fvs[2]);
            //                 f_feature[new_f] = true;
            //                 ++new_f;
            //             }
            //         }
            //         else { // have wall facets, label feature facets
            //             std::unordered_set<std::array<GEO::index_t, 3>, hosm::Array3Hash<GEO::index_t>> feature_facets_set;
            //             for (auto& fvs : feature_facets) {
            //                 std::ranges::sort(fvs);
            //                 feature_facets_set.insert(fvs);
            //             }
            //
            //             for (const auto& f : M.facets) {
            //                 std::array<GEO::index_t, 3> fvs{
            //                     M.facets.vertex(f, 0), M.facets.vertex(f, 1), M.facets.vertex(f, 2)
            //                 };
            //                 std::ranges::sort(fvs);
            //                 if (feature_facets_set.contains(fvs))
            //                     f_feature[f] = true;
            //             }
            //         }
            //     }
            // }
        }
        catch (const std::string& what) {
            LOG::ERROR("{}", what);
            return false;
        }
        catch (const std::exception& ex) {
            LOG::ERROR("{}", ex.what());
            return false;
        }
        catch (...) {
            LOG::ERROR("Caught exception!");
            return false;
        }

        M.cells.connect();

        return true;
    }
}