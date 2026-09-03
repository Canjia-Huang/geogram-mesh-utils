//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/3.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "ovm_io.h"
#include "line_stream.h"
#include <geolio/common/log.h>

namespace geolio
{
    bool OVM_IOHandler::load(
        const std::string& filename,
        GEO::Mesh& M,
        const GEO::MeshIOFlags& ioflags
        ) {
        M.clear();
        M.vertices.set_dimension(3);

        LineInput in(filename);
        if (!in.OK()) {
            LOG::ERROR("Cannot load file `{}`!", filename);
            return false;
        }

        /* Header */
        in.get_line();
        in.get_fields();
        if(in.nb_fields() != 2 || strcmp(in.field(0),"OVM") != 0 || strcmp(in.field(1),"ASCII") != 0) {
            LOG::ERROR("Invalid file header `{}` - `{}`!", in.field(0), in.field(1));
            return false;
        }

        try {
            while(!in.eof()) {
                if (!in.get_line())
                    break; // back empty line
                in.get_fields();
                if (in.nb_fields() != 1)
                    throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Invalid keyword format!");

                if (const std::string kw = in.field(0);
                    kw == "Vertices") { // == Vertices ================================================================
                    in.get_line();
                    in.get_fields();
                    if (in.nb_fields() != 1)
                        throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Invalid vertices nb format!");

                    const GEO::index_t nb_vertices = in.field_as_uint(0);
                    M.vertices.create_vertices(nb_vertices);

                    for (GEO::index_t v = 0; v < nb_vertices; ++v) {
                        in.get_line();
                        in.get_fields();
                        if(in.nb_fields() != 3)
                            throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Invalid vertex, expected 3 coordinates!");

                        auto& p = M.vertices.point(v);
                        p.x = in.field_as_double(0);
                        p.y = in.field_as_double(1);
                        p.z = in.field_as_double(2);
                    }
                }
                else if (kw == "Edges") { // == Edges =================================================================
                    in.get_line();
                    in.get_fields();
                    if (in.nb_fields() != 1)
                        throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Invalid edges nb format!");

                    const GEO::index_t nb_edges = in.field_as_uint(0);
                    M.edges.create_edges(nb_edges);

                    for (GEO::index_t e = 0; e < nb_edges; ++e) {
                        in.get_line();
                        in.get_fields();
                        if(in.nb_fields() != 2)
                            throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Invalid edge, expected 2 indices!");

                        M.edges.set_vertex(e, 0, in.field_as_uint(0));
                        M.edges.set_vertex(e, 1, in.field_as_uint(1));
                    }
                }
                else if (kw == "Faces") { // == Facets =================================================================
                    in.get_line();
                    in.get_fields();
                    if (in.nb_fields() != 1)
                        throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Invalid facets nb format!");

                    const GEO::index_t nb_facets = in.field_as_uint(0);

                    std::vector<GEO::index_t> facet_ptr; // nb_vertices, v0, v1, ..., vn, nb_vertices, v1, ...
                    GEO::index_t triangles_nb = 0;
                    GEO::index_t quads_nb = 0;
                    GEO::index_t polygons_nb = 0;
                    for (GEO::index_t f = 0; f < nb_facets; ++f) {
                        in.get_line();
                        in.get_fields();
                        if(in.nb_fields() == 0)
                            throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Invalid facet, empty line!");

                        const GEO::index_t fv_nb = in.field_as_uint(0);
                        if (fv_nb == 3)
                            ++triangles_nb;
                        else if (fv_nb == 4)
                            ++quads_nb;
                        else if (fv_nb > 4)
                            ++polygons_nb;
                        else
                            throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Invalid facet vertices nb "+std::to_string(fv_nb)+"!");

                        if (in.nb_fields() != fv_nb+1)
                            throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Invalid facet, wrong number of elements!");
                        facet_ptr.push_back(fv_nb);

                        for (GEO::index_t lv = 0; lv < fv_nb; ++lv) {
                            const GEO::index_t ie = in.field_as_uint(1+lv);
                            const GEO::index_t e = ie/2;
                            if(e > M.edges.nb())
                                throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Invalid edge id in facet!");
                            if (ie%2 == 0)
                                facet_ptr.push_back(M.edges.vertex(e, 0));
                            else
                                facet_ptr.push_back(M.edges.vertex(e, 1));
                        }
                    }

                    /* Create facets */
                    if (triangles_nb != 0) {
                        GEO::index_t new_f = M.facets.create_triangles(triangles_nb);
                        for (GEO::index_t i = 0, i_end = facet_ptr.size(); i < i_end; ++i) {
                            const GEO::index_t fv_nb = facet_ptr[i];
                            if (fv_nb == 3) {
                                M.facets.set_vertex(new_f, 0, facet_ptr[i+1]);
                                M.facets.set_vertex(new_f, 1, facet_ptr[i+2]);
                                M.facets.set_vertex(new_f, 2, facet_ptr[i+3]);
                                ++new_f;
                            }
                            i += fv_nb;
                        }
                        assert(new_f == M.facets.nb());
                    }
                    if (quads_nb != 0) {
                        GEO::index_t new_f = M.facets.create_quads(quads_nb);
                        for (GEO::index_t i = 0, i_end = facet_ptr.size(); i < i_end; ++i) {
                            const GEO::index_t fv_nb = facet_ptr[i];
                            if (fv_nb == 4) {
                                M.facets.set_vertex(new_f, 0, facet_ptr[i+1]);
                                M.facets.set_vertex(new_f, 1, facet_ptr[i+2]);
                                M.facets.set_vertex(new_f, 2, facet_ptr[i+3]);
                                M.facets.set_vertex(new_f, 3, facet_ptr[i+4]);
                                ++new_f;
                            }
                            i += fv_nb;
                        }
                        assert(new_f == M.facets.nb());
                    }
                    if (polygons_nb != 0) {
                        M.facets.reserve(polygons_nb);
                        for (GEO::index_t i = 0, i_end = facet_ptr.size(); i < i_end; ++i) {
                            const GEO::index_t fv_nb = facet_ptr[i];
                            if (fv_nb > 4) {
                                const GEO::index_t new_f = M.facets.create_polygon(fv_nb);
                                for (GEO::index_t lv = 0; lv < fv_nb; ++lv)
                                    M.facets.set_vertex(new_f, lv, facet_ptr[i+lv+1]);
                            }
                            i += fv_nb;
                        }
                    }

                    M.facets.connect();
                }
                else if (kw == "Polyhedra") { // == Polyhedra =========================================================
                    in.get_line();
                    in.get_fields();
                    if (in.nb_fields() != 1)
                        throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Invalid polyhedra nb format!");

                    const GEO::index_t nb_polyhedra = in.field_as_uint(0);

                    std::vector<GEO::index_t> cell_ptr; // nb_vertices, hf0, hf1, ..., hfn, nb_vertices, hf1, ...
                    GEO::index_t tets_nb = 0;
                    GEO::index_t hexes_nb = 0;
                    GEO::index_t polyhedra_nb = 0;
                    for (GEO::index_t i = 0; i < nb_polyhedra; ++i) {
                        in.get_line();
                        in.get_fields();
                        if(in.nb_fields() == 0)
                            throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Invalid cell, empty line!");

                        const GEO::index_t cf_nb = in.field_as_uint(0);
                        if (cf_nb == 4)
                            ++tets_nb;
                        else if (cf_nb == 6)
                            ++hexes_nb;
                        else if (cf_nb > 6)
                            ++polyhedra_nb;
                        else
                            throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Invalid cell vertices nb "+std::to_string(cf_nb)+"!");

                        if (in.nb_fields() != cf_nb+1)
                            throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Invalid cell, wrong number of elements!");
                        cell_ptr.push_back(cf_nb);

                        for (GEO::index_t lf = 0; lf < cf_nb; ++lf) {
                            const GEO::index_t hf = in.field_as_uint(1+lf);
                            if(const GEO::index_t f = hf/2;
                                f > M.facets.nb())
                                throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Invalid facet id in cell!");
                            cell_ptr.push_back(hf);
                        }
                    }

                    /* Create cells */
                    if (tets_nb != 0) {
                        throw std::runtime_error("Not support tets yet!");
                    }
                    if (hexes_nb != 0) {
                        GEO::index_t new_c = M.cells.create_hexes(hexes_nb);
                        for (GEO::index_t i = 0, i_end = cell_ptr.size(); i < i_end; ++i) {
                            const GEO::index_t cf_nb = cell_ptr[i];
                            if (cf_nb == 6) {
                                const auto& hf0 = cell_ptr[i+1];
                                const auto& hf1 = cell_ptr[i+2];
                                const auto& hf3 = cell_ptr[i+4];
                                const auto& hf4 = cell_ptr[i+5];
                                const auto f0 = hf0/2;
                                const auto f1 = hf1/2;
                                const auto f3 = hf3/2;
                                const auto f4 = hf4/2;
                                assert(f0 < M.facets.nb());
                                assert(f1 < M.facets.nb());
                                assert(M.facets.nb_vertices(f0) == 4); // quad
                                assert(M.facets.nb_vertices(f1) == 4); // quad
                                // ref: https://github.com/LIHPC-Computational-Geometry/ovm.io/blob/main/app/ovm.io.cpp
                                // OVM convention is
                                //       5-------6
                                //      /|      /|
                                //     / |     / |
                                //    3-------2  |
                                //    |  4----|--7
                                //    | /     | /
                                //    |/      |/
                                //    0-------1
                                //
                                // and Geogram convention is
                                //        4-------6
                                //       /|      /|
                                //      / |     / |
                                //     0-------2  |
                                //     |  5----|--7
                                //     | /     | /
                                //     |/      |/
                                //     1-------3
                                /* Find v0 (f0, f3, f4) */
                                GEO::index_t v0 = GEO::NO_VERTEX;
                                {
                                    for (GEO::index_t lv0 = 0; lv0 < 4; ++lv0) {
                                        const auto& f0v = M.facets.vertex(f0, lv0);
                                        for (GEO::index_t lv3 = 0; lv3 < 4; ++lv3) {
                                            if (const auto& f3v = M.facets.vertex(f3, lv3);
                                                f3v != f0v)
                                                continue;
                                            for (GEO::index_t lv4 = 0; lv4 < 4; ++lv4) {
                                                if (const auto& f4v = M.facets.vertex(f4, lv4);
                                                    f4v != f0v)
                                                    continue;
                                                v0 = f0v;
                                                break;
                                            }
                                            if (v0 != GEO::NO_VERTEX)
                                                break;
                                        }
                                        if (v0 != GEO::NO_VERTEX)
                                            break;
                                    }
                                    assert(v0 != GEO::NO_VERTEX);
                                }

                                /* Find v4 (f1, f3, f4) */
                                GEO::index_t v4 = GEO::NO_VERTEX;
                                {
                                    for (GEO::index_t lv1 = 0; lv1 < 4; ++lv1) {
                                        const auto& f1v = M.facets.vertex(f1, lv1);
                                        for (GEO::index_t lv3 = 0; lv3 < 4; ++lv3) {
                                            if (const auto& f3v = M.facets.vertex(f3, lv3);
                                                f3v != f1v)
                                                continue;
                                            for (GEO::index_t lv4 = 0; lv4 < 4; ++lv4) {
                                                if (const auto& f4v = M.facets.vertex(f4, lv4);
                                                    f4v != f1v)
                                                    continue;
                                                v4 = f1v;
                                                break;
                                            }
                                            if (v4 != GEO::NO_VERTEX)
                                                break;
                                        }
                                        if (v4 != GEO::NO_VERTEX)
                                            break;
                                    }
                                    assert(v4 != GEO::NO_VERTEX);
                                }

                                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                                    if (M.facets.vertex(f0, lv) != v0)
                                        continue;
                                    if (hf0%2 == 0) {
                                        M.cells.set_vertex(new_c, 0, M.facets.vertex(f0, (lv+3)%4));
                                        M.cells.set_vertex(new_c, 1, M.facets.vertex(f0, lv));
                                        M.cells.set_vertex(new_c, 2, M.facets.vertex(f0, (lv+2)%4));
                                        M.cells.set_vertex(new_c, 3, M.facets.vertex(f0, (lv+1)%4));
                                    }
                                    else { // inverse
                                        M.cells.set_vertex(new_c, 0, M.facets.vertex(f0, (lv+1)%4));
                                        M.cells.set_vertex(new_c, 1, M.facets.vertex(f0, lv));
                                        M.cells.set_vertex(new_c, 2, M.facets.vertex(f0, (lv+2)%4));
                                        M.cells.set_vertex(new_c, 3, M.facets.vertex(f0, (lv+3)%4));
                                    }
                                }

                                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                                    if (M.facets.vertex(f1, lv) != v4)
                                        continue;
                                    if (hf1%2 == 0) {
                                        M.cells.set_vertex(new_c, 4, M.facets.vertex(f1, (lv+1)%4));
                                        M.cells.set_vertex(new_c, 5, M.facets.vertex(f1, lv));
                                        M.cells.set_vertex(new_c, 6, M.facets.vertex(f1, (lv+2)%4));
                                        M.cells.set_vertex(new_c, 7, M.facets.vertex(f1, (lv+3)%4));
                                    }
                                    else { // inverse
                                        M.cells.set_vertex(new_c, 4, M.facets.vertex(f1, (lv+3)%4));
                                        M.cells.set_vertex(new_c, 5, M.facets.vertex(f1, lv));
                                        M.cells.set_vertex(new_c, 6, M.facets.vertex(f1, (lv+2)%4));
                                        M.cells.set_vertex(new_c, 7, M.facets.vertex(f1, (lv+1)%4));
                                    }
                                }
                                ++new_c;
                            }
                            i += cf_nb;
                        }
                        assert(new_c == M.cells.nb());
                    }
                    if (polyhedra_nb != 0) {
                        throw std::runtime_error("Not support polyhedra yet!");
                    }

                    M.cells.connect();
                }
                else if (kw == "Vertex_Property") {
                    for (const auto& v : M.vertices)
                        in.get_line();
                }
                else if (kw == "Edge_Property") {
                    for (const auto& e : M.edges)
                        in.get_line();
                }
                else if (kw == "HalfEdge_Property") {
                    for (const auto& e : M.edges) {
                        in.get_line();
                        in.get_line();
                    }
                }
                else if (kw == "Face_Property") {
                    for (const auto& f : M.facets)
                        in.get_line();
                }
                else if (kw == "HalfFace_Property") {
                    for (const auto& f : M.facets) {
                        in.get_line();
                        in.get_line();
                    }
                }
                else if (kw == "Polyhedron_Property") {
                    for (const auto& c : M.cells)
                        in.get_line();
                }
                else
                    throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Unknown kw `"+kw+"`!");
            }
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

        return true;
    }
}
