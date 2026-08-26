//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "aedtplt_io.h"
#include <geogram/mesh/mesh_io.h>
#include <geogram/basic/line_stream.h>
#include "geolio/common/log.h"
#include "line_stream.h"

namespace geolio
{
    bool AEDTPLT_IOHandler::load(
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
            while (!in.eof()) {
                in.get_line();
                in.get_fields();

                if (in.nb_fields() == 0)
                    continue;

                if (const std::string begin_kw = in.field(0);
                    begin_kw == "$begin"
                    ) {
                    if (in.nb_fields() != 2)
                        throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Expect drawing name!");
                    const std::string drawing_name = in.field(1);

                    /* IsUniformMesh */
                    {
                        in.get_line();
                        if (const std::string kw = in.consume_until("=");
                            kw != "IsUniformMesh")
                            throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Expect `IsUniformMesh`!");
                    }

                    /* HasCurvElem */
                    {
                        in.get_line();
                        if (const std::string kw = in.consume_until("=");
                            kw != "HasCurvElem")
                            throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Expect `HasCurvElem`!");
                    }

                    /* BoundingBox */
                    {
                        in.get_line();
                        if (const std::string kw = in.consume_until("(");
                            kw != "BoundingBox")
                            throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Expect `BoundingBox`!");
                    }

                    /* Elements */
                    GEO::index_t nodes_nb = 0, elements_nb = 0;
                    std::vector<GEO::index_t> triangles; // 3*cells_nb
                    std::vector<GEO::index_t> tetrahedra; // 4*cells_nb
                    {
                        in.get_line();
                        if (const std::string kw = in.consume_until("(");
                            kw != "Elements")
                            throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Expect `Elements`!");
                        in.get_fields(",");

                        nodes_nb = in.field_as_uint(0);
                        elements_nb = in.field_as_uint(1);

                        GEO::index_t pos = 2;
                        for (GEO::index_t i = 0; i < elements_nb; ++i) {
                            pos += 4;
                            if (const GEO::index_t nb = in.field_as_uint(pos++);
                                nb == 6
                                ) { // 2-order triangle
                                /* Convert to linear triangle */
                                triangles.push_back(in.field_as_uint(pos));
                                triangles.push_back(in.field_as_uint(pos+2));
                                triangles.push_back(in.field_as_uint(pos+5));
                                pos += 6;
                            }
                            else if (nb == 10) { // 2-order tetrahedron
                                /* Convert to linear tetrahedron */
                                tetrahedra.push_back(in.field_as_uint(pos));
                                tetrahedra.push_back(in.field_as_uint(pos+2));
                                tetrahedra.push_back(in.field_as_uint(pos+5));
                                tetrahedra.push_back(in.field_as_uint(pos+9));
                                pos += 10;
                            }
                            else
                                throw std::runtime_error("Line "+std::to_string(in.line_number())+" :Unknown element type!");
                        }

                        LOG::DEBUG("load {}, {} cells", triangles.size()/3, tetrahedra.size()/4);
                    }


                    /* Nodes */
                    in.get_line();

                    /* End */
                    in.get_line();
                    in.get_fields();
                    if (in.nb_fields() != 2)
                        throw("Line "+std::to_string(in.line_number())+" :Expect `$end drawing_name`!");
                    if (const std::string kw = in.field(0);
                        kw != "$end")
                        throw("Line "+std::to_string(in.line_number())+" :Expect `$end`!");
                    if (in.field(1) != drawing_name)
                        throw("Line "+std::to_string(in.line_number())+" The names of begin and end are different!!");
                }
            }
        }
        catch(const std::string& what) {
            LOG::ERROR("{}", what);
            return false;
        } catch(const std::exception& ex) {
            LOG::ERROR("{}", ex.what());
            return false;
        } catch(...) {
            LOG::ERROR("Caught exception!");
            return false;
        }

        return true;
    }
}