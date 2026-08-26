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

                if (const std::string kw = in.field(0);
                    kw == "$begin"
                    ) {
                    if (in.nb_fields() != 2)
                        throw("Line "+std::to_string(in.line_number())+" :Expect drawing name!");
                    const std::string drawing_name = in.field(1);

                    /* IsUniformMesh */
                    in.get_line();

                    /* HasCurvElem */
                    in.get_line();

                    /* BoundingBox */
                    in.get_line();

                    /* Elements */
                    in.get_line();

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