//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_AEDTPLT_IO_H
#define GEOLIO_AEDTPLT_IO_H

#include <geogram/mesh/mesh_io.h>

namespace geolio
{
    /**
     * I/O for ".aedtplt" files exported by Ansys HFSS through the plot mesh.
     */
    class AEDTPLT_IOHandler : public GEO::MeshIOHandler {
    public:
        bool load(
            const std::string& filename,
            GEO::Mesh& mesh,
            const GEO::MeshIOFlags& ioflags) override;

        [[deprecated("This function is not yet implemented.")]]
        bool save(
            const GEO::Mesh& mesh,
            const std::string& filename,
            const GEO::MeshIOFlags& ioflags
            ) override {
            return false;
        }
    };
}

#endif //GEOLIO_AEDTPLT_IO_H
