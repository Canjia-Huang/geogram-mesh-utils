//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/29.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_HEXEX_IO_H
#define GEOLIO_HEXEX_IO_H

#include <geogram/mesh/mesh_io.h>

namespace geolio
{
    class HEXEX_IOHandler : public GEO::MeshIOHandler {
    public:
        bool load(
            const std::string& filename,
            GEO::Mesh& M,
            const GEO::MeshIOFlags& ioflags) override;

        [[deprecated("This function is not yet implemented.")]]
        bool save(
            const GEO::Mesh& M,
            const std::string& filename,
            const GEO::MeshIOFlags& ioflags
            ) override {
            return false;
        }
    };
}

#endif //GEOLIO_HEXEX_IO_H
