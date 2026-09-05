//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_OFF_IO_H
#define GEOLIO_OFF_IO_H
#include <geogram/mesh/mesh_io.h>

namespace geolio
{
    /**
     * @brief IO handler for ".off" files (with or without vertex normal).
     * @see ".off" with vertex normal: https://www.graphics.rwth-aachen.de/publication/03197/
     */
    class OFF_IOHandler : public GEO::MeshIOHandler {
    public:
        bool load(
            const std::string& filename,
            GEO::Mesh& mesh,
            const GEO::MeshIOFlags& ioflags) override;

        bool save(
            const GEO::Mesh& mesh,
            const std::string& filename,
            const GEO::MeshIOFlags& ioflags) override;
    };
}

#endif //GEOLIO_OFF_IO_H
