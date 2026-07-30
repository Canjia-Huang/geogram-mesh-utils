//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_DETECT_MESH_DEFECTS_H
#define GEOLIO_DETECT_MESH_DEFECTS_H

#include <geogram/mesh/mesh.h>

namespace geolio
{
    /**
     * @brief Detects duplicate triangles in a mesh.
     * @param[in] mesh The input mesh to inspect.
     * @param[out] duplicate_triangles Output list that receives the indices of
     *        triangles identified as duplicates.
     * @return The number of duplicate triangles found.
     *
     * This function scans the mesh for repeated triangle faces and appends
     * their indices to @p duplicate_triangles.
     */
    GEO::index_t detect_duplicate_triangles(
        const GEO::Mesh& mesh,
        std::vector<GEO::index_t>& duplicate_triangles);
}

#endif //GEOLIO_DETECT_MESH_DEFECTS_H
