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

    /**
     * @brief Detects non-manifold vertices in a mesh.
     * @param[in] mesh The input mesh to inspect.
     * @param[out] non_manifold_vertices Output list that receives the indices
     *            of vertices classified as non-manifold.
     * @param[in] pseudo_manifold Whether to also treat pseudo-manifold
     *            vertices as defects.
     * @return The number of non-manifold vertices found.
     *
     * This function scans mesh connectivity and records each vertex whose
     * incident faces do not form a single manifold fan.
     */
    GEO::index_t detect_non_manifold_vertices(
        const GEO::Mesh& mesh,
        std::vector<GEO::index_t>& non_manifold_vertices,
        bool pseudo_manifold = true);
}

#endif //GEOLIO_DETECT_MESH_DEFECTS_H
