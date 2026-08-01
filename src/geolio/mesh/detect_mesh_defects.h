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
     * @details Scans every facet, skipping non-triangular faces, and builds a
     *          sorted key from each triangle's three vertex indices. Each key
     *          is inserted into an unordered set; the first time a key is seen
     *          the triangle is kept, and any later triangle sharing the same
     *          key is flagged as a duplicate. The flagged indices are then
     *          collected into @p duplicate_triangles.
     * @param[in] mesh The input mesh to inspect.
     * @param[out] duplicate_triangles Output list that receives the indices of
     *        triangles identified as duplicates.
     * @return The number of duplicate triangles found.
     */
    GEO::index_t detect_duplicate_triangles(
        const GEO::Mesh& mesh,
        std::vector<GEO::index_t>& duplicate_triangles);

    /**
     * @brief Detects non-manifold vertices in a mesh.
     * @details Counts the facets incident to each vertex, then for every vertex
     *          runs a one-ring traversal from one incident facet via
     *          `get_vertex_incident_facets`. A vertex is classified as
     *          non-manifold when the traversal does not reach all of its
     *          incident facets (i.e. the incident faces do not form a single
     *          manifold fan), or when `pseudo_manifold` is false and the
     *          traversal ends on the mesh border.
     * @param[in] mesh The input mesh to inspect.
     * @param[out] non_manifold_vertices Output list that receives the indices
     *            of vertices classified as non-manifold.
     * @param[in] pseudo_manifold Whether to also treat pseudo-manifold
     *            vertices as defects.
     * @return The number of non-manifold vertices found.
     */
    GEO::index_t detect_non_manifold_vertices(
        const GEO::Mesh& mesh,
        std::vector<GEO::index_t>& non_manifold_vertices,
        bool pseudo_manifold = true);
}

#endif //GEOLIO_DETECT_MESH_DEFECTS_H
