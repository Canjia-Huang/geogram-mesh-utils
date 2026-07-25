//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/5/16.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAMMESHUTILS_MESH_OPERATIONS_H
#define GEOGRAMMESHUTILS_MESH_OPERATIONS_H

#include <geogram/mesh/mesh.h>

namespace geolio
{
    /**
     * @brief Collect facets incident to a vertex in one-ring order.
     *
     * Starting from facet @p start_f and its local vertex slot @p start_lv, this function traverses
     * the incident facets around the target vertex and outputs ordered pairs (facet index, local vertex index).
     * The traversal is applicable to arbitrary polygonal meshes, including hybrid meshes with facets of
     * different sizes, rather than being limited to triangular meshes. For interior vertices, the sequence
     * forms a closed ring. For border vertices, the sequence is ordered from one border side to the other.
     *
     * For non-manifold vertices, only the connected component of incident facets reachable from the seed
     * facet is explored. The resulting list is therefore not guaranteed to be complete for the full set of
     * incident facets.
     *
     * @param[in] M Input mesh.
     * @param[in] start_f Seed facet index incident to the target vertex.
     * @param[in] start_lv Local vertex index of the target vertex in @p start_f.
     * @param[out] ordered_f_and_lv Output ordered one-ring list. Each element is (f, lv), where
     *                              @p f is an incident facet and @p lv is the local index of the target
     *                              vertex inside that facet. Existing contents are cleared.
     * @return true if the target vertex is on the mesh border; false if it is an interior vertex.
     */
    bool get_vertex_incident_facets(
        const GEO::Mesh& M,
        GEO::index_t start_f,
        GEO::index_t start_lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_f_and_lv);

    bool get_edge_incident_cells(
        const GEO::Mesh& M,
        GEO::index_t start_c,
        GEO::index_t start_le,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& ordered_c_le_lf);
}

#endif //GEOGRAMMESHUTILS_MESH_OPERATIONS_H
