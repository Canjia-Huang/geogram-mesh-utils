//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/5/16.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_OPERATIONS_H
#define GEOLIO_MESH_OPERATIONS_H

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

    /**
     * Collects cells incident to a vertex from a seed cell.
     *
     * Starting from (@p start_c, @p start_lv), this function traverses neighboring
     * cells that share the same global vertex and outputs pairs (cell index, local
     * vertex index in that cell). The traversal currently supports tetrahedra and
     * hexahedra.
     *
     * For non-manifold configurations, only the component reachable from the seed
     * cell through adjacency links is collected.
     *
     * @param[in] M Input mesh.
     * @param[in] start_c Seed cell index incident to the target vertex.
     * @param[in] start_lv Local vertex index of the target vertex in @p start_c.
     * @param[out] c_and_lv Output incident list. Each element is (c, lv), where
     *                      @p c is an incident cell and @p lv is the local index
     *                      of the target vertex in that cell. Existing contents are cleared.
     * @return true if any incident side of the vertex reaches the border; false otherwise.
     */
    bool get_vertex_incident_cells(
        const GEO::Mesh& M,
        GEO::index_t start_c,
        GEO::index_t start_lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& c_and_lv);

    /**
     * Collects edge-incident cells in ring/chain order from a seed local edge.
     *
     * The edge is identified by local edge index @p start_le in cell @p start_c.
     * The output stores tuples (c, le, lf), where @p c is an incident cell, @p le
     * is the corresponding local edge index in that cell, and @p lf is the local
     * facet used by the traversal to move from @p c to the next cell in the ordered
     * sequence.
     *
     * For interior edges, the sequence forms a closed loop. For border edges, the
     * sequence is ordered from one border side to the other.
     *
     * @param[in] M Input mesh.
     * @param[in] start_c Seed cell containing the target edge.
     * @param[in] start_le Local edge index in @p start_c.
     * @param[out] ordered_c_le_lf Output ordered incident list of (c, le, lf).
     *                              Existing contents are cleared.
     * @return true if the target edge is on the border; false if it is interior.
     */
    bool get_edge_incident_cells(
        const GEO::Mesh& M,
        GEO::index_t start_c,
        GEO::index_t start_le,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& ordered_c_le_lf);

    /**
     * Collects edge-incident cells in ring/chain order from a facet edge seed.
     *
     * The target edge is derived from two consecutive facet vertices:
     * `facet_vertex(start_c, start_lf, start_lv)` and
     * `facet_vertex(start_c, start_lf, (start_lv+1)%N)`, where `N` is the number
     * of vertices of the facet (3 for tetrahedra, 4 for hexahedra). The function
     * then delegates to the local-edge overload.
     *
     * @param[in] M Input mesh.
     * @param[in] start_c Seed cell index.
     * @param[in] start_lf Local facet index in @p start_c.
     * @param[in] start_lv Local vertex slot inside @p start_lf; together with the
     *                     next facet vertex defines the seed edge.
     * @param[out] ordered_c_le_lf Output ordered incident list of (c, le, lf).
     *                              Existing contents are cleared.
     * @return true if the derived edge is on the border; false if it is interior.
     */
    bool get_edge_incident_cells(
        const GEO::Mesh& M,
        GEO::index_t start_c,
        GEO::index_t start_lf,
        GEO::index_t start_lv,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& ordered_c_le_lf);
}

#endif //GEOLIO_MESH_OPERATIONS_H
