//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_TRIANGLE_OPERATIONS_H
#define GEOLIO_TRIANGLE_OPERATIONS_H

#include <geogram/mesh/mesh.h>

namespace geolio
{
    /**
     * @brief Split a triangle edge and update mesh connectivity.
     * @details Given triangle facet @p f and local vertex index @p lv, insert a new vertex @p new_v
     *          on the directed edge (lv -> (lv+1)%3). The new vertex position is set to the point
     *          interpolated on that edge (for example the midpoint). Facet @p f is replaced by two
     *          triangles that include @p new_v. If an adjacent facet exists across that edge, it is
     *          also split so manifold connectivity is preserved. The function updates vertex
     *          coordinates, facet vertex indices, facet-to-facet adjacency, and copies/restores
     *          per-facet and per-corner attributes when @p update_attributes is true.
     * @tparam DIM Coordinate dimension (2 or 3) used to read/write vertex coordinates.
     * @param[in,out] mesh The mesh to modify. Storage for vertices and facets must be accessible.
     * @param[in] f Index of the triangle facet to split.
     * @param[in] lv Local vertex index in {0,1,2} identifying the edge between lv and (lv+1)%3.
     * @param[in] new_v Index of a pre-allocated vertex; its coordinates are set to the interpolated point.
     * @param[in] new_f0 Index of a pre-allocated facet used for one of the two new facets replacing @p f.
     * @param[in] new_f1 Index of a pre-allocated facet used to split the adjacent facet; ignored if the edge is a boundary.
     */
    template <GEO::index_t DIM>
    void tri_edge_split(
        GEO::Mesh& mesh,
        GEO::index_t f,
        GEO::index_t lv,
        GEO::index_t new_v,
        GEO::index_t new_f0,
        GEO::index_t new_f1,
        bool update_attributes = true);

    extern template void tri_edge_split<2>(
        GEO::Mesh& mesh, GEO::index_t f, GEO::index_t lv, GEO::index_t new_v, GEO::index_t new_f0, GEO::index_t new_f1, bool update_attributes);
    extern template void tri_edge_split<3>(
        GEO::Mesh& mesh, GEO::index_t f, GEO::index_t lv, GEO::index_t new_v, GEO::index_t new_f0, GEO::index_t new_f1, bool update_attributes);

    /**
     * @brief Check whether collapsing a triangle edge preserves local orientation.
     * @details For facet @p f and local edge (lv -> lv+1), the function evaluates the collapse
     *          that moves vertex v(lv) to `(1-r)*p(lv) + r*p((lv+1)%3)` and merges v((lv+1)%3)
     *          into v(lv). It collects the one-rings of both endpoints via
     *          get_vertex_incident_facets(), rejects boundary configurations that would create a
     *          non-manifold vertex, and checks for degenerate or duplicate triangles around the
     *          collapsed edge.
     * @param[in] mesh Target triangle mesh used only for geometric/topological queries.
     * @param[in] f Index of a triangle facet adjacent to the candidate edge.
     * @param[in] lv Local vertex index in {0,1,2} identifying the oriented edge (lv -> lv+1).
     * @return true if the local edge collapse preserves triangle orientations and manifoldness;
     *         false if any incident triangle would flip, become degenerate, or violate boundary constraints.
     */
    bool is_tri_edge_collapse_valid(
        const GEO::Mesh& mesh,
        GEO::index_t f,
        GEO::index_t lv);

    /**
     * @brief Collapse an edge of a triangle and update local connectivity.
     * @details Given facet @p f and local vertex index @p lv, this function collapses edge
     *          (lv -> lv+1) by moving vertex v(lv) to (1-r)*p(lv) + r*p((lv+1)%3), then merging
     *          v((lv+1)%3) into v(lv). It interpolates the vertex attributes, rewires every facet
     *          incident to the removed vertex to reference the surviving vertex, and relinks the
     *          facet-to-facet adjacency of the neighbouring facets across the collapsed cavity.
     *          Incident facets that used the collapsed edge become unused and are reported through
     *          output parameters; physical deletion is left to the caller.
     * @tparam DIM Coordinate dimension (2 or 3) used to read the interpolated vertex position.
     * @param[in,out] mesh The target mesh topology/geometry to update.
     * @param[in] f Index of a triangle facet incident to the edge to collapse.
     * @param[in] lv Local vertex index in {0,1,2} identifying the directed edge (lv -> lv+1).
     * @param[out] disuse_v Receives the index of the vertex that was merged away (the original v(lv+1)).
     * @param[out] disuse_f0 Receives the index of the first facet that becomes unused (typically @p f).
     * @param[out] disuse_f1 Receives the index of the opposite facet across the collapsed edge, or
     *                      GEO::NO_FACET if the edge was on the boundary.
     */
    template <GEO::index_t DIM>
    void tri_edge_collapse(
        GEO::Mesh& mesh,
        GEO::index_t f,
        GEO::index_t lv,
        GEO::index_t& disuse_v,
        GEO::index_t& disuse_f0,
        GEO::index_t& disuse_f1);

    extern template void tri_edge_collapse<2>(
        GEO::Mesh& mesh, GEO::index_t f, GEO::index_t lv, GEO::index_t& disuse_v, GEO::index_t& disuse_f0,
        GEO::index_t& disuse_f1);
    extern template void tri_edge_collapse<3>(
        GEO::Mesh& mesh, GEO::index_t f, GEO::index_t lv, GEO::index_t& disuse_v, GEO::index_t& disuse_f0,
        GEO::index_t& disuse_f1);

    /**
     * @brief Check whether swapping a triangle edge is geometrically valid.
     * @details The function inspects the interior edge shared by facet @p f and its adjacent
     *          facet across local edge @p lv. It first requires the edge to have an adjacent
     *          facet, then rejects the flip if the opposite vertex of the adjacent facet already
     *          appears in any neighbour across the quad, which would create duplicate edges or
     *          non-manifold connectivity.
     * @param[in] mesh Target triangle mesh used for geometric/topological queries.
     * @param[in] f Index of one incident facet of the interior edge to consider.
     * @param[in] lv Local edge index in {0,1,2} in facet @p f identifying the edge opposite local vertex @p lv.
     * @return true if the edge flip preserves triangle orientations and produces valid, non-degenerate geometry;
     *         false if the edge is on the boundary or the flip would create inverted/degenerate triangles.
     */
    bool is_tri_edge_swap_valid(
        const GEO::Mesh& mesh,
        GEO::index_t f,
        GEO::index_t lv);

    /**
     * @brief Swap (flip) an interior edge shared by two triangles.
     * @details For facet @p f and local edge @p lv, replace the shared diagonal of the two incident
     *          triangles by the other diagonal of the local quadrilateral. The facet indices remain
     *          unchanged; the function updates each facet's vertex connectivity and the surrounding
     *          facet-to-facet adjacency. Per-facet and per-corner attributes are copied/restored when
     *          @p update_attributes is true.
     * @param[in,out] mesh Mesh whose connectivity and adjacency are modified.
     * @param[in] f Index of one incident facet of the interior edge to flip.
     * @param[in] lv Local vertex index in {0,1,2} in facet @p f identifying the edge opposite local vertex @p lv.
     * @return true if the flip was applied; false if the edge is on the boundary or the operation is invalid.
     */
    bool tri_edge_swap(
        GEO::Mesh& mesh,
        GEO::index_t f,
        GEO::index_t lv,
        bool update_attributes = true);
}

#endif //GEOLIO_TRIANGLE_OPERATIONS_H
