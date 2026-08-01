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
     * @brief Split an edge of a triangle in a mesh and update the adjacency topology accordingly.
     * @details Given triangle facet @p f and local vertex index @p lv, a new vertex @p new_v is
     *          inserted on the directed edge (lv -> lv+1) at interpolation ratio @p r. The owning
     *          facet @p f is replaced by two triangles that use @p new_v; if the opposite facet
     *          across that edge exists it is also split to maintain a consistent manifold
     *          connectivity. The implementation writes the interpolated point to @p new_v, rewrites
     *          the facet vertex and adjacency entries of the created facets, and copies or restores
     *          the per-facet and per-corner attributes.
     * @param[in,out] M The target mesh. Vertex and facet storage must be pre-allocated and
     *                  reachable via the mesh accessors used by this function.
     * @param[in] f Index of the triangle facet to split.
     * @param[in] lv Local vertex index in {0,1,2} that identifies the edge to split (edge between
     *               local vertices lv and (lv+1)%3).
     * @param[in] new_v Index of a pre-allocated new vertex. The function sets its position to
     *                  (1-r)*p(lv) + r*p((lv+1)%3).
     * @param[in] new_f0 Index of a pre-allocated new facet that becomes one of the two facets
     *                   created from splitting facet @p f.
     * @param[in] new_f1 Index of a pre-allocated new facet that becomes one of the two facets
     *                   created from splitting the adjacent facet across the edge. Ignored when
     *                   the edge is a boundary edge (no adjacent facet).
     * @param[in] r  Interpolation ratio in [0,1] controlling the new vertex placement along the edge
     *              (default: 0.5 places the vertex at the midpoint).
     */
    void tri_edge_split(
        GEO::Mesh& M,
        GEO::index_t f,
        GEO::index_t lv,
        GEO::index_t new_v,
        GEO::index_t new_f0,
        GEO::index_t new_f1,
        double r = 0.5);

    /**
     * @brief Check whether collapsing a triangle edge preserves local orientation.
     * @details For facet @p f and local edge (lv -> lv+1), the function evaluates the collapse
     *          that moves vertex v(lv) to `(1-r)*p(lv) + r*p((lv+1)%3)` and merges v((lv+1)%3)
     *          into v(lv). It collects the one-rings of both endpoints via
     *          get_vertex_incident_facets(), rejects boundary configurations that would create a
     *          non-manifold vertex, and checks for degenerate or duplicate triangles around the
     *          collapsed edge.
     * @param[in] M Target triangle mesh used only for geometric/topological queries.
     * @param[in] f Index of a triangle facet adjacent to the candidate edge.
     * @param[in] lv Local vertex index in {0,1,2} identifying the oriented edge (lv -> lv+1).
     * @return true if the local edge collapse preserves triangle orientations and manifoldness;
     *         false if any incident triangle would flip, become degenerate, or violate boundary constraints.
     */
    bool is_tri_edge_collapse_valid(
        const GEO::Mesh& M,
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
     * @param[in,out] M The target mesh topology/geometry to update.
     * @param[in] f Index of a triangle facet incident to the edge to collapse.
     * @param[in] lv Local vertex index in {0,1,2} identifying the directed edge (lv -> lv+1).
     * @param[out] disuse_v Receives the index of the vertex that was merged away (the original v(lv+1)).
     * @param[out] disuse_f0 Receives the index of the first facet that becomes unused (typically @p f).
     * @param[out] disuse_f1 Receives the index of the opposite facet across the collapsed edge, or
     *                      GEO::NO_FACET if the edge was on the boundary.
     * @param[in] r  Interpolation ratio in [0,1] controlling new position of the surviving vertex (default 0.5).
     */
    void tri_edge_collapse(
        GEO::Mesh& M,
        GEO::index_t f,
        GEO::index_t lv,
        GEO::index_t& disuse_v,
        GEO::index_t& disuse_f0,
        GEO::index_t& disuse_f1,
        double r = 0.5);

    /**
     * @brief Check whether swapping a triangle edge is geometrically valid.
     * @details The function inspects the interior edge shared by facet @p f and its adjacent
     *          facet across local edge @p lv. It first requires the edge to have an adjacent
     *          facet, then rejects the flip if the opposite vertex of the adjacent facet already
     *          appears in any neighbour across the quad, which would create duplicate edges or
     *          non-manifold connectivity.
     * @param[in] M Target triangle mesh used for geometric/topological queries.
     * @param[in] f Index of one incident facet of the interior edge to consider.
     * @param[in] lv Local edge index in {0,1,2} in facet @p f identifying the edge opposite local vertex @p lv.
     * @return true if the edge flip preserves triangle orientations and produces valid, non-degenerate geometry;
     *         false if the edge is on the boundary or the flip would create inverted/degenerate triangles.
     */
    bool is_tri_edge_swap_valid(
        const GEO::Mesh& M,
        GEO::index_t f,
        GEO::index_t lv);

    /**
     * @brief Swap an interior edge shared by two triangles.
     * @details For facet @p f and local edge @p lv, this operation replaces the shared diagonal
     *          of the two incident triangles with the other diagonal of the local quadrilateral.
     *          The two facet indices are kept unchanged while their vertex connectivity and
     *          adjacency links are updated in place: the function rewires the four edges of the
     *          quad, updates the neighbour facet-to-facet adjacency, and copies or restores the
     *          affected facet and corner attributes.
     * @param[in,out] M Target triangle mesh whose facet connectivity and adjacency are modified.
     * @param[in] f Index of one incident facet of the edge to flip.
     * @param[in] lv Local edge index in {0,1,2} in facet @p f identifying the edge opposite local vertex @p lv.
     * @return true if the swap is performed successfully; false if the target edge is on the border
     *         or the operation is not applicable.
     */
    bool tri_edge_swap(
        GEO::Mesh& M,
        GEO::index_t f,
        GEO::index_t lv);
}

#endif //GEOLIO_TRIANGLE_OPERATIONS_H
