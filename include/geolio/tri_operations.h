//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/13.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOGRAM_MESH_UTILS_TRIANGLE_OPERATIONS_H
#define GEOGRAM_MESH_UTILS_TRIANGLE_OPERATIONS_H

#include <geogram/mesh/mesh.h>
#include <cassert>
#include "mesh_operations.h"

namespace geolio
{
    /**
     * @brief Split an edge of a triangle in a mesh and update the adjacency topology accordingly.
     *
     * Given triangle facet @p f and local vertex index @p lv, a new vertex @p new_v is inserted
     * on the directed edge (lv -> lv+1) at interpolation ratio @p r. The owning facet @p f is
     * replaced by two triangles that use @p new_v; if the opposite facet across that edge exists
     * (af != GEO::NO_FACET) it is also split to maintain a consistent manifold connectivity.
     *
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
     *
     * @details
     * Implementation steps:
     * - Determine the two global vertex indices v0 and v1 that define the target edge.
     * - Compute the new vertex coordinates by linear interpolation and write them to `new_v`.
     * - Create two new triangles from the original facet `f` by replacing the edge with the
     *   two new edges that connect to `new_v`, using `new_f0` (and `new_f1` for adjacent facet).
     * - If an adjacent facet exists, perform a symmetric split there and update adjacency links
     *   (facet-to-facet and facet-to-vertex relationships) so that mesh connectivity remains valid.
     * - For boundary edges, only split `f` and properly set the adjacency entries for the created facets.
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
     *
     * For facet @p f and local edge (lv -> lv+1), the function evaluates the collapse
     * that moves vertex v(lv) to `(1-r)*p(lv) + r*p((lv+1)%3)` and merges v((lv+1)%3)
     * into v(lv). The test inspects all triangles incident to the two edge endpoints
     * and rejects the collapse if any adjacent triangle would become inverted or degenerate.
     *
     * @param[in] M Target triangle mesh used only for geometric/topological queries.
     * @param[in] f Index of a triangle facet adjacent to the candidate edge.
     * @param[in] lv Local vertex index in {0,1,2} identifying the oriented edge (lv -> lv+1).
     * @return true if the local edge collapse preserves triangle orientations and manifoldness;
     *         false if any incident triangle would flip, become degenerate, or violate boundary constraints.
     *
     * @details
     * Typical checks performed by the implementation:
     * - The target edge must be interior or satisfy boundary collapse policy.
     * - For each triangle adjacent to either endpoint, the signed area after mapping the moved vertex
     *   is computed; a non-positive sign indicates an invalid collapse.
     * - Topological constraints (e.g., resulting vertex valence, duplicated edges) are also verified
     *   to avoid creating non-manifold connections.
     */
    bool is_tri_edge_collapse_valid(
        const GEO::Mesh& M,
        GEO::index_t f,
        GEO::index_t lv);

    /**
     * @brief Collapse an edge of a triangle and update local connectivity.
     *
     * Given facet @p f and local vertex index @p lv, this function collapses edge (lv -> lv+1)
     * by moving vertex v(lv) to (1-r)*p(lv) + r*p((lv+1)%3), then merging v((lv+1)%3) into v(lv).
     * Incident facets that used the collapsed edge become unused and are reported via output
     * parameters so callers can release their storage.
     *
     * @param[in,out] M The target mesh topology/geometry to update.
     * @param[in] f Index of a triangle facet incident to the edge to collapse.
     * @param[in] lv Local vertex index in {0,1,2} identifying the directed edge (lv -> lv+1).
     * @param[out] disuse_v Receives the index of the vertex that was merged away (the original v(lv+1)).
     * @param[out] disuse_f0 Receives the index of the first facet that becomes unused (typically @p f).
     * @param[out] disuse_f1 Receives the index of the opposite facet across the collapsed edge, or
     *                      GEO::NO_FACET if the edge was on the boundary.
     * @param[in] r  Interpolation ratio in [0,1] controlling new position of the surviving vertex (default 0.5).
     *
     * @details
     * Implementation outline:
     * - Check preconditions (edge not already flagged, is_tri_edge_collapse_valid() returns true).
     * - Compute the target position for the surviving vertex and assign it.
     * - Rewire all incident facets of the removed vertex to reference the surviving vertex.
     * - Update adjacency links for neighbouring facets to bypass removed facets.
     * - Mark facets `disuse_f0` and `disuse_f1` as logically removed so callers can add them to free lists.
     * - Set `disuse_v` to the removed vertex index for later reuse.
     *
     * Note: The function does not physically erase vertices/facets from mesh arrays; it only updates
     * connectivity and reports removed indices. Physical deletion or reuse is the caller's responsibility.
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
     *
     * The function inspects the interior edge shared by facet @p f and its adjacent facet across
     * local edge @p lv. It evaluates whether flipping the shared diagonal to the other diagonal of
     * the quad formed by the two triangles would produce inverted or degenerate triangles.
     *
     * @param[in] M Target triangle mesh used for geometric/topological queries.
     * @param[in] f Index of one incident facet of the interior edge to consider.
     * @param[in] lv Local edge index in {0,1,2} in facet @p f identifying the edge opposite local vertex @p lv.
     * @return true if the edge flip preserves triangle orientations and produces valid, non-degenerate geometry;
     *         false if the edge is on the boundary or the flip would create inverted/degenerate triangles.
     *
     * @details
     * Common checks include:
     * - The edge must have an adjacent facet (not a boundary edge).
     * - The resulting two triangles' signed areas must be positive.
     * - The flip should not introduce duplicate edges or non-manifold connectivity.
     */
    bool is_tri_edge_swap_valid(
        const GEO::Mesh& M,
        GEO::index_t f,
        GEO::index_t lv);

    /**
     * @brief Swap an interior edge shared by two triangles.
     *
     * For facet @p f and local edge @p lv, this operation replaces the shared diagonal of the
     * two incident triangles with the other diagonal of the local quadrilateral. The two facet
     * indices are kept unchanged, while their vertex connectivity and adjacency links are updated
     * in-place.
     *
     * @param[in,out] M Target triangle mesh whose facet connectivity and adjacency are modified.
     * @param[in] f Index of one incident facet of the edge to flip.
     * @param[in] lv Local edge index in {0,1,2} in facet @p f identifying the edge opposite local vertex @p lv.
     * @return true if the swap is performed successfully; false if the target edge is on the border
     *         or the operation is not applicable.
     *
     * @details
     * Steps performed by the implementation:
     * - Verify the edge is interior and is_tri_edge_swap_valid() returns true.
     * - Retrieve the four vertex indices that form the local quad (two from each triangle).
     * - Replace the two triangle connectivity entries to reference the new diagonal.
     * - Update adjacency pointers for the modified triangles and their neighbours so that facet
     *   adjacency remains consistent.
     * - Recompute per-facet auxiliary data if needed (e.g., normals) by the caller.
     */
    bool tri_edge_swap(
        GEO::Mesh& M,
        GEO::index_t f,
        GEO::index_t lv);
}

#endif //GEOGRAM_MESH_UTILS_TRIANGLE_OPERATIONS_H
