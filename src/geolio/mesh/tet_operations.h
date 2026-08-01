//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/3/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_TETRAHEDRON_OPERATIONS_H
#define GEOLIO_TETRAHEDRON_OPERATIONS_H

#include <geogram/mesh/mesh.h>
#include <cassert>
#include <ranges>
#include <unordered_set>
#include <stack>
#include "tet_descriptor.h"

namespace geolio
{
    /**
     * Finds the local edge index in a tetrahedron from two local endpoint vertices.
     *
     * Edge direction is ignored.
     *
     * @param[in] lv0 Local vertex index (0-3) of one endpoint
     * @param[in] lv1 Local vertex index (0-3) of the other endpoint
     * @return Local edge index (0-5) if @p lv0 and @p lv1 form a tetrahedron edge; otherwise GEO::NO_INDEX
     */
    inline GEO::index_t find_tet_edge_from_local_vertices(
        const GEO::index_t lv0,
        const GEO::index_t lv1
        ) {
        assert(lv0 < 4);
        assert(lv1 < 4);

        switch ((1<<lv0) | (1<<lv1)) {
            case TET_ENCODED_LE[0]: return 0;
            case TET_ENCODED_LE[1]: return 1;
            case TET_ENCODED_LE[2]: return 2;
            case TET_ENCODED_LE[3]: return 3;
            case TET_ENCODED_LE[4]: return 4;
            case TET_ENCODED_LE[5]: return 5;
            default:
                return GEO::NO_INDEX;
        }
    }

    /**
     * Finds the local edge index in a tetrahedron from two endpoint vertex.
     *
     * The search is performed in cell @p c using global vertex indices @p ev0 and @p ev1.
     * Edge direction is ignored.
     *
     * @param[in] M    The mesh to query
     * @param[in] c    Index of the hexahedral cell to search
     * @param[in] v0  Global vertex index of one endpoint
     * @param[in] v1  Global vertex index of the other endpoint
     * @return Local edge index (0-5) in @p c if found; otherwise GEO::NO_INDEX
     */
    inline GEO::index_t find_tet_edge(
        const GEO::Mesh& M,
        const GEO::index_t c,
        const GEO::index_t v0,
        const GEO::index_t v1
        ) {
        assert(c < M.cells.nb());
        assert(M.cells.type(c) == GEO::MeshCellType::MESH_TET);

        for (GEO::index_t lv = 0; lv < 4; ++lv) {
            if (M.cells.vertex(c, lv) == v0) {
                for (const auto& adj_lv : TET_LV_ADJACENT_LV[lv]) {
                    if (M.cells.vertex(c, adj_lv) == v1)
                        return find_tet_edge_from_local_vertices(lv, adj_lv);
                }
                break;
            }
        }
        return GEO::NO_INDEX;
    }

    /**
     * Returns the third vertex of a tetrahedron facet from two known facet vertices.
     *
     * @param[in] M  Input tetrahedral mesh.
     * @param[in] c  Cell index.
     * @param[in] lf Local facet index (0..3) in cell @p c.
     * @param[in] v0 First known vertex on facet (@p c, @p lf).
     * @param[in] v1 Second known vertex on facet (@p c, @p lf); must be different from @p v0.
     * @return The facet vertex in (@p c, @p lf) that is different from @p v0 and @p v1.
     *
     * @note Preconditions (debug-checked with assertions): @p c and @p lf are valid,
     *       and both @p v0 and @p v1 belong to facet (@p c, @p lf).
     */
    inline GEO::index_t get_tet_facet_another_vertex(
        const GEO::Mesh& M,
        const GEO::index_t c,
        const GEO::index_t lf,
        const GEO::index_t v0,
        const GEO::index_t v1
        ) {
        assert(c < M.cells.nb());
        assert(M.cells.type(c) == GEO::MeshCellType::MESH_TET);
        assert(lf < 4);
        assert(M.cells.facet_vertex(c, lf, 0) == v0 || M.cells.facet_vertex(c, lf, 1) == v0 || M.cells.facet_vertex(c, lf, 2) == v0);
        assert(M.cells.facet_vertex(c, lf, 0) == v1 || M.cells.facet_vertex(c, lf, 1) == v1 || M.cells.facet_vertex(c, lf, 2) == v1);

        return M.cells.facet_vertex(c, lf, 0)^
               M.cells.facet_vertex(c, lf, 1)^
               M.cells.facet_vertex(c, lf, 2)^
               v0^
               v1;

        GEO::index_t v2 = GEO::NO_VERTEX;
        for (GEO::index_t lv = 0; lv < 3; ++lv) {
            v2 = M.cells.facet_vertex(c, lf, lv);
            if (v2 != v0 && v2 != v1)
                break;
        }
        return v2;
    }

    /**
     * Splits one tetrahedron into four tetrahedra by inserting an interior vertex.
     *
     * The vertex index @p new_v is expected to be pre-allocated. Its position will be set
     * to the barycenter of cell @p c. The original cell @p c is updated in-place and
     * three additional tetrahedra (@p new_c0, @p new_c1, @p new_c2) are filled.
     * Adjacency between the four resulting cells and neighboring cells is updated.
     *
     * @param[in,out] M      The tetrahedral mesh to modify
     * @param[in]     c      Index of the tetrahedron to split
     * @param[in]     new_v  Index of a pre-allocated vertex used as the split vertex
     * @param[in]     new_c0 Index of the first pre-allocated tetrahedron created by the split
     * @param[in]     new_c1 Index of the second pre-allocated tetrahedron created by the split
     * @param[in]     new_c2 Index of the third pre-allocated tetrahedron created by the split
     */
    void tet_split(
        GEO::Mesh& M,
        GEO::index_t c,
        GEO::index_t new_v,
        GEO::index_t new_c0,
        GEO::index_t new_c1,
        GEO::index_t new_c2);

    /**
     * Splits a tetrahedral facet by inserting a new vertex on the facet.
     *
     * The vertex index @p new_v is expected to be pre-allocated. Its position will be set
     * to the barycenter of facet @p lf in cell @p c. The incident tetrahedral connectivity
     * is updated, and the resulting tetrahedra are written into the pre-allocated cells.
     * For a boundary facet, only @p new_c0 and @p new_c1 are used. For an interior facet,
     * @p new_c2 and @p new_c3 are also used.
     *
     * @param[in,out] M      The tetrahedral mesh to modify.
     * @param[in]     c      Index of the tetrahedron that owns the target facet.
     * @param[in]     lf     Local facet index (0-3) in cell @p c.
     * @param[in]     new_v  Index of a pre-allocated vertex used as the split vertex.
     * @param[in]     new_c0 Index of the first pre-allocated tetrahedron created by the split.
     * @param[in]     new_c1 Index of the second pre-allocated tetrahedron created by the split.
     * @param[in]     new_c2 Optional; index of the third pre-allocated tetrahedron used for interior facets.
     * @param[in]     new_c3 Optional; index of the fourth pre-allocated tetrahedron used for interior facets.
     */
    void tet_facet_split(
        GEO::Mesh& M,
        GEO::index_t c,
        GEO::index_t lf,
        GEO::index_t new_v,
        GEO::index_t new_c0,
        GEO::index_t new_c1,
        GEO::index_t new_c2 = GEO::NO_CELL,
        GEO::index_t new_c3 = GEO::NO_CELL);

    /**
     * Splits a tetrahedral edge by inserting one vertex and splitting all incident cells.
     *
     * The edge is identified by local edge index @p le in seed cell @p _c.
     * The function traverses all tetrahedra incident to that edge, places @p new_v
     * on the edge using interpolation ratio @p r, and splits each incident tetrahedron
     * into two tetrahedra while updating adjacency relations.
     *
     * The newly created tetrahedra are provided through @p new_cs, which must point to
     * an array large enough to store one cell index per tetrahedron incident to the edge.
     * Each entry in that array must reference a pre-allocated tetrahedron slot; on return
     * the function clears the entries by writing GEO::NO_CELL after they have been consumed.
     *
     * @param[in,out] M      The tetrahedral mesh to modify.
     * @param[in]     _c     Seed cell that contains the edge to split.
     * @param[in]     le     Local edge index (0-5) in cell @p _c.
     * @param[in]     new_v   Index of the pre-allocated vertex to place on the edge.
     * @param[in,out] new_cs  Array of pre-allocated tetrahedron indices, one per incident
     *                        cell. The array is consumed in order and reset to GEO::NO_CELL.
     * @param[in]     r       Interpolation ratio used to position @p new_v on the edge
     *                        (`0` at the first endpoint, `1` at the second).
     */
    void tet_edge_split(
        GEO::Mesh& M,
        GEO::index_t _c,
        GEO::index_t le,
        GEO::index_t new_v,
        GEO::index_t* new_cs,
        double r = 0.5);

    /**
     * Checks whether collapsing a tetrahedral edge at interpolation ratio @p r is valid.
     *
     * The target point is computed on the edge identified by @p le in cell @p _c.
     * The function virtually moves both edge endpoints to that target point and tests
     * all incident tetrahedra to ensure no negative signed volume is produced.
     *
     * @param[in] M   The tetrahedral mesh to query.
     * @param[in] _c  Index of a seed cell containing the target edge.
     * @param[in] le  Local edge index (0-5) in cell @p _c.
     * @param[in] r   Interpolation ratio on the edge (`0` at endpoint 0, `1` at endpoint 1).
     * @return true if all affected tetrahedra keep non-negative orientation; false otherwise.
     */
    bool is_tet_edge_collapse_valid(
        const GEO::Mesh& M,
        GEO::index_t _c,
        GEO::index_t le,
        double r = 0.5);

    /**
     * Collapses a tetrahedral edge by moving one endpoint along the edge and
     * updating the local cavity connectivity.
     *
     * The edge is identified by local edge index @p le in cell @p c.
     * Parameter @p r controls the new endpoint position by interpolation on the
     * edge segment (`0` keeps the first endpoint, `1` keeps the second endpoint).
     * Collapsed cells/vertices are reported through optional output arguments.
     *
     * @param[in,out] M         The tetrahedral mesh to modify.
     * @param[in]     _c         Index of a cell containing the target edge.
     * @param[in]     _le        Local edge index (0-5) in cell @p c.
     * @param[in]     r         Interpolation ratio for the kept vertex position on the edge.
     * @param[out]    disuse_v  Receives the removed vertex index.
     * @param[out]    disuse_cs Receives indices of cells removed by the collapse.
     */
    void tet_edge_collapse(
        GEO::Mesh& M,
        GEO::index_t _c,
        GEO::index_t _le,
        GEO::index_t& disuse_v,
        std::vector<GEO::index_t>& disuse_cs,
        double r = 0.5);

    /**
     * Checks whether a 2-3 tetrahedral swap can be applied on a facet.
     *
     * The facet is identified by local facet index @p lf in cell @p c. The function
     * requires a valid adjacent tetrahedron across that facet and verifies that the
     * three tetrahedra generated by the prospective flip would keep non-negative
     * signed volume.
     *
     * @param[in] M   The tetrahedral mesh to query.
     * @param[in] c   Index of the seed cell containing the candidate facet.
     * @param[in] lf  Local facet index (0-3) in cell @p c.
     * @return true if the 2-3 swap is geometrically valid; false otherwise.
     */
    bool is_tet_edge_swap_2_3_valid(
        const GEO::Mesh& M,
        GEO::index_t c,
        GEO::index_t lf);

    /**
     * Performs a 2-3 facet swap operation on a tetrahedral mesh.
     *
     * This operation replaces two tetrahedra that share a common facet with
     * three tetrahedra by flipping that facet. Given a seed cell @p c and its
     * local facet @p lf, this function identifies the adjacent tetrahedron
     * across the facet and updates the local connectivity accordingly.
     *
     * @param[in,out] M      The tetrahedral mesh to modify.
     * @param[in]     c      Index of the seed cell containing the target facet.
     * @param[in]     lf     Local facet index (0-3) of cell @p c.
     * @param[in,out] new_c  Index of the pre-allocated cell used to store the newly created tetrahedron.
     * @return true if the swap is performed successfully; false if the target facet is on the border or the operation cannot be applied.
     */
    bool tet_edge_swap_2_3(
        GEO::Mesh& M,
        GEO::index_t c,
        GEO::index_t lf,
        GEO::index_t new_c);

    /**
     * Performs a 3-2 edge swap operation on a tetrahedral mesh.
     *
     * This operation replaces 3 tetrahedra sharing a common edge with 2 tetrahedra
     * by removing the shared edge. Given a cell @p _c with a local edge @p _le,
     * this function identifies all incident cells (expected to be exactly 3),
     * performs the topological transformation, and reports the removed cell.
     *
     * @param[in,out] M        The tetrahedral mesh to modify.
     * @param[in]     _c       Index of a seed cell containing the target edge.
     * @param[in]     _le      Local edge index (0-5) in cell @p _c.
     * @param[out]    disuse_c Reference to receive the index of the removed cell.
     * @return true if the swap was performed successfully; false if preconditions are not met.
     */
    bool tet_edge_swap_3_2(
        GEO::Mesh& M,
        GEO::index_t _c,
        GEO::index_t _le,
        GEO::index_t& disuse_c);
}

#endif //GEOLIO_TETRAHEDRON_OPERATIONS_H
