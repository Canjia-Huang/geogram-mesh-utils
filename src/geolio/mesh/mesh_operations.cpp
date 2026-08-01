//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/6/25.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "mesh_operations.h"
#include <cassert>
#include <stack>
#include <tuple>
#include <unordered_set>
#include <utility>
#include <vector>
#include <geogram/mesh/mesh_io.h>
#include "hex_operations.h"
#include "tet_operations.h"

namespace geolio
{
    /**
     * @brief Collect facets incident to a vertex in one-ring order.
     * @details Starting from the seed facet @p start_f and its local vertex slot @p start_lv,
     *          the function walks facet-to-facet adjacency links around the target vertex,
     *          recording ordered (facet, local vertex) pairs. For interior vertices the walk
     *          forms a closed ring; for border vertices it stops at the border and then walks
     *          in the opposite direction to order the list from one border side to the other.
     *          The traversal handles arbitrary polygonal (including hybrid) meshes. For
     *          non-manifold vertices, only the connected component reachable from the seed
     *          facet is explored.
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
        const GEO::index_t start_f,
        const GEO::index_t start_lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& ordered_f_and_lv
        ) {
        assert(start_f < M.facets.nb());
        assert(start_lv < M.facets.nb_vertices(start_f));

        const GEO::index_t v = M.facets.vertex(start_f, start_lv);
        bool is_on_border = false;

        std::vector<std::pair<GEO::index_t, GEO::index_t>> next_ordered_f_and_lv;
        std::vector<std::pair<GEO::index_t, GEO::index_t>> prev_ordered_f_and_lv;
        {
            GEO::index_t f = start_f;
            GEO::index_t lv = start_lv;
            do {
                next_ordered_f_and_lv.emplace_back(f, lv);

                const GEO::index_t next_f = M.facets.adjacent(f, lv);
                if (next_f == GEO::NO_FACET) { // is not 2-manifold vertex
                    is_on_border = true;
                    break;
                }
                f = next_f;
                lv = M.facets.find_vertex(f, v);
                assert(lv != GEO::NO_INDEX);
            } while (f != start_f);
        }

        if (is_on_border) { // inverse travel
            GEO::index_t f = start_f;
            GEO::index_t lv = (start_lv+M.facets.nb_vertices(f)-1)%M.facets.nb_vertices(f);

            for (;;) {
                const GEO::index_t next_f = M.facets.adjacent(f, lv);
                if (next_f == GEO::NO_FACET)
                    break;
                f = next_f;
                lv = M.facets.find_vertex(f, v);
                prev_ordered_f_and_lv.emplace_back(f, lv);
                lv = (lv+M.facets.nb_vertices(f)-1)%M.facets.nb_vertices(f);
            }
        }

        /* Output */
        ordered_f_and_lv.clear();
        ordered_f_and_lv.reserve(next_ordered_f_and_lv.size() + prev_ordered_f_and_lv.size());
        for (GEO::index_t i = 0, i_end = prev_ordered_f_and_lv.size(); i < i_end; ++i)
            ordered_f_and_lv.push_back(prev_ordered_f_and_lv[i_end-i-1]);
        for (const auto& f_lv : next_ordered_f_and_lv)
            ordered_f_and_lv.push_back(f_lv);

        return is_on_border;
    }

    /**
     * @brief Collect cells incident to a vertex from a seed cell.
     * @details Starting from (@p start_c, @p start_lv), the function performs a
     *          depth-first search over cells sharing the target global vertex,
     *          using a stack and a processed-cell set. For each visited cell it
     *          outputs (cell index, local vertex index) and pushes the neighbors
     *          across every local facet that also contains the vertex. The traversal
     *          currently supports tetrahedra and hexahedra. For non-manifold
     *          configurations, only the component reachable from the seed cell is
     *          collected.
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
        const GEO::index_t start_c,
        const GEO::index_t start_lv,
        std::vector<std::pair<GEO::index_t, GEO::index_t>>& c_and_lv
        ) {
        assert(start_c < M.cells.nb());
        assert([&]() {
            switch (M.cells.type(start_c)) {
                case GEO::MeshCellType::MESH_TET:
                    if (start_lv >= 4)
                        return false;
                    break;
                case GEO::MeshCellType::MESH_HEX:
                    if (start_lv >= 8)
                        return false;
                    break;
                case GEO::MeshCellType::MESH_PRISM:
                    return false; // TODO: support
                    break;
                case GEO::MeshCellType::MESH_PYRAMID:
                    return false; // TODO: support
                    break;
                default: break;
            }
            return true;
        }());

        c_and_lv.clear();

        const auto v = M.cells.vertex(start_c, start_lv);

        std::unordered_set<GEO::index_t> processed_cells;
        bool is_on_border = false;

        std::stack<std::pair<GEO::index_t, GEO::index_t>> stack;
        stack.emplace(start_c, start_lv);
        while (!stack.empty()) {
            const auto [c, lv] = stack.top();
            stack.pop();

            if (!processed_cells.insert(c).second)
                continue;
            c_and_lv.emplace_back(c, lv);

            if (const auto& CELL_TYPE = M.cells.type(c);
                CELL_TYPE == GEO::MeshCellType::MESH_TET
                ) {
                for (const auto& lf : TET_LV_INCIDENT_LF[lv]) {
                    if (const auto nc = M.cells.adjacent(c, lf);
                        nc != GEO::NO_CELL
                        ) {
                        const auto nlv = M.cells.find_tet_vertex(nc, v);
                        assert(nlv != GEO::NO_INDEX);
                        stack.emplace(nc, nlv);
                    }
                    else
                        is_on_border = true;
                }
            }
            else {
                assert(CELL_TYPE == GEO::MeshCellType::MESH_HEX);

                for (const auto& lf : HEX_LV_INCIDENT_LF[lv]) {
                    if (const auto nc = M.cells.adjacent(c, lf);
                        nc != GEO::NO_CELL
                        ) {
                        const auto nlv = find_hex_vertex(M, nc, v);
                        assert(nlv != GEO::NO_INDEX);
                        stack.emplace(nc, nlv);
                    }
                    else
                        is_on_border = true;
                }
            }
        }

        return is_on_border;
    }

    /**
     * @brief Collect edge-incident cells in ring/chain order from a seed local edge.
     * @details The edge is identified by local edge index @p start_le in cell @p start_c.
     *          The function walks cell-to-cell adjacency links across the local facets
     *          adjacent to the edge, recording ordered (c, le, lf) tuples, where @p lf is
     *          the facet used to move to the next cell. For interior edges the walk forms
     *          a closed loop; for border edges it additionally walks in the opposite
     *          direction to order the sequence from one border side to the other. Only
     *          tetrahedral and hexahedral cells are currently supported.
     * @param[in] M Input mesh.
     * @param[in] start_c Seed cell containing the target edge.
     * @param[in] start_le Local edge index in @p start_c.
     * @param[out] ordered_c_le_lf Output ordered incident list of (c, le, lf).
     *                              Existing contents are cleared.
     * @return true if the target edge is on the border; false if it is interior.
     */
    bool get_edge_incident_cells(
        const GEO::Mesh& M,
        const GEO::index_t start_c,
        const GEO::index_t start_le,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& ordered_c_le_lf
        ) {
        assert(start_c < M.cells.nb());
        assert([&]() {
            switch (M.cells.type(start_c)) {
                case GEO::MeshCellType::MESH_TET:
                    if (start_le >= 6)
                        return false;
                    break;
                case GEO::MeshCellType::MESH_HEX:
                    if (start_le >= 12)
                        return false;
                    break;
                case GEO::MeshCellType::MESH_PRISM:
                    return false; // TODO: support
                    break;
                case GEO::MeshCellType::MESH_PYRAMID:
                    return false; // TODO: support
                    break;
                default: break;
            }
            return true;
        }());

        const auto ev0 = M.cells.edge_vertex(start_c, start_le, 0);
        const auto ev1 = M.cells.edge_vertex(start_c, start_le, 1);
        bool is_on_border = false;

        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> next_ordered_c_le_lf;
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> prev_ordered_c_le_lf;
        {
            GEO::index_t c = start_c;
            GEO::index_t le = start_le;
            GEO::index_t lf = M.cells.edge_adjacent_facet(start_c, start_le, 0);
            for (;;) {
                next_ordered_c_le_lf.emplace_back(c, le, lf);

                const GEO::index_t nc = M.cells.adjacent(c, lf);
                if (nc == GEO::NO_CELL) {
                    is_on_border = true;
                    break;
                }
                if (nc == start_c) // a loop
                    break;

                /* Get next lf */
                if (M.cells.type(nc) == GEO::MeshCellType::MESH_TET)
                    le = find_tet_edge(M, nc, ev0, ev1);
                else if (M.cells.type(nc) == GEO::MeshCellType::MESH_HEX)
                    le = find_hex_edge(M, nc, ev0, ev1);
                else
                    assert(0); // TODO: support MESH_PRISM and MESH_PYRAMID
                assert(le != GEO::NO_INDEX);
                lf = M.cells.edge_adjacent_facet(nc, le, 0);
                if (M.cells.adjacent(nc, lf) == c)
                    lf = M.cells.edge_adjacent_facet(nc, le, 1);
                assert(M.cells.adjacent(nc, lf) != c);
                c = nc;
            }
        }

        if (is_on_border) {
            GEO::index_t c = start_c;
            GEO::index_t lf = M.cells.edge_adjacent_facet(start_c, start_le, 1);
            for (;;) {
                const GEO::index_t nc = M.cells.adjacent(c, lf);
                if (nc == GEO::NO_CELL)
                    break;

                /* Get next lf */
                GEO::index_t le;
                if (M.cells.type(nc) == GEO::MeshCellType::MESH_TET)
                    le = find_tet_edge(M, nc, ev0, ev1);
                else if (M.cells.type(nc) == GEO::MeshCellType::MESH_HEX)
                    le = find_hex_edge(M, nc, ev0, ev1);
                else
                    assert(0); // TODO: support MESH_PRISM and MESH_PYRAMID
                assert(le != GEO::NO_INDEX);
                lf = M.cells.edge_adjacent_facet(nc, le, 0);
                GEO::index_t lf1 = M.cells.edge_adjacent_facet(nc, le, 1);
                if (M.cells.adjacent(nc, lf) == c)
                    std::swap(lf, lf1);
                assert(M.cells.adjacent(nc, lf) != c && M.cells.adjacent(nc, lf1) == c);
                c = nc;

                prev_ordered_c_le_lf.emplace_back(c, le, lf1);
            }
        }

        /* Output */
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>().swap(ordered_c_le_lf);
        ordered_c_le_lf.reserve(next_ordered_c_le_lf.size() + prev_ordered_c_le_lf.size());
        for (GEO::index_t i = 0, i_end = prev_ordered_c_le_lf.size(); i < i_end; ++i)
            ordered_c_le_lf.push_back(prev_ordered_c_le_lf[i_end-i-1]);
        for (const auto& c_lf : next_ordered_c_le_lf)
            ordered_c_le_lf.push_back(c_lf);

        return is_on_border;
    }

    /**
     * @brief Collect edge-incident cells in ring/chain order from a facet edge seed.
     * @details The target edge is derived from two consecutive facet vertices:
     *          `facet_vertex(start_c, start_lf, start_lv)` and
     *          `facet_vertex(start_c, start_lf, (start_lv+1)%N)`, where `N` is the
     *          number of vertices of the facet (3 for tetrahedra, 4 for hexahedra).
     *          The function then delegates to the local-edge overload.
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
        const GEO::index_t start_c,
        const GEO::index_t start_lf,
        const GEO::index_t start_lv,
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& ordered_c_le_lf
        ) {
        assert(start_c < M.cells.nb());

        if (M.cells.type(start_c) == GEO::MeshCellType::MESH_TET) {
            assert(start_lf < 4);
            assert(start_lv < 3);

            const auto ev0 = M.cells.facet_vertex(start_c, start_lf, start_lv);
            const auto ev1 = M.cells.facet_vertex(start_c, start_lf, (start_lv+1)%3);

            for (const auto& start_le : TET_LF_INCIDENT_LE[start_lf]) {
                const auto cev0 = M.cells.edge_vertex(start_c, start_le, 0);
                const auto cev1 = M.cells.edge_vertex(start_c, start_le, 1);
                if ((cev0 == ev0 && cev1 == ev1) ||
                    (cev0 == ev1 && cev1 == ev0))
                    return get_edge_incident_cells(M, start_c, start_le, ordered_c_le_lf);
            }
            assert(0);
        }
        else {
            assert(M.cells.type(start_c) == GEO::MeshCellType::MESH_HEX);
            assert(start_lf < 6);
            assert(start_lv < 4);

            const auto ev0 = M.cells.facet_vertex(start_c, start_lf, start_lv);
            const auto ev1 = M.cells.facet_vertex(start_c, start_lf, (start_lv+1)%4);

            for (const auto& start_le : HEX_LF_INCIDENT_LE[start_lf]) {
                const auto cev0 = M.cells.edge_vertex(start_c, start_le, 0);
                const auto cev1 = M.cells.edge_vertex(start_c, start_le, 1);
                if ((cev0 == ev0 && cev1 == ev1) ||
                    (cev0 == ev1 && cev1 == ev0))
                    return get_edge_incident_cells(M, start_c, start_le, ordered_c_le_lf);
            }
            assert(0);
        }
        return false;
    }
}
