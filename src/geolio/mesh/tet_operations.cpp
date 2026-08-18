//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/6/25.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "tet_operations.h"
#include <array>
#include <cassert>
#include <tuple>
#include <utility>
#include <vector>
#include "mesh_operations.h"

namespace geolio
{
    void tet_split(
        GEO::Mesh& M,
        const GEO::index_t c,
        const GEO::index_t new_v,
        const GEO::index_t new_c0,
        const GEO::index_t new_c1,
        const GEO::index_t new_c2
        ) {
        assert(c < M.cells.nb());
        assert(new_v < M.cells.nb());
        assert(new_c0 < M.cells.nb());
        assert(new_c1 < M.cells.nb());
        assert(new_c2 < M.cells.nb());

        const GEO::index_t v0 = M.cells.vertex(c, 0);
        const GEO::index_t v1 = M.cells.vertex(c, 1);
        const GEO::index_t v2 = M.cells.vertex(c, 2);
        const GEO::index_t v3 = M.cells.vertex(c, 3);
        const GEO::index_t nc0 = M.cells.adjacent(c, 0);
        const GEO::index_t nc1 = M.cells.adjacent(c, 1);
        const GEO::index_t nc2 = M.cells.adjacent(c, 2);
        // const GEO::index_t nc3 = M.cells.adjacent(c, 3);

        /* Create new vertex */
        M.vertices.point(new_v) = 0.25 * (
            M.cells.point(c, 0) + M.cells.point(c, 1) + M.cells.point(c, 2) + M.cells.point(c, 3));

        /* Set vertices */
        M.cells.set_vertex(new_c0, 0, new_v);
        M.cells.set_vertex(new_c0, 1, v1);
        M.cells.set_vertex(new_c0, 2, v2);
        M.cells.set_vertex(new_c0, 3, v3);
        M.cells.set_vertex(new_c1, 0, v0);
        M.cells.set_vertex(new_c1, 1, new_v);
        M.cells.set_vertex(new_c1, 2, v2);
        M.cells.set_vertex(new_c1, 3, v3);
        M.cells.set_vertex(new_c2, 0, v0);
        M.cells.set_vertex(new_c2, 1, v1);
        M.cells.set_vertex(new_c2, 2, new_v);
        M.cells.set_vertex(new_c2, 3, v3);
        M.cells.set_vertex(c, 3, new_v);

        /* Set adjacency */
        M.cells.set_adjacent(new_c0, 0, nc0);
        M.cells.set_adjacent(new_c0, 1, new_c1);
        M.cells.set_adjacent(new_c0, 2, new_c2);
        M.cells.set_adjacent(new_c0, 3, c);
        M.cells.set_adjacent(new_c1, 0, new_c0);
        M.cells.set_adjacent(new_c1, 1, nc1);
        M.cells.set_adjacent(new_c1, 2, new_c2);
        M.cells.set_adjacent(new_c1, 3, c);
        M.cells.set_adjacent(new_c2, 0, new_c0);
        M.cells.set_adjacent(new_c2, 1, new_c1);
        M.cells.set_adjacent(new_c2, 2, nc2);
        M.cells.set_adjacent(new_c2, 3, c);
        M.cells.set_adjacent(c, 0, new_c0);
        M.cells.set_adjacent(c, 1, new_c1);
        M.cells.set_adjacent(c, 2, new_c2);
        if (nc0 != GEO::NO_CELL) {
            assert(M.cells.find_tet_facet(nc0, v1, v2, v3) != GEO::NO_INDEX);
            M.cells.set_adjacent(nc0, M.cells.find_tet_facet(nc0, v1, v2, v3), new_c0);
        }
        if (nc1 != GEO::NO_CELL) {
            assert(M.cells.find_tet_facet(nc1, v0, v3, v2) != GEO::NO_INDEX);
            M.cells.set_adjacent(nc1, M.cells.find_tet_facet(nc1, v0, v3, v2), new_c1);
        }
        if (nc2 != GEO::NO_CELL) {
            assert(M.cells.find_tet_facet(nc2, v0, v1, v3) != GEO::NO_INDEX);
            M.cells.set_adjacent(nc2, M.cells.find_tet_facet(nc2, v0, v1, v3), new_c2);
        }
    }

    void tet_facet_split(
        GEO::Mesh& M,
        const GEO::index_t c,
        const GEO::index_t lf,
        const GEO::index_t new_v,
        const GEO::index_t new_c0,
        const GEO::index_t new_c1,
        const GEO::index_t new_c2,
        const GEO::index_t new_c3
        ) {
        assert(c < M.cells.nb());
        assert(lf < 4);
        assert(new_v < M.vertices.nb());
        assert(new_c0 < M.cells.nb());
        assert(new_c1 < M.cells.nb());

        /* Set new vertex */
        M.vertices.point(new_v) = (M.vertices.point(M.cells.facet_vertex(c, lf, 0)) +
                                    M.vertices.point(M.cells.facet_vertex(c, lf, 1)) +
                                    M.vertices.point(M.cells.facet_vertex(c, lf, 2))) / 3;


        const GEO::index_t v0 = M.cells.vertex(c, TET_LF_INCIDENT_LV[lf][0]);
        const GEO::index_t v1 = M.cells.vertex(c, TET_LF_INCIDENT_LV[lf][1]);
        const GEO::index_t v2 = M.cells.vertex(c, TET_LF_INCIDENT_LV[lf][2]);
        {
            const GEO::index_t lv0 = TET_LF_INCIDENT_LV[lf][0];
            const GEO::index_t lv1 = TET_LF_INCIDENT_LV[lf][1];
            const GEO::index_t lv2 = TET_LF_INCIDENT_LV[lf][2];
            const GEO::index_t lv3 = (0^1^2^3)^lv0^lv1^lv2;
            assert(lv3 < 4 && lv3 != lv0 && lv3 != lv1 && lv3 != lv2);

            const GEO::index_t v3 = M.cells.vertex(c, lv3);

            const GEO::index_t nc0 = M.cells.adjacent(c, lv0);
            const GEO::index_t nc1 = M.cells.adjacent(c, lv1);

            /*
             *            lv1 (v1)
             *            /  |   \
             *          /    |     \
             *        /    new_v     \
             *      /        /\        \
             *    /   c   /      \ new_c0\
             *  /      /   new_c1   \      \
             * lv0 (v0) ------------- lv2 (v2)
             */

            /* Set cells vertices */
            M.cells.set_vertex(c, lv2, new_v);
            M.cells.set_vertex(new_c0, lv0, new_v);
            M.cells.set_vertex(new_c0, lv1, v1);
            M.cells.set_vertex(new_c0, lv2, v2);
            M.cells.set_vertex(new_c0, lv3, v3);
            M.cells.set_vertex(new_c1, lv0, v0);
            M.cells.set_vertex(new_c1, lv1, new_v);
            M.cells.set_vertex(new_c1, lv2, v2);
            M.cells.set_vertex(new_c1, lv3, v3);

            /* Set cells adjacent */
            M.cells.set_adjacent(c, lv0, new_c0);
            M.cells.set_adjacent(c, lv1, new_c1);
            M.cells.set_adjacent(new_c0, lv0, nc0);
            M.cells.set_adjacent(new_c0, lv1, new_c1);
            M.cells.set_adjacent(new_c0, lv2, c);
            M.cells.set_adjacent(new_c0, lv3, new_c2);
            M.cells.set_adjacent(new_c1, lv0, new_c0);
            M.cells.set_adjacent(new_c1, lv1, nc1);
            M.cells.set_adjacent(new_c1, lv2, c);
            M.cells.set_adjacent(new_c1, lv3, new_c3);
            if (nc0 != GEO::NO_CELL) {
                const GEO::index_t nlf = M.cells.find_tet_facet(
                    nc0,
                    M.cells.facet_vertex(new_c0, lv0, 2),
                    M.cells.facet_vertex(new_c0, lv0, 1),
                    M.cells.facet_vertex(new_c0, lv0, 0));
                assert(nlf != GEO::NO_INDEX);
                M.cells.set_adjacent(nc0, nlf, new_c0);
            }
            if (nc1 != GEO::NO_CELL) {
                const GEO::index_t nlf = M.cells.find_tet_facet(
                    nc1,
                    M.cells.facet_vertex(new_c1, lv1, 2),
                    M.cells.facet_vertex(new_c1, lv1, 1),
                    M.cells.facet_vertex(new_c1, lv1, 0));
                assert(nlf != GEO::NO_INDEX);
                M.cells.set_adjacent(nc1, nlf, new_c1);
            }
        }

        if (const auto& ac = M.cells.adjacent(c, lf);
            ac != GEO::NO_CELL
            ) {
            assert(new_c2 < M.cells.nb());
            assert(new_c3 < M.cells.nb());

            const GEO::index_t lv0 = M.cells.find_tet_vertex(ac, v0);
            const GEO::index_t lv1 = M.cells.find_tet_vertex(ac, v2);
            const GEO::index_t lv2 = M.cells.find_tet_vertex(ac, v1);
            const GEO::index_t lv3 = (0^1^2^3)^lv0^lv1^lv2;
            assert(lv3 < 4 && lv3 != lv0 && lv3 != lv1 && lv3 != lv2);

            const GEO::index_t v3 = M.cells.vertex(ac, lv3);

            const GEO::index_t nc0 = M.cells.adjacent(ac, lv0);
            const GEO::index_t nc2 = M.cells.adjacent(ac, lv2);

            /*
             *            lv2 (v1)
             *            /  |   \
             *          /    |     \
             *        /    new_v     \
             *      /        /\        \
             *    /  ac   /      \ new_c2\
             *  /      /   new_c3   \      \
             * lv0 (v0) ------------- lv1 (v2)
             */

            /* Set cells vertices */
            M.cells.set_vertex(ac, lv1, new_v);
            M.cells.set_vertex(new_c2, lv0, new_v);
            M.cells.set_vertex(new_c2, lv1, v2);
            M.cells.set_vertex(new_c2, lv2, v1);
            M.cells.set_vertex(new_c2, lv3, v3);
            M.cells.set_vertex(new_c3, lv0, v0);
            M.cells.set_vertex(new_c3, lv1, v2);
            M.cells.set_vertex(new_c3, lv2, new_v);
            M.cells.set_vertex(new_c3, lv3, v3);

            /* Set cells adjacent */
            M.cells.set_adjacent(ac, lv0, new_c2);
            M.cells.set_adjacent(ac, lv2, new_c3);
            M.cells.set_adjacent(new_c2, lv0, nc0);
            M.cells.set_adjacent(new_c2, lv1, ac);
            M.cells.set_adjacent(new_c2, lv2, new_c3);
            M.cells.set_adjacent(new_c2, lv3, new_c0);
            M.cells.set_adjacent(new_c3, lv0, new_c2);
            M.cells.set_adjacent(new_c3, lv1, ac);
            M.cells.set_adjacent(new_c3, lv2, nc2);
            M.cells.set_adjacent(new_c3, lv3, new_c1);
            if (nc0 != GEO::NO_CELL) {
                const GEO::index_t nlf = M.cells.find_tet_facet(
                    nc0,
                    M.cells.facet_vertex(new_c2, lv0, 2),
                    M.cells.facet_vertex(new_c2, lv0, 1),
                    M.cells.facet_vertex(new_c2, lv0, 0));
                assert(nlf != GEO::NO_INDEX);
                M.cells.set_adjacent(nc0, nlf, new_c2);
            }
            if (nc2 != GEO::NO_CELL) {
                const GEO::index_t nlf = M.cells.find_tet_facet(
                    nc2,
                    M.cells.facet_vertex(new_c3, lv2, 2),
                    M.cells.facet_vertex(new_c3, lv2, 1),
                    M.cells.facet_vertex(new_c3, lv2, 0));
                assert(nlf != GEO::NO_INDEX);
                M.cells.set_adjacent(nc2, nlf, new_c3);
            }
        }
    }

    void tet_edge_split(
        GEO::Mesh& M,
        const std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>>& ordered_c_le_lf,
        const GEO::index_t new_v,
        const std::vector<GEO::index_t>& new_cs,
        const double r
        ) {
        assert(new_v < M.vertices.nb());
        assert(new_cs.size() == ordered_c_le_lf.size());

        /* Find all adjacent cells */
        const GEO::index_t INCIDENT_CELLS_NB = ordered_c_le_lf.size();

        const auto start_c = get<0>(ordered_c_le_lf[0]);
        const auto start_le = get<1>(ordered_c_le_lf[0]);
        const GEO::index_t ev0 = M.cells.edge_vertex(start_c, start_le, 0);
        const GEO::index_t ev1 = M.cells.edge_vertex(start_c, start_le, 1);

        /* Set new vertex */
        M.vertices.point(new_v) = (1-r)*M.vertices.point(ev0) + r*M.vertices.point(ev1);

        for (GEO::index_t i = 0; i < INCIDENT_CELLS_NB; ++i) {
            const auto& [c, _, lf0] = ordered_c_le_lf[i];
            const auto& new_c = new_cs[i];

            const GEO::index_t lv0 = M.cells.find_tet_vertex(c, ev0);
            const GEO::index_t lv1 = M.cells.find_tet_vertex(c, ev1);
            const GEO::index_t lv2 = TET_LF_INCIDENT_LV[lf0][0]^TET_LF_INCIDENT_LV[lf0][1]^TET_LF_INCIDENT_LV[lf0][2]^lv0^lv1;
            assert(lv2 < 4 && lv2 != lv0 && lv2 != lv1);
            const GEO::index_t lv3 = 0^1^2^3^lv0^lv1^lv2;
            assert(lv3 < 4 && lv3 != lv0 && lv3 != lv1 && lv3 != lv2);

            const GEO::index_t v2 = M.cells.vertex(c, lv2);
            const GEO::index_t v3 = M.cells.vertex(c, lv3);

            // const GEO::index_t nc0 = M.cells.adjacent(c, lv0);
            const GEO::index_t nc1 = M.cells.adjacent(c, lv1);

            /* Set cell vertices */
            M.cells.set_vertex(c, lv0, new_v);
            M.cells.set_vertex(new_c, lv0, ev0);
            M.cells.set_vertex(new_c, lv1, new_v);
            M.cells.set_vertex(new_c, lv2, v2);
            M.cells.set_vertex(new_c, lv3, v3);

            /* Set cell adjacent */
            M.cells.set_adjacent(c, lv1, new_c);
            M.cells.set_adjacent(new_c, lv0, c);
            M.cells.set_adjacent(new_c, lv1, nc1);
            if (M.cells.adjacent(c, lv2) != GEO::NO_CELL)
                M.cells.set_adjacent(new_c, lv2, new_cs[(i+INCIDENT_CELLS_NB-1)%INCIDENT_CELLS_NB]);
            if (M.cells.adjacent(c, lv3) != GEO::NO_CELL)
                M.cells.set_adjacent(new_c, lv3, new_cs[(i+1)%INCIDENT_CELLS_NB]);
            if (nc1 != GEO::NO_CELL) {
                const GEO::index_t nlf = M.cells.find_tet_facet(
                    nc1,
                    M.cells.facet_vertex(new_c, lv1, 2),
                    M.cells.facet_vertex(new_c, lv1, 1),
                    M.cells.facet_vertex(new_c, lv1, 0));
                assert(nlf != GEO::NO_INDEX);
                M.cells.set_adjacent(nc1, nlf, new_c);
            }
        }
    }

    bool is_tet_edge_collapse_valid(
        const GEO::Mesh& M,
        const GEO::index_t _c,
        const GEO::index_t le,
        const double r
        ) {
        assert(_c < M.cells.nb());
        assert(M.cells.type(_c) == GEO::MeshCellType::MESH_TET);
        assert(le < 6);
        assert(r >= 0 && r <= 1);

        const auto& ev0 = M.cells.edge_vertex(_c, le, 0);
        const auto& ev1 = M.cells.edge_vertex(_c, le, 1);

        /* Move vertex */
        const auto& ep0 = M.vertices.point(ev0);
        const auto& ep1 = M.vertices.point(ev1);
        const auto target_ep = (1-r)*ep0 + r*ep1;

        /* Find all adjacent tets */
        std::vector<std::pair<GEO::index_t, GEO::index_t>> ev0_incident_c_and_lv;
        get_vertex_incident_cells(M, _c, TET_LE_INCIDENT_LV[le][0], ev0_incident_c_and_lv);
        for (const auto& [c, lv] : ev0_incident_c_and_lv) {
            std::array<GEO::vec3, 4> cell_points = {
                M.cells.point(c, 0), M.cells.point(c, 1), M.cells.point(c, 2), M.cells.point(c, 3)
            };
            cell_points[lv] = target_ep;

            if (GEO::Geom::tetra_signed_volume(cell_points[0], cell_points[1], cell_points[2], cell_points[3]) < 0)
                return false;
        }

        std::vector<std::pair<GEO::index_t, GEO::index_t>> ev1_incident_c_and_lv;
        get_vertex_incident_cells(M, _c, TET_LE_INCIDENT_LV[le][1], ev1_incident_c_and_lv);
        for (const auto& [c, lv] : ev1_incident_c_and_lv) {
            std::array<GEO::vec3, 4> cell_points = {
                M.cells.point(c, 0), M.cells.point(c, 1), M.cells.point(c, 2), M.cells.point(c, 3)
            };
            cell_points[lv] = target_ep;

            if (GEO::Geom::tetra_signed_volume(cell_points[0], cell_points[1], cell_points[2], cell_points[3]) < 0)
                return false;
        }

        return true;
    }

    void tet_edge_collapse(
        GEO::Mesh& M,
        const GEO::index_t _c,
        const GEO::index_t _le,
        GEO::index_t& disuse_v,
        std::vector<GEO::index_t>& disuse_cs,
        const double r
        ) {
        assert(_c < M.cells.nb());
        assert(M.cells.type(_c) == GEO::MeshCellType::MESH_TET);
        assert(_le < 6);
        assert(r >= 0 && r <= 1);

        const auto& ev0 = M.cells.edge_vertex(_c, _le, 0);
        const auto& ev1 = M.cells.edge_vertex(_c, _le, 1);

        /* Move vertex */
        auto& ep0 = M.vertices.point(ev0);
        const auto& ep1 = M.vertices.point(ev1);
        ep0 = (1-r)*ep0 + r*ep1;
        disuse_v = ev1;

        /* Find all adjacent tets */
        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> ordered_c_le_lf;
        get_edge_incident_cells(M, _c, _le, ordered_c_le_lf);

        std::vector<std::pair<GEO::index_t, GEO::index_t>> ev1_incident_c_and_lv;
        get_vertex_incident_cells(M, _c, TET_LE_INCIDENT_LV[_le][1], ev1_incident_c_and_lv);

        /* Collapse */
        for (const auto& c: ordered_c_le_lf | std::views::keys) {
            const auto lf0 = M.cells.find_tet_vertex(c, ev0);
            const auto lf1 = M.cells.find_tet_vertex(c, ev1);
            const auto nc0 = M.cells.adjacent(c, lf0);
            const auto nc1 = M.cells.adjacent(c, lf1);

            if (nc0 != GEO::NO_CELL) {
                /* Set adjacent */
                const auto nlf = M.cells.find_tet_facet(
                    nc0,
                    M.cells.facet_vertex(c, lf0, 2),
                    M.cells.facet_vertex(c, lf0, 1),
                    M.cells.facet_vertex(c, lf0, 0));
                assert(nlf != GEO::NO_INDEX);
                M.cells.set_adjacent(nc0, nlf, nc1);
            }
            if (nc1 != GEO::NO_CELL) {
                /* Set adjacent */
                const auto nlf = M.cells.find_tet_facet(
                    nc1,
                    M.cells.facet_vertex(c, lf1, 2),
                    M.cells.facet_vertex(c, lf1, 1),
                    M.cells.facet_vertex(c, lf1, 0));
                assert(nlf != GEO::NO_INDEX);
                M.cells.set_adjacent(nc1, nlf, nc0);
            }

            disuse_cs.push_back(c);
        }

        /* Update vertex of other cells */
        for (const auto& [c, lv] :ev1_incident_c_and_lv)
            M.cells.set_vertex(c, lv, ev0);
    }

    bool is_tet_edge_swap_2_3_valid(
        const GEO::Mesh& M,
        const GEO::index_t c,
        const GEO::index_t lf
        ) {
        assert(c < M.cells.nb());
        assert(M.cells.type(c) == GEO::MeshCellType::MESH_TET);
        assert(lf < 4);

        const GEO::index_t nc = M.cells.adjacent(c, lf);
        if (nc == GEO::NO_CELL)
            return false;

        const GEO::index_t v = M.cells.vertex(c, lf);
        const GEO::index_t v0 = M.cells.facet_vertex(c, lf, 0);
        const GEO::index_t v1 = M.cells.facet_vertex(c, lf, 1);
        const GEO::index_t v2 = M.cells.facet_vertex(c, lf, 2);

        const GEO::index_t nlf = M.cells.find_tet_facet(nc, v2, v1, v0);
        assert(nlf != GEO::NO_INDEX);
        const GEO::index_t nv = M.cells.vertex(nc, nlf);

        const auto& p = M.vertices.point(v);
        const auto& p0 = M.vertices.point(v0);
        const auto& p1 = M.vertices.point(v1);
        const auto& p2 = M.vertices.point(v2);
        const auto& np = M.vertices.point(nv);

        if (GEO::Geom::tetra_signed_volume(p, np, p1, p0) < 0 ||
            GEO::Geom::tetra_signed_volume(p, np, p2, p1) < 0 ||
            GEO::Geom::tetra_signed_volume(p, np, p0, p2) < 0)
            return false;
        return true;
    }

    bool tet_edge_swap_2_3(
        GEO::Mesh& M,
        const GEO::index_t c,
        const GEO::index_t lf,
        const GEO::index_t new_c
        ) {
        assert(c < M.cells.nb());
        assert(M.cells.type(c) == GEO::MeshCellType::MESH_TET);
        assert(lf < 4);

        const GEO::index_t nc = M.cells.adjacent(c, lf);
        if (nc == GEO::NO_CELL)
            return false;

        assert(new_c < M.cells.nb());

        const GEO::index_t v = M.cells.vertex(c, lf);
        const GEO::index_t v0 = M.cells.facet_vertex(c, lf, 0);
        const GEO::index_t v1 = M.cells.facet_vertex(c, lf, 1);
        const GEO::index_t v2 = M.cells.facet_vertex(c, lf, 2);
        const GEO::index_t lv0 = TET_LF_INCIDENT_LV[lf][0];
        const GEO::index_t lv1 = TET_LF_INCIDENT_LV[lf][1];
        const GEO::index_t lv2 = TET_LF_INCIDENT_LV[lf][2];
        const GEO::index_t nc0 = M.cells.adjacent(c, lv0);
        const GEO::index_t nc1 = M.cells.adjacent(c, lv1);
        const GEO::index_t nc2 = M.cells.adjacent(c, lv2);

        const GEO::index_t nlf = M.cells.find_tet_facet(nc, v2, v1, v0);
        assert(nlf != GEO::NO_INDEX);
        GEO::index_t nlv0{GEO::NO_INDEX}, nlv1{GEO::NO_INDEX}, nlv2{GEO::NO_INDEX};
        for (GEO::index_t i = 0; i < 3; ++i) {
            if (M.cells.facet_vertex(nc, nlf, i) == v0) { // cell_vertex(nc, nlv0) == cell_vertex(c, lv0)
                nlv0 = TET_LF_INCIDENT_LV[nlf][i];
                nlv1 = TET_LF_INCIDENT_LV[nlf][(i+1)%3];
                nlv2 = TET_LF_INCIDENT_LV[nlf][(i+2)%3];
                break;
            }
        }
        assert(nlv0 != GEO::NO_INDEX && nlv1 != GEO::NO_INDEX && nlv2 != GEO::NO_INDEX);
        const GEO::index_t nv = M.cells.vertex(nc, nlf);
        const GEO::index_t nv0 = M.cells.vertex(nc, nlv0);
        assert(nv0 == v0);
        const GEO::index_t nv1 = M.cells.vertex(nc, nlv1);
        assert(nv1 == v2);
        const GEO::index_t nv2 = M.cells.vertex(nc, nlv2);
        assert(nv2 == v1);
        const GEO::index_t nc_nc0 = M.cells.adjacent(nc, nlv0);
        const GEO::index_t nc_nc1 = M.cells.adjacent(nc, nlv1);
        const GEO::index_t nc_nc2 = M.cells.adjacent(nc, nlv2);

        /* Set cell vertices */
        M.cells.set_vertex(c, 0, v);
        M.cells.set_vertex(c, 1, nv);
        M.cells.set_vertex(c, 2, v1);
        M.cells.set_vertex(c, 3, v0);
        M.cells.set_vertex(nc, 0, v);
        M.cells.set_vertex(nc, 1, nv);
        M.cells.set_vertex(nc, 2, v2);
        M.cells.set_vertex(nc, 3, v1);
        M.cells.set_vertex(new_c, 0, v);
        M.cells.set_vertex(new_c, 1, nv);
        M.cells.set_vertex(new_c, 2, v0);
        M.cells.set_vertex(new_c, 3, v2);

        /* Set adjacency */
        M.cells.set_adjacent(c, 0, nc_nc1);
        M.cells.set_adjacent(c, 1, nc2);
        M.cells.set_adjacent(c, 2, new_c);
        M.cells.set_adjacent(c, 3, nc);
        M.cells.set_adjacent(nc, 0, nc_nc0);
        M.cells.set_adjacent(nc, 1, nc0);
        M.cells.set_adjacent(nc, 2, c);
        M.cells.set_adjacent(nc, 3, new_c);
        M.cells.set_adjacent(new_c, 0, nc_nc2);
        M.cells.set_adjacent(new_c, 1, nc1);
        M.cells.set_adjacent(new_c, 2, nc);
        M.cells.set_adjacent(new_c, 3, c);
        if (nc0 != GEO::NO_CELL) {
            assert(M.cells.find_tet_facet(nc0, v, v1, v2) != GEO::NO_INDEX);
            M.cells.set_adjacent(nc0, M.cells.find_tet_facet(nc0, v, v1, v2), nc);
        }
        if (nc1 != GEO::NO_CELL) {
            assert(M.cells.find_tet_facet(nc1, v, v2, v0) != GEO::NO_INDEX);
            M.cells.set_adjacent(nc1, M.cells.find_tet_facet(nc1, v, v2, v0), new_c);
        }
        if (nc2 != GEO::NO_CELL) {
            assert(M.cells.find_tet_facet(nc2, v, v0, v1) != GEO::NO_INDEX);
            M.cells.set_adjacent(nc2, M.cells.find_tet_facet(nc2, v, v0, v1), c);
        }
        if (nc_nc0 != GEO::NO_CELL) {
            assert(M.cells.find_tet_facet(nc_nc0, nv, nv1, nv2) != GEO::NO_INDEX);
            M.cells.set_adjacent(nc_nc0, M.cells.find_tet_facet(nc_nc0, nv, nv1, nv2), nc);
        }
        if (nc_nc1 != GEO::NO_CELL) {
            assert(M.cells.find_tet_facet(nc_nc1, nv, nv2, nv0) != GEO::NO_INDEX);
            M.cells.set_adjacent(nc_nc1, M.cells.find_tet_facet(nc_nc1, nv, nv2, nv0), c);
        }
        if (nc_nc2 != GEO::NO_CELL) {
            assert(M.cells.find_tet_facet(nc_nc2, nv, nv0, nv1) != GEO::NO_INDEX);
            M.cells.set_adjacent(nc_nc2, M.cells.find_tet_facet(nc_nc2, nv, nv0, nv1), new_c);
        }

        return true;
    }

    bool tet_edge_swap_3_2(
        GEO::Mesh& M,
        const GEO::index_t _c,
        const GEO::index_t _le,
        GEO::index_t& disuse_c
        ) {
        assert(_c < M.cells.nb());
        assert(M.cells.type(_c) == GEO::MeshCellType::MESH_TET);
        assert(_le < 6);

        std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> ordered_c_le_lf;
        if(const bool is_on_border = get_edge_incident_cells(M, _c, _le, ordered_c_le_lf);
            is_on_border ||
            ordered_c_le_lf.size() != 3)
            return false;

        const GEO::index_t v0 = M.cells.edge_vertex(_c, _le, 0);
        const GEO::index_t v1 = M.cells.edge_vertex(_c, _le, 1);

        const GEO::index_t c0 = get<0>(ordered_c_le_lf[0]);
        const GEO::index_t c1 = get<0>(ordered_c_le_lf[1]);
        const GEO::index_t c2 = get<0>(ordered_c_le_lf[2]);
        disuse_c = c2;

        const GEO::index_t v2 = get_tet_facet_another_vertex(M, c0, get<2>(ordered_c_le_lf[0]), v0, v1);
        const GEO::index_t v4 = get_tet_facet_another_vertex(M, c1, get<2>(ordered_c_le_lf[1]), v0, v1);

        const GEO::index_t c0_lv0 = M.cells.find_tet_vertex(c0, v0);
        const GEO::index_t c0_lv1 = M.cells.find_tet_vertex(c0, v1);
        const GEO::index_t c0_lv2 = M.cells.find_tet_vertex(c0, v2);
        assert(c0_lv0 != GEO::NO_INDEX && c0_lv1 != GEO::NO_INDEX && c0_lv2 != GEO::NO_INDEX);
        const GEO::index_t c0_lv3 = 0^1^2^3^c0_lv0^c0_lv1^c0_lv2;
        assert(c0_lv3 < 4 && c0_lv3 != c0_lv0 && c0_lv3 != c0_lv1 && c0_lv3 != c0_lv2);

        const GEO::index_t v3 = M.cells.vertex(c0, c0_lv3);

        const GEO::index_t c1_lv0 = M.cells.find_tet_vertex(c1, v0);
        const GEO::index_t c1_lv1 = M.cells.find_tet_vertex(c1, v1);
        const GEO::index_t c1_lv4 = M.cells.find_tet_vertex(c1, v4);
        assert(c1_lv0 != GEO::NO_INDEX && c1_lv1 != GEO::NO_INDEX && c1_lv4 != GEO::NO_INDEX);
        const GEO::index_t c1_lv2 = 0^1^2^3^c1_lv0^c1_lv1^c1_lv4;
        assert(c1_lv2 < 4 && c1_lv2 != c1_lv0 && c1_lv2 != c1_lv1 && c1_lv2 != c1_lv4);

        // const GEO::index_t nc00 = M.cells.adjacent(c0, c0_lv0);
        const GEO::index_t nc01 = M.cells.adjacent(c0, c0_lv1);
        const GEO::index_t nc10 = M.cells.adjacent(c1, c1_lv0);
        // const GEO::index_t nc11 = M.cells.adjacent(c1, c1_lv1);
        const GEO::index_t nc20 = M.cells.adjacent(c2, M.cells.find_tet_vertex(c2, v0));
        const GEO::index_t nc21 = M.cells.adjacent(c2, M.cells.find_tet_vertex(c2, v1));

        /* Set vertices */
        M.cells.set_vertex(c0, c0_lv0, v4);
        M.cells.set_vertex(c1, c1_lv1, v3);

        /* Set adjacent */
        // M.cells.set_adjacent(c0, c0_lv0, nc00);
        M.cells.set_adjacent(c0, c0_lv1, c1);
        M.cells.set_adjacent(c0, c0_lv2, nc20);
        M.cells.set_adjacent(c0, c0_lv3, nc10);
        M.cells.set_adjacent(c1, c1_lv0, c0);
        // M.cells.set_adjacent(c1, c1_lv1, nc11);
        M.cells.set_adjacent(c1, c1_lv2, nc21);
        M.cells.set_adjacent(c1, c1_lv4, nc01);
        if (nc01 != GEO::NO_CELL) {
            const GEO::index_t nlf = M.cells.find_tet_facet(nc01, v0, v2, v3);
            assert(nlf != GEO::NO_INDEX);
            M.cells.set_adjacent(nc01, nlf, c1);
        }
        if (nc10 != GEO::NO_CELL) {
            const GEO::index_t nlf = M.cells.find_tet_facet(nc10, v1, v2, v4);
            assert(nlf != GEO::NO_INDEX);
            M.cells.set_adjacent(nc10, nlf, c0);
        }
        if (nc20 != GEO::NO_CELL) {
            const GEO::index_t nlf = M.cells.find_tet_facet(nc20, v1, v4, v3);
            assert(nlf != GEO::NO_INDEX);
            M.cells.set_adjacent(nc20, nlf, c0);
        }
        if (nc21 != GEO::NO_CELL) {
            const GEO::index_t nlf = M.cells.find_tet_facet(nc21, v0, v3, v4);
            assert(nlf != GEO::NO_INDEX);
            M.cells.set_adjacent(nc21, nlf, c1);
        }

        return true;
    }
}
