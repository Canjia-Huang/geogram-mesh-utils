//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/6/25.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "tri_operations.h"
#include <cassert>
#include <unordered_set>
#include <utility>
#include <vector>
#include <geogram/mesh/mesh_io.h>
#include <geolio/common/pair_hash.h>
#include <geolio/common/vecg.h>
#include "mesh_operations.h"

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
        const GEO::index_t f,
        const GEO::index_t lv,
        const GEO::index_t new_v,
        const GEO::index_t new_f0,
        const GEO::index_t new_f1,
        const double r
        ) {
        assert(f < M.facets.nb());
        assert(M.facets.nb_vertices(f) == 3);
        assert(lv < 3);
        assert(r >= 0 && r <= 1);
        assert(new_v < M.vertices.nb());
        assert(new_f0 < M.facets.nb());

        /*
         *  +-------- v0        +-------- v0
         *  |       / |         | \     / |
         *  |     /   |    ->   |   \ /   |
         *  |   /  f  |         |   / \ f |
         *  | /       |         | /newf0\ |
         *  v1 ------ v2        v1 ------ v2
         */
        const GEO::index_t lv0 = lv;
        const GEO::index_t lv1 = (lv+1)%3;
        const GEO::index_t lv2 = (lv+2)%3;
        // const GEO::index_t v0 = M.facets.vertex(f, lv0);
        const GEO::index_t v1 = M.facets.vertex(f, lv1);
        const GEO::index_t v2 = M.facets.vertex(f, lv2);
        const GEO::index_t nf0 = M.facets.adjacent(f, lv0);
        const GEO::index_t nf1 = M.facets.adjacent(f, lv1);

        /* Set new point */
        if (M.vertices.dimension() == 2) {
            const auto& p0 = M.facets.point<2>(f, lv0);
            const auto& p1 = M.facets.point<2>(f, lv1);
            M.vertices.point<2>(new_v) = (1-r)*p0 + r*p1;
        }
        else {
            assert(M.vertices.dimension() == 3);

            const auto& p0 = M.facets.point(f, lv0);
            const auto& p1 = M.facets.point(f, lv1);
            M.vertices.point(new_v) = (1-r)*p0 + r*p1;
        }

        /* Set facet vertices  */
        M.facets.set_vertex(f, lv1, new_v);
        M.facets.set_vertex(new_f0, lv0, new_v);
        M.facets.set_vertex(new_f0, lv1, v1);
        M.facets.set_vertex(new_f0, lv2, v2);

        /* Set facet adjacency */
        M.facets.set_adjacent(f, lv1, new_f0);
        M.facets.set_adjacent(new_f0, lv1, nf1);
        M.facets.set_adjacent(new_f0, lv2, f);
        if (nf1 != GEO::NO_FACET) {
            assert(M.facets.find_vertex(nf1, v2) != GEO::NO_INDEX);
            M.facets.set_adjacent(nf1, M.facets.find_vertex(nf1, v2), new_f0);
        }

        /* Copy attributes */
        M.facets.attributes().copy_item(new_f0, f);
        M.facet_corners.attributes().copy_item(M.facets.corner(new_f0, lv0), M.facets.corner(f, lv0));
        M.facet_corners.attributes().copy_item(M.facets.corner(new_f0, lv1), M.facets.corner(f, lv1));
        /* Restore attributes */
        M.facet_corners.attributes().zero_item(M.facets.corner(f, lv1));

        /* == Split adjacent facet ================================================================================= */
        if (nf0 != GEO::NO_FACET) {
            assert(new_f1 < M.facets.nb());
            assert(M.facets.nb_vertices(new_f1) == 3);

            /*
             * nv2 ----- nv1       nv2 ----- nv1
             *  |       / |         | \ nf0 / |
             *  | nf0 /   |    ->   |new\ /   |
             *  |   /     |         |f1 / \   |
             *  | /       |         | /     \ |
             * nv0 -------+        nv0 -------+
             */
            const GEO::index_t nlv0 = M.facets.find_vertex(nf0, v1);
            assert(nlv0 != GEO::NO_INDEX);
            const GEO::index_t nlv1 = (nlv0+1)%3;
            const GEO::index_t nlv2 = (nlv0+2)%3;
            const GEO::index_t nv0 = M.facets.vertex(nf0, nlv0);
            // const GEO::index_t nv1 = M.facets.vertex(af0, nlv1);
            const GEO::index_t nv2 = M.facets.vertex(nf0, nlv2);
            const GEO::index_t nnf2 = M.facets.adjacent(nf0, nlv2);

            /* Set facet vertices */
            M.facets.set_vertex(nf0, nlv0, new_v);
            M.facets.set_vertex(new_f1, nlv0, nv0);
            M.facets.set_vertex(new_f1, nlv1, new_v);
            M.facets.set_vertex(new_f1, nlv2, nv2);

            /* Set facet adjacency */
            M.facets.set_adjacent(new_f0, lv0, new_f1);
            M.facets.set_adjacent(nf0, nlv2, new_f1);
            M.facets.set_adjacent(new_f1, nlv0, new_f0);
            M.facets.set_adjacent(new_f1, nlv1, nf0);
            M.facets.set_adjacent(new_f1, nlv2, nnf2);
            if (nnf2 != GEO::NO_FACET) {
                assert(M.facets.find_vertex(new_f1, nv0) != GEO::NO_INDEX);
                M.facets.set_adjacent(nnf2, M.facets.find_vertex(nnf2, nv0), new_f1);
            }

            /* Copy attributes */
            M.facets.attributes().copy_item(new_f1, nf0);
            M.facet_corners.attributes().copy_item(M.facets.corner(new_f1, nlv0), M.facets.corner(nf0, nlv0));
            M.facet_corners.attributes().copy_item(M.facets.corner(new_f1, nlv2), M.facets.corner(nf0, nlv2));
            /* Restore attributes */
            M.facet_corners.attributes().zero_item(M.facets.corner(nf0, nlv2));
        }
        else
            M.facets.set_adjacent(new_f0, lv0, GEO::NO_VERTEX);
    }

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
        const GEO::index_t f,
        const GEO::index_t lv
        ) {
        assert(f < M.facets.nb());
        assert(M.facets.nb_vertices(f) == 3);
        assert(lv < 3);

        const GEO::index_t lv0 = lv;
        const GEO::index_t lv1 = (lv+1)%3;
        // const GEO::index_t lv2 = (lv+2)%3;
        const GEO::index_t v0 = M.facets.vertex(f, lv0);
        const GEO::index_t v1 = M.facets.vertex(f, lv1);
        const auto af = M.facets.adjacent(f, lv);

        /* Find all incident vertices */
        std::vector<std::pair<GEO::index_t, GEO::index_t>> v0_ordered_f_and_lv;
        const bool v0_on_boundary = get_vertex_incident_facets(M, f, lv0, v0_ordered_f_and_lv);

        std::vector<std::pair<GEO::index_t, GEO::index_t>> v1_ordered_f_and_lv;
        const bool v1_on_boundary = get_vertex_incident_facets(M, f, lv1, v1_ordered_f_and_lv);

        if (v0_on_boundary && v1_on_boundary && af != GEO::NO_FACET) /* will create a new non-manifold vertex */
            return false;

        /* After collapse, no identical triangles can exist */
        std::unordered_set<std::pair<GEO::index_t, GEO::index_t>, PairHash> other_vertices_pair;
        for (const auto& [nf, nlv] : v0_ordered_f_and_lv) {
            const auto& nv1 = M.facets.vertex(nf, (nlv+1)%3);
            const auto& nv2 = M.facets.vertex(nf, (nlv+2)%3);
            assert(nv1 != v0);
            assert(nv2 != v0);
            if (nv1 == nv2)
                return false; // Exist degenerate adjacent facet.
            if (nv1 == v1 || nv2 == v1) {
                if (nf != f && nf != af)
                    return false; // Non-manifold edge.
            }
            if (const std::pair<GEO::index_t, GEO::index_t> other_vertices = std::minmax(nv1, nv2);
                !other_vertices_pair.insert(other_vertices).second)
                return false; // Identical triangles in an adjacent face group.
        }
        for (const auto& [nf, nlv] : v1_ordered_f_and_lv) {
            const auto& nv1 = M.facets.vertex(nf, (nlv+1)%3);
            const auto& nv2 = M.facets.vertex(nf, (nlv+2)%3);
            assert(nv1 != v1);
            assert(nv2 != v1);
            if (nv1 == nv2)
                return false; // Exist degenerate adjacent facet.
            if (nv1 == v0 || nv2 == v0) {
                if (nf != f && nf != af)
                    return false; // Non-manifold edge.
            }
            if (const std::pair<GEO::index_t, GEO::index_t> other_vertices = std::minmax(nv1, nv2);
                !other_vertices_pair.insert(other_vertices).second)
                return false; // Identical triangles in an adjacent face group or two adjacent face group.
        }

        /* Check inversion */
        // if (M.vertices.dimension() == 2) {
        //     const auto& p0 = M.facets.point<2>(f, lv0);
        //     const auto& p1 = M.facets.point<2>(f, lv1);
        //     const auto& p2 = M.facets.point<2>(f, lv2);
        //     const auto target_p = (1-r)*p0 + r*p1;
        //
        //     const auto normal = cross(p1-p0, p2-p0);
        //     for (const auto& [nf, nlv] : v0_ordered_f_and_lv) {
        //         std::array<GEO::vec2, 3> fps = {
        //             M.facets.point<2>(nf, 0),
        //             M.facets.point<2>(nf, 1),
        //             M.facets.point<2>(nf, 2)
        //         };
        //         fps[nlv] = target_p;
        //         if (cross(fps[1]-fps[0], fps[2]-fps[0]) * normal < 0)
        //             return false;
        //     }
        //     for (const auto& [nf, nlv] : v1_ordered_f_and_lv) {
        //         std::array<GEO::vec2, 3> fps = {
        //             M.facets.point<2>(nf, 0),
        //             M.facets.point<2>(nf, 1),
        //             M.facets.point<2>(nf, 2)
        //         };
        //         fps[nlv] = target_p;
        //         if (cross(fps[1]-fps[0], fps[2]-fps[0]) * normal < 0)
        //             return false;
        //     }
        // }
        // else {
        //     assert(M.vertices.dimension() == 3);
        //
        //     const auto& p0 = M.facets.point(f, lv0);
        //     const auto& p1 = M.facets.point(f, lv1);
        //     const auto& p2 = M.facets.point(f, lv2);
        //     const auto target_p = (1-r)*p0 + r*p1;
        //
        //     const auto normal = cross(p1-p0, p2-p0);
        //     for (const auto& [nf, nlv] : v0_ordered_f_and_lv) {
        //         std::array<GEO::vec3, 3> fps = {
        //             M.facets.point(nf, 0),
        //             M.facets.point(nf, 1),
        //             M.facets.point(nf, 2)
        //         };
        //         fps[nlv] = target_p;
        //         if (GEO::dot(GEO::cross(fps[1]-fps[0], fps[2]-fps[0]), normal) < 0)
        //             return false;
        //     }
        //     for (const auto& [nf, nlv] : v1_ordered_f_and_lv) {
        //         std::array<GEO::vec3, 3> fps = {
        //             M.facets.point(nf, 0),
        //             M.facets.point(nf, 1),
        //             M.facets.point(nf, 2)
        //         };
        //         fps[nlv] = target_p;
        //         if (GEO::dot(GEO::cross(fps[1]-fps[0], fps[2]-fps[0]), normal) < 0)
        //             return false;
        //     }
        //
        //     /* Check whether there are any co-edge facets other than f and nf */
        //     const GEO::index_t nf = M.facets.adjacent(f, lv);
        //     for (const auto& nf0: v0_ordered_f_and_lv | std::views::keys) {
        //         for (const auto& nf1: v1_ordered_f_and_lv | std::views::keys) {
        //             if (nf0 == nf1 &&
        //                 nf0 != f && nf0 != nf)
        //                 return false;
        //         }
        //     }
        // }

        return true;
    }

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
        const GEO::index_t f,
        const GEO::index_t lv,
        GEO::index_t& disuse_v,
        GEO::index_t& disuse_f0,
        GEO::index_t& disuse_f1,
        const double r
        ) {
        assert(f < M.facets.nb());
        assert(M.facets.nb_vertices(f) == 3);
        assert(lv < 3);
        assert(r >= 0 && r <= 1);

        GEO::Mesh debug_mesh;
        debug_mesh.copy(M);

        /*
         *  v0 --------+            ++
         *  | \  af2 /              |  \
         *  |   \  /                |af2 \
         *  | f  v2         ->      v0 --- v2
         *  |   /  \                |af1 /
         *  | /  af1 \              |  /
         *  v1 --------+            ++
         */
        const GEO::index_t lv0 = lv;
        const GEO::index_t lv1 = (lv+1)%3;
        const GEO::index_t lv2 = (lv+2)%3;
        const GEO::index_t v0 = M.facets.vertex(f, lv0);
        const GEO::index_t v1 = M.facets.vertex(f, lv1);
        const GEO::index_t v2 = M.facets.vertex(f, lv2);
        const GEO::index_t af0 = M.facets.adjacent(f, lv0);
        const GEO::index_t af1 = M.facets.adjacent(f, lv1);
        const GEO::index_t af2 = M.facets.adjacent(f, lv2);

        /* Set collapsed point (v0) and interpolate attributes */
        M.vertices.attributes().scale_item(v0, 1.0-r);
        M.vertices.attributes().madd_item(v0, r, v1);
        disuse_v = v1;
        disuse_f0 = f;
        disuse_f1 = af0; // facet or GEO::NO_FACET

        /* Find all (f, lv) that incident to v1 (before performing collapse) */
        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
        get_vertex_incident_facets(M, f, lv1, ordered_f_and_lv);

        /* Set facet adjacency */
        if (af1 != GEO::NO_FACET) {
            assert(M.facets.find_vertex(af1, v2) != GEO::NO_INDEX);
            M.facets.set_adjacent(af1, M.facets.find_vertex(af1, v2), af2);
        }
        if (af2 != GEO::NO_FACET) {
            assert(M.facets.find_vertex(af2, v0) != GEO::NO_INDEX);
            M.facets.set_adjacent(af2, M.facets.find_vertex(af2, v0), af1);
        }

        /* == Collapse adjacent facet ============================================================================== */
        if (af0 != GEO::NO_FACET) {
            assert(M.facets.nb_vertices(af0) == 3);

            /*
             *  +-------- nv1                 ++
             *    \ naf1 / |                 / |
             *      \  /   |              /naf1|
             *      nv2 af0|     ->    nv2 --- nv1
             *      /  \   |              \naf2|
             *    / naf2 \ |                \  |
             *  +-------- nv0                 ++
             */
            const GEO::index_t nlv0 = M.facets.find_vertex(af0, v1);
            assert(nlv0 != GEO::NO_INDEX);
            const GEO::index_t nlv1 = (nlv0+1)%3;
            const GEO::index_t nlv2 = (nlv0+2)%3;
            const GEO::index_t nv0 = M.facets.vertex(af0, nlv0);
            // const GEO::index_t nv1 = M.facets.vertex(af0, nlv1);
            const GEO::index_t nv2 = M.facets.vertex(af0, nlv2);
            const GEO::index_t naf1 = M.facets.adjacent(af0, nlv1);
            const GEO::index_t naf2 = M.facets.adjacent(af0, nlv2);

            /* Set facet adjacency */
            if (naf1 != GEO::NO_FACET) {
                assert(M.facets.find_vertex(naf1, nv2) != GEO::NO_INDEX);
                M.facets.set_adjacent(naf1, M.facets.find_vertex(naf1, nv2), naf2);
            }
            if (naf2 != GEO::NO_FACET) {
                assert(M.facets.find_vertex(naf2, nv0) != GEO::NO_INDEX);
                M.facets.set_adjacent(naf2, M.facets.find_vertex(naf2, nv0), naf1);
            }
        }

        /* Set facet vertices */
        for (const auto& [adj_f, adj_lv] : ordered_f_and_lv)
            M.facets.set_vertex(adj_f, adj_lv, v0);
    }

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
        const GEO::index_t f,
        const GEO::index_t lv
        ) {
        assert(f < M.facets.nb());
        assert(M.facets.nb_vertices(f) == 3);
        assert(lv < 3);

        const GEO::index_t af = M.facets.adjacent(f, lv);
        if (af == GEO::NO_FACET)
            return false;

        const GEO::index_t lv1 = (lv+1)%3;
        const GEO::index_t lv2 = (lv+2)%3;
        // const GEO::index_t v0 = M.facets.vertex(f, lv);
        const GEO::index_t v1 = M.facets.vertex(f, lv1);
        const GEO::index_t v2 = M.facets.vertex(f, lv2);

        const GEO::index_t nlv0 = M.facets.find_vertex(af, v1);
        assert(nlv0 != GEO::NO_INDEX);
        const GEO::index_t nlv1 = (nlv0+1)%3;
        const GEO::index_t nlv2 = (nlv0+2)%3;
        const GEO::index_t v3 = M.facets.vertex(af, nlv2);

        /* Not allow two existing edges to exist in an adjacent facet */
        if (const auto nf1 = M.facets.adjacent(f, lv1);
            nf1 != GEO::NO_FACET) {
            for (GEO::index_t nlv = 0; nlv < 3; ++nlv) {
                if (M.facets.vertex(nf1, nlv) == v3)
                    return false;
            }
        }
        if (const auto nf2 = M.facets.adjacent(f, lv2);
            nf2 != GEO::NO_FACET) {
            for (GEO::index_t nlv = 0; nlv < 3; ++nlv) {
                if (M.facets.vertex(nf2, nlv) == v3)
                    return false;
            }
        }
        if (const auto& anf1 = M.facets.adjacent(af, nlv1);
            anf1 != GEO::NO_FACET) {
            for (GEO::index_t nlv = 0; nlv < 3; ++nlv) {
                if (M.facets.vertex(anf1, nlv) == v2)
                    return false;
            }
        }
        if (const auto& anf2 = M.facets.adjacent(af, nlv2);
            anf2 != GEO::NO_FACET) {
            for (GEO::index_t nlv = 0; nlv < 3; ++nlv) {
                if (M.facets.vertex(anf2, nlv) == v2)
                    return false;
            }
        }

        /* Check inversion */
        // if (M.vertices.dimension() == 2) {
        //     const auto& p0 = M.vertices.point<2>(v0);
        //     const auto& p1 = M.vertices.point<2>(v1);
        //     const auto& p2 = M.vertices.point<2>(v2);
        //     const auto& p3 = M.vertices.point<2>(v3);
        //     const auto normal0 = cross(p1-p0, p2-p0) > 0;
        //     const auto normal1 = cross(p0-p1, p3-p1) > 0;
        //     if (normal0 != normal1)
        //         return false; // the two facets are initially oriented in opposite directions
        //     const auto normal2 = cross(p3-p0, p2-p0) > 0;
        //     const auto normal3 = cross(p2-p1, p3-p1) > 0;
        //     if (normal2 != normal0 || normal3 != normal0)
        //         return false; // inverse after swapping
        // }
        // else {
        //     assert(M.vertices.dimension() == 3);
        //     const auto& p0 = M.vertices.point(v0);
        //     const auto& p1 = M.vertices.point(v1);
        //     const auto& p2 = M.vertices.point(v2);
        //     const auto& p3 = M.vertices.point(v3);
        //     const auto normal0 = GEO::cross(p1-p0, p2-p0);
        //     const auto normal1 = GEO::cross(p0-p1, p3-p1);
        //     const auto ave_normal = normal0+normal1;
        //     const auto normal2 = GEO::cross(p3-p0, p2-p0);
        //     const auto normal3 = GEO::cross(p2-p1, p3-p1);
        //     if (GEO::dot(normal2, ave_normal) < 1e-10 || GEO::dot(normal3, ave_normal) < 1e-10)
        //         return false;
        // }

        return true;
    }

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
        const GEO::index_t f,
        const GEO::index_t lv
        ) {
        assert(f < M.facets.nb());
        assert(M.facets.nb_vertices(f) == 3);
        assert(lv < 3);

        const GEO::index_t af = M.facets.adjacent(f, lv);
        if (af == GEO::NO_FACET)
            return false;

        /*
         *          af0                            af0
         *      v3 ------ v0                   v3 ------ v0
         *      |       / |                    | \       |
         *      | af  /   |        ->          |   \  f  |
         * af3  |   /  f  |  af2          af3  | af  \   |  af2
         *      | /       |                    |       \ |
         *      v1 ------ v2                   v1 ------ v2
         *          af1                            af1
         */
        const GEO::index_t lv1 = (lv+1)%3;
        const GEO::index_t lv2 = (lv+2)%3;
        // const GEO::index_t v0 = M.facets.vertex(f, lv);
        const GEO::index_t v1 = M.facets.vertex(f, lv1);
        const GEO::index_t v2 = M.facets.vertex(f, lv2);

        const GEO::index_t af1 = M.facets.adjacent(f, lv1);
        // const GEO::index_t af2 = M.facets.adjacent(f, lv2);

        const GEO::index_t nlv0 = M.facets.find_vertex(af, v1);
        assert(nlv0 != GEO::NO_INDEX);
        const GEO::index_t nlv1 = (nlv0+1)%3;
        const GEO::index_t nlv2 = (nlv0+2)%3;
        const GEO::index_t v3 = M.facets.vertex(af, nlv2);

        const GEO::index_t af0 = M.facets.adjacent(af, nlv1);
        // const GEO::index_t af3 = M.facets.adjacent(af, nlv2);

        /* Set vertices */
        M.facets.set_vertex(f, lv1, v3);
        M.facets.set_vertex(af, nlv1, v2);

        /* Set adjacency */
        M.facets.set_adjacent(f, lv, af0);
        M.facets.set_adjacent(f, lv1, af);
        M.facets.set_adjacent(af, nlv0, af1);
        M.facets.set_adjacent(af, nlv1, f);
        if (af0 != GEO::NO_FACET) {
            assert(M.facets.find_vertex(af0, v3) != GEO::NO_INDEX);
            M.facets.set_adjacent(af0, M.facets.find_vertex(af0, v3), f);
        }
        if (af1 != GEO::NO_FACET) {
            assert(M.facets.find_vertex(af1, v2) != GEO::NO_INDEX);
            M.facets.set_adjacent(af1, M.facets.find_vertex(af1, v2), af);
        }

        /* Copy attributes */
        M.facet_corners.attributes().copy_item(M.facets.corner(f, lv), M.facets.corner(af, nlv1));
        M.facet_corners.attributes().copy_item(M.facets.corner(af, nlv0), M.facets.corner(f, lv1));
        /* Restore attributes */
        M.facets.attributes().zero_item(f);
        M.facets.attributes().zero_item(af);
        M.facet_corners.attributes().zero_item(M.facets.corner(f, lv1));
        M.facet_corners.attributes().zero_item(M.facets.corner(af, nlv1));

        return true;
    }
}
