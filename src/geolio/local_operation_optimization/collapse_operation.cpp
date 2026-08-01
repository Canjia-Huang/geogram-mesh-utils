//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "collapse_operation.h"
#include <cassert>
#include <utility>
#include <vector>
#include <geogram/mesh/mesh_io.h>
#include <geolio/common/log.h>
#include <geolio/mesh/tri_operations.h>

#include "geolio/mesh/mesh_operations.h"

namespace geolio
{
    /**
     * @brief Constructs a CollapseOperation for collapsing overly short edges.
     * @details Initializes the base operation and stores the maximum edge length below
     *          which an edge is eligible for collapse.
     * @param[in] mesh_element_manager The mesh element manager exposing the mesh and its
     *                                 usage/fixed element attributes.
     * @param[in] limit_edge_length Edges longer than this threshold are never collapsed.
     */
    CollapseOperation::CollapseOperation(
        MeshElementManager& mesh_element_manager,
        const double limit_edge_length
        ) : BaseOperation(mesh_element_manager),
            limit_edge_length_(limit_edge_length)
    {}

    /**
     * @brief Executes a single pass of edge-collapse over the whole mesh.
     * @details Resets the per-facet "processed" flags, then iterates over every facet and
     *          local edge; for each edge that passes is_perform_valid(), it performs the
     *          collapse, applies post_process() bookkeeping, and asserts post_check().
     *          Processed flags are not set after a collapse because the collapsed facet is
     *          marked disused, so a later re-scan of that facet is safely rejected.
     */
    void CollapseOperation::perform_one_pass(
        ) {
        mesh_f_processed_.fill(false);

        for (GEO::index_t f = 0, f_end = mesh_.facets.nb(); f < f_end; ++f) {
            if (mesh_f_processed_[f])
                continue;

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (!is_perform_valid(f, lv))
                    continue;

                const auto v = mesh_.facets.vertex(f, lv);

                GEO::index_t disuse_v0, disuse_v1, disuse_v2, disuse_f0, disuse_f1;
                perform(f, lv, disuse_v0, disuse_v1, disuse_v2, disuse_f0, disuse_f1);

                post_process(f, lv, disuse_v0, disuse_v1, disuse_v2, disuse_f0, disuse_f1);

                assert(post_check());
            }
        }
    }

    /**
     * @brief Checks whether the edge of facet @p f at local vertex @p lv may be collapsed.
     * @details Verifies that the facet is still in use, that the edge endpoint v1 is not
     *          fixed and not incident to a fixed edge (unless ALLOW_COLLAPSE_FIXED_EDGES is
     *          set), that neither endpoint is non-manifold, that the edge is not longer than
     *          limit_edge_length_, and that is_tri_edge_collapse_valid() accepts the collapse.
     * @param[in] f Index of the facet adjacent to the candidate edge.
     * @param[in] lv Local vertex index identifying the oriented edge (lv -> lv+1).
     * @return true if the edge may be collapsed; false otherwise.
     */
    bool CollapseOperation::is_perform_valid(
        const GEO::index_t f,
        const GEO::index_t lv
        ) const {
        assert(f < mesh_.facets.nb());
        assert(lv < 3);

        if (!manager_.mesh_f_used[f]) // This facet should not yet exist.
            return false;

        const auto nf = mesh_.facets.adjacent(f, lv);
        const bool EDGE_ON_BOUNDARY = (nf == GEO::NO_FACET);
        const auto ev0 = mesh_.facets.vertex(f, lv);
        const auto ev1 = mesh_.facets.vertex(f, (lv+1)%3);

        if (manager_.mesh_v_fixed[ev1]) /* Collapse pulls v1 toward v0, no operation is performed when v1
                        is fixed, so that the vertex indices remain unchanged. */
            return false;
        { // The fixed edge involving ev1 also prevents collapse (because it would remove ev1).
            std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
            get_vertex_incident_facets(mesh_, f, (lv+1)%3, ordered_f_and_lv);
            for (const auto& [ff, llv] : ordered_f_and_lv) {
                if (manager_.mesh_fc_fixed[mesh_.facets.corner(ff, llv)])
                    return false;
            }
        }

        if (ALLOW_COLLAPSE_FIXED_EDGES) {
            // TODO: Sliding along collinear edges is permitted.
        }
        else if (const auto& fc = mesh_.facets.corner(f, lv);
                manager_.mesh_fc_fixed[fc]) // Do not collapse the fixed edge.
            return false;

        if (manager_.mesh_v_non_manifold[ev0] ||
            manager_.mesh_v_non_manifold[ev1]) // After collapse, non-manifold vertices will be retained.
            return false;

        if (const auto edge_length = manager_.get_edge_length(f, lv);
            edge_length > limit_edge_length_) // Do not collapse edges greater than the limit length.
            return false;

        if (!is_tri_edge_collapse_valid(mesh_, f, lv)) // Collapse operation is not valid.
            return false;

        return true;
    }

    /**
     * @brief Performs the edge collapse on the mesh topology.
     * @details For an isolated facet the collapse removes the facet and all three of its
     *          vertices directly. Otherwise it computes a collapse ratio R (0 when the edge
     *          endpoint or a fixed edge pulls toward v0, 0.5 for the midpoint) and calls
     *          tri_edge_collapse() to merge the two edge endpoints and rewire connectivity.
     * @param[in] f Index of the facet adjacent to the edge to collapse.
     * @param[in] lv Local vertex index identifying the oriented edge (lv -> lv+1).
     * @param[out] disuse_v0 Receives the collapsed-away vertex index (the first of the three
     *                       removed vertices for an isolated facet).
     * @param[out] disuse_v1 Receives the second removed vertex of an isolated facet, or
     *                       GEO::NO_VERTEX for a non-isolated collapse.
     * @param[out] disuse_v2 Receives the third removed vertex of an isolated facet, or
     *                       GEO::NO_VERTEX for a non-isolated collapse.
     * @param[out] disuse_f0 Receives the index of the first disused facet (the collapsed one).
     * @param[out] disuse_f1 Receives the index of the opposite disused facet, or GEO::NO_FACET.
     */
    void CollapseOperation::perform(
        const GEO::index_t f,
        const GEO::index_t lv,
        GEO::index_t& disuse_v0,
        GEO::index_t& disuse_v1,
        GEO::index_t& disuse_v2,
        GEO::index_t& disuse_f0,
        GEO::index_t& disuse_f1
        ) const {
        assert(f < mesh_.facets.nb());
        assert(lv < 3);

        disuse_v0 = GEO::NO_VERTEX;
        disuse_v1 = GEO::NO_VERTEX;
        disuse_v2 = GEO::NO_VERTEX;
        disuse_f0 = GEO::NO_FACET;
        disuse_f1 = GEO::NO_FACET;

        if (mesh_.facets.adjacent(f, 0) == GEO::NO_FACET &&
            mesh_.facets.adjacent(f, 1) == GEO::NO_FACET &&
            mesh_.facets.adjacent(f, 2) == GEO::NO_FACET
            ) { // For an isolated facet, collapse will directly remove it.
            disuse_v0 = mesh_.facets.vertex(f, 0);
            disuse_v1 = mesh_.facets.vertex(f, 1);
            disuse_v2 = mesh_.facets.vertex(f, 2);
            disuse_f0 = f;
            return;
        }

        double R = 0.5; // mid point
        if (const auto& ev0 = mesh_.facets.vertex(f, lv);
            manager_.mesh_v_fixed[ev0]) // pull ev1 -> ev0
            R = 0;
        { // The fixed edge involving ev0 also pull ev1 -> ev0.
            std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
            get_vertex_incident_facets(mesh_, f, lv, ordered_f_and_lv);
            for (const auto& [ff, llv] : ordered_f_and_lv) {
                if (manager_.mesh_fc_fixed[mesh_.facets.corner(ff, llv)]) {
                    R = 0;
                    break;
                }
            }
        }

        tri_edge_collapse(mesh_, f, lv, disuse_v0, disuse_f0, disuse_f1, R);
    }

    /**
     * @brief Applies post-collapse bookkeeping to the manager's element attributes.
     * @details Propagates the boundary flag from the collapsed vertex to the surviving vertex,
     *          re-marks fixed-edge corner flags on the neighbouring facets when the collapsed
     *          edge (or an adjacent facet edge) was fixed, and finally disposes all disused
     *          vertices and facets through the manager for recycling.
     * @param[in] f Index of the facet that contained the collapsed edge.
     * @param[in] lv Local vertex index that identified the collapsed edge.
     * @param[in] disuse_v0 First disused vertex index reported by perform().
     * @param[in] disuse_v1 Second disused vertex index, or GEO::NO_VERTEX.
     * @param[in] disuse_v2 Third disused vertex index, or GEO::NO_VERTEX.
     * @param[in] disuse_f0 Index of the disused (collapsed) facet reported by perform().
     * @param[in] disuse_f1 Index of the opposite disused facet, or GEO::NO_FACET.
     */
    void CollapseOperation::post_process(
        const GEO::index_t f,
        const GEO::index_t lv,
        const GEO::index_t disuse_v0,
        const GEO::index_t disuse_v1,
        const GEO::index_t disuse_v2,
        const GEO::index_t disuse_f0,
        const GEO::index_t disuse_f1
        ) const {
        assert(f < mesh_.facets.nb());
        assert(lv < 3);
        assert(disuse_v0 != GEO::NO_VERTEX);
        assert(disuse_f0 != GEO::NO_FACET);

        /* After edge collapse on a boundary vertex, it remains a boundary vertex. */
        const auto v0 = mesh_.facets.vertex(disuse_v0, lv);
        if (manager_.mesh_v_boundary[disuse_v0])
            manager_.mesh_v_boundary[v0] = true;

        /* Update fixed edges */
        {
            if (manager_.mesh_fc_fixed[mesh_.facets.corner(disuse_f0, (lv+1)%3)] ||
                manager_.mesh_fc_fixed[mesh_.facets.corner(disuse_f0, (lv+2)%3)]
                ) {
                const auto v2 = mesh_.facets.vertex(disuse_f0, (lv+2)%3);
                if (const auto& nf1 = mesh_.facets.adjacent(disuse_f0, (lv+1)%3);
                    nf1 != GEO::NO_FACET) {
                    const auto nlv = mesh_.facets.find_vertex(nf1, v2);
                    assert(nlv != GEO::NO_INDEX);
                    manager_.mesh_fc_fixed[mesh_.facets.corner(nf1, nlv)] = true;
                }
                if (const auto& nf2 = mesh_.facets.adjacent(disuse_f0, (lv+2)%3);
                    nf2 != GEO::NO_FACET) {
                    const auto nlv = mesh_.facets.find_vertex(nf2, v2);
                    assert(nlv != GEO::NO_INDEX);
                    manager_.mesh_fc_fixed[mesh_.facets.corner(nf2, (nlv+2)%3)] = true;
                }
            }
            if (disuse_f1 != GEO::NO_FACET) {
                // mesh_.facets.vertex(disuse_f1, (nlv+1)%3) is not reliable (-> v0 rather than v1)
                GEO::index_t nlv = GEO::NO_INDEX;
                for (GEO::index_t i = 0; i < 3; ++i) {
                    if (mesh_.facets.adjacent(disuse_f1, i) == disuse_f0) {
                        nlv = i;
                        break;
                    }
                }
                assert(nlv != GEO::NO_INDEX);

                if (manager_.mesh_fc_fixed[mesh_.facets.corner(disuse_f1, (nlv+1)%3)] ||
                    manager_.mesh_fc_fixed[mesh_.facets.corner(disuse_f1, (nlv+2)%3)]
                    ) {
                    const auto v3 = mesh_.facets.vertex(disuse_f1, (nlv+2)%3);
                    if (const auto& nf0 = mesh_.facets.adjacent(disuse_f1, (nlv+1)%3);
                        nf0 != GEO::NO_FACET) {
                        const auto nnlv = mesh_.facets.find_vertex(nf0, v3);
                        assert(nnlv != GEO::NO_INDEX);
                        manager_.mesh_fc_fixed[mesh_.facets.corner(nf0, nnlv)] = true;
                    }
                    if (const auto& nf3 = mesh_.facets.adjacent(disuse_f1, (nlv+2)%3);
                        nf3 != GEO::NO_FACET) {
                        const auto nnlv = mesh_.facets.find_vertex(nf3, v3);
                        assert(nnlv != GEO::NO_INDEX);
                        manager_.mesh_fc_fixed[mesh_.facets.corner(nf3, (nnlv+2)%3)] = true;
                    }
                }
            }
        }

        /* Disuse elements */
        manager_.disuse_vertex(disuse_v0);
        if (disuse_v1 != GEO::NO_VERTEX)
            manager_.disuse_vertex(disuse_v1);
        if (disuse_v2 != GEO::NO_VERTEX)
            manager_.disuse_vertex(disuse_v2);
        assert(disuse_f0 != GEO::NO_FACET);
        manager_.disuse_facet(disuse_f0);
        if (disuse_f1 != GEO::NO_FACET)
            manager_.disuse_facet(disuse_f1);
    }
}
