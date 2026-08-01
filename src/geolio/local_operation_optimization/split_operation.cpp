//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "split_operation.h"
#include <geolio//mesh/tri_operations.h>

namespace geolio
{
    /**
     * @brief Constructs a SplitOperation for splitting overly long edges.
     * @details Initializes the base operation and stores the minimum edge length above
     *          which an edge is eligible for splitting.
     * @param[in] mesh_element_manager The mesh element manager exposing the mesh and its
     *                                 usage/fixed element attributes.
     * @param[in] limit_edge_length Edges shorter than this threshold are never split.
     */
    SplitOperation::SplitOperation(
        MeshElementManager& mesh_element_manager,
        const double limit_edge_length
        ) : BaseOperation(mesh_element_manager),
            limit_edge_length_(limit_edge_length)
    {}

    /**
     * @brief Executes a single pass of edge-splitting over the whole mesh.
     * @details Resets the per-facet "processed" flags, then iterates over every facet and
     *          local edge; for each edge that passes is_perform_valid(), it performs the
     *          split, applies post_process() bookkeeping, asserts post_check(), and marks the
     *          original facet, the newly created facets and the adjacent facet as processed
     *          so they are not revisited in this pass.
     */
    void SplitOperation::perform_one_pass(
        ) {
        mesh_f_processed_.fill(false);

        for (GEO::index_t f = 0, f_end = mesh_.facets.nb(); f < f_end; ++f) {
            if (mesh_f_processed_[f])
                continue;

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (!is_perform_valid(f, lv))
                    continue;

                GEO::index_t new_v, new_f0, new_f1;
                perform(f, lv, new_v, new_f0, new_f1);

                post_process(f, lv, new_v, new_f0, new_f1);

                assert(post_check());

                /* Label processed facets */
                mesh_f_processed_[f] = true;
                mesh_f_processed_[new_f0] = true;
                if (const auto& nf = mesh_.facets.adjacent(f, lv);
                    nf != GEO::NO_FACET
                    ) {
                    mesh_f_processed_[nf] = true;
                    assert(new_f1 != GEO::NO_FACET);
                    mesh_f_processed_[new_f1] = true;
                }
            }
        }
    }

    /**
     * @brief Checks whether the edge of facet @p f at local vertex @p lv may be split.
     * @details Verifies that the facet is still in use, that the edge is not fixed unless
     *          ALLOW_SPLIT_FIXED_EDGES is set, and that the edge is at least as long as
     *          limit_edge_length_.
     * @param[in] f Index of the facet adjacent to the candidate edge.
     * @param[in] lv Local vertex index identifying the oriented edge (lv -> lv+1).
     * @return true if the edge may be split; false otherwise.
     */
    bool SplitOperation::is_perform_valid(
        const GEO::index_t f,
        const GEO::index_t lv
        ) const {
        assert(f < mesh_.facets.nb());
        assert(lv < 3);

        if (!manager_.mesh_f_used[f]) // This facet should not yet exist.
            return false;

        if (const auto& fc = mesh_.facets.corner(f, lv);
            !ALLOW_SPLIT_FIXED_EDGES
            && manager_.mesh_fc_fixed[fc]) // Splitting fixed edges is not allowed.
            return false;

        if (const auto edge_length = manager_.get_edge_length(f, lv);
            edge_length < limit_edge_length_) // Do not split edges lesser than the limit length.
            return false;

        return true;
    }

    /**
     * @brief Performs the edge split on the mesh topology.
     * @details Determines whether the edge is on the boundary, requests a new vertex and new
     *          facets from the manager (two facets for an interior edge, one for a boundary
     *          edge), and calls tri_edge_split() to insert the new vertex at the edge
     *          midpoint and rewire the incident triangles.
     * @param[in] f Index of the facet adjacent to the edge to split.
     * @param[in] lv Local vertex index identifying the oriented edge (lv -> lv+1).
     * @param[out] new_v Receives the index of the newly created vertex.
     * @param[out] new_f0 Receives the index of the first newly created facet.
     * @param[out] new_f1 Receives the index of the second newly created facet, or
     *                     GEO::NO_FACET when the edge is on the boundary.
     */
    void SplitOperation::perform(
        const GEO::index_t f,
        const GEO::index_t lv,
        GEO::index_t& new_v,
        GEO::index_t& new_f0,
        GEO::index_t& new_f1
        ) const {
        assert(f < mesh_.facets.nb());
        assert(lv < 3);

        const bool EDGE_ON_BOUNDARY = mesh_.facets.adjacent(f, lv) == GEO::NO_FACET;

        /* Split */
        new_v = manager_.require_new_vertex();
        new_f0 = manager_.require_new_facet();
        new_f1 = EDGE_ON_BOUNDARY ? GEO::NO_FACET : manager_.require_new_facet();
        tri_edge_split(mesh_, f, lv, new_v, new_f0, new_f1);
    }

    /**
     * @brief Applies post-split bookkeeping to the manager's element attributes.
     * @details If the split edge lies on the boundary, marks the newly created vertex as a
     *          boundary vertex so the boundary attribute is inherited by the new vertex.
     * @param[in] f Index of the facet that contained the split edge.
     * @param[in] lv Local vertex index that identified the split edge.
     * @param[in] new_v Index of the newly created vertex.
     * @param[in] new_f0 Index of the first newly created facet.
     * @param[in] new_f1 Index of the second newly created facet, or GEO::NO_FACET.
     */
    void SplitOperation::post_process(
        const GEO::index_t f,
        const GEO::index_t lv,
        const GEO::index_t new_v,
        const GEO::index_t new_f0,
        const GEO::index_t new_f1
        ) const {
        assert(f < mesh_.facets.nb());
        assert(lv < 3);
        assert(new_v < mesh_.vertices.nb());
        assert(new_f0 < mesh_.facets.nb());

        const auto nf = mesh_.facets.adjacent(f, lv);
        const bool EDGE_ON_BOUNDARY = (nf == GEO::NO_FACET);

        if (EDGE_ON_BOUNDARY) // Split edge inherits boundary attribute.
            manager_.mesh_v_boundary[new_v] = true;
    }
}
