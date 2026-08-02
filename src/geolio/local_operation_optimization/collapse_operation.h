//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_COLLAPSE_OPERATION_H
#define GEOLIO_COLLAPSE_OPERATION_H

#include "base_operation.h"

namespace geolio
{
    class CollapseOperation : public BaseOperation {
    public:
        /**
         * @brief Constructs a CollapseOperation for collapsing overly short edges.
         * @details Initializes the base operation and stores the maximum edge length below
         *          which an edge is eligible for collapse.
         * @param[in] mesh_element_manager The mesh element manager exposing the mesh and its
         *                                 usage/fixed element attributes.
         * @param[in] limit_edge_length Edges longer than this threshold are never collapsed.
         */
        explicit CollapseOperation(
            MeshElementManager& mesh_element_manager,
            double limit_edge_length,
            bool allow_collapse_fixed_edges = true);

        bool do_once(bool iteratively = true);

        /**
         * @brief Executes a single pass of edge-collapse over the whole mesh.
         * @details Resets the per-facet "processed" flags, then iterates over every facet and
         *          local edge; for each edge that passes is_perform_valid(), it performs the
         *          collapse, applies post_process() bookkeeping, and asserts post_check().
         *          Processed flags are not set after a collapse because the collapsed facet is
         *          marked disused, so a later re-scan of that facet is safely rejected.
         */
        void run_through(bool iteratively = true);

    private:
        struct EdgeToCollapse {
            EdgeToCollapse(
                const GEO::index_t _f,
                const GEO::index_t _lv,
                const GEO::index_t _timestamping,
                const double _length
            ) : f(_f), lv(_lv), timestamping(_timestamping), length(_length)
            {}

            GEO::index_t f = GEO::NO_FACET;
            GEO::index_t lv = GEO::NO_INDEX;
            GEO::index_t timestamping = GEO::NO_INDEX;
            double length = -1.0;

            bool operator<(const EdgeToCollapse& other) const { // min-heap
                return length > other.length;
            }
        };

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
        [[nodiscard]] bool is_perform_valid(GEO::index_t f, GEO::index_t lv) const;

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
        void perform(GEO::index_t f, GEO::index_t lv,
                     GEO::index_t& disuse_v0, GEO::index_t& disuse_v1, GEO::index_t& disuse_v2,
                     GEO::index_t& disuse_f0, GEO::index_t& disuse_f1) const;

        /**
         * @brief Applies post-collapse bookkeeping to the manager's element attributes.
         * @details Propagates the boundary flag from the collapsed vertex to the surviving vertex,
         *          re-marks fixed-edge corner flags on the neighbouring facets when the collapsed
         *          edge (or an adjacent facet edge) was fixed, and finally disposes all disused
         *          vertices and facets through the manager for recycling.
         * @param[in] f Index of the facet that contained the collapsed edge.
         * @param[in] lv Local vertex index that identified the collapsed edge.
         * @param[in] disuse_v0 Collapsed-away vertex index reported by perform().
         * @param[in] disuse_v1 Second disused vertex index, or GEO::NO_VERTEX.
         * @param[in] disuse_v2 Third disused vertex index, or GEO::NO_VERTEX.
         * @param[in] disuse_f0 Index of the disused (collapsed) facet reported by perform().
         * @param[in] disuse_f1 Index of the opposite disused facet, or GEO::NO_FACET.
         */
        void post_process(GEO::index_t f, GEO::index_t lv,
                          GEO::index_t disuse_v0, GEO::index_t disuse_v1, GEO::index_t disuse_v2,
                          GEO::index_t disuse_f0, GEO::index_t disuse_f1) const;

        const double limit_edge_length_;

        bool ALLOW_COLLAPSE_FIXED_EDGES_ = true; // When true, collapse of fixed (locked) edges is allowed.

        std::priority_queue<EdgeToCollapse> pq_;

        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv_0_; // just pre-allocated
        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv_1_; // just pre-allocated
    };
}

#endif //GEOLIO_COLLAPSE_OPERATION_H
