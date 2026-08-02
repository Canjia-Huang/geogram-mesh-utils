//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_SPLIT_OPERATION_H
#define GEOLIO_SPLIT_OPERATION_H

#include "base_operation.h"

namespace geolio
{
    class SplitOperation : public BaseOperation {
    public:
        /**
         * @brief Constructs a SplitOperation for splitting overly long edges.
         * @details Initializes the base operation and stores the minimum edge length above
         *          which an edge is eligible for splitting.
         * @param[in] mesh_element_manager The mesh element manager exposing the mesh and its
         *                                 usage/fixed element attributes.
         * @param[in] limit_edge_length Edges shorter than this threshold are never split.
         */
        explicit SplitOperation(
            MeshElementManager& mesh_element_manager,
            double limit_edge_length,
            bool allow_split_fixed_edges = true);

        ~SplitOperation();

        bool do_once(bool iteratively = true);

        /**
         * @brief Executes a single pass of edge-splitting over the whole mesh.
         * @details Resets the per-facet "processed" flags, then iterates over every facet and
         *          local edge; for each edge that passes is_perform_valid(), it performs the
         *          split, applies post_process() bookkeeping, asserts post_check(), and marks the
         *          original facet, the newly created facets and the adjacent facet as processed
         *          so they are not revisited in this pass.
         */
        void run_through(bool iteratively = true);

    private:
        struct EdgeToSplit {
            EdgeToSplit(
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

            bool operator<(const EdgeToSplit& other) const { // max-heap
                return length < other.length;
            }
        };

        /**
         * @brief Checks whether the edge of facet @p f at local vertex @p lv may be split.
         * @details Verifies that the facet is still in use, that the edge is not fixed unless
         *          ALLOW_SPLIT_FIXED_EDGES is set, and that the edge is at least as long as
         *          limit_edge_length_.
         * @param[in] f Index of the facet adjacent to the candidate edge.
         * @param[in] lv Local vertex index identifying the oriented edge (lv -> lv+1).
         * @return true if the edge may be split; false otherwise.
         */
        [[nodiscard]] bool is_perform_valid(GEO::index_t f, GEO::index_t lv) const;

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
        void perform(GEO::index_t f, GEO::index_t lv,
                     GEO::index_t& new_v, GEO::index_t& new_f0, GEO::index_t& new_f1) const;

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
        void post_process(GEO::index_t f, GEO::index_t lv,
                          GEO::index_t new_v, GEO::index_t new_f0, GEO::index_t new_f1) const;

        const double limit_edge_length_;

        bool ALLOW_SPLIT_FIXED_EDGES_ = true; // When true, splitting of fixed (locked) edges is allowed.

        std::priority_queue<EdgeToSplit> pq_;

        GEO::Attribute<bool> mesh_fc_locked_; // locked edge should not be split (only used when not allow to split fixed edges)
    };
}

#endif //GEOLIO_SPLIT_OPERATION_H
