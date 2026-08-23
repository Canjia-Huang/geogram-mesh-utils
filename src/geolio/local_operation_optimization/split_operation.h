//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_SPLIT_OPERATION_H
#define GEOLIO_SPLIT_OPERATION_H

#include "base_operation.h"
#include <queue>

namespace geolio
{
    template <GEO::index_t DIM>
    class SplitOperation : public BaseOperation<DIM> {
    public:
        /**
         * @brief Constructs a SplitOperation for splitting overly long edges.
         * @details Initializes the base operation and stores the split threshold. When
         *          fixed-edge splitting is disallowed, it additionally binds and pre-fills a
         *          per-corner "locked" attribute marking the fixed edges, then pre-populates a
         *          max-heap priority queue with every currently eligible edge (ordered by
         *          length, longest first).
         * @param[in] mesh_element_manager The mesh element manager exposing the mesh and its
         *                                 usage/fixed element attributes.
         * @param[in] limit_edge_length Edges shorter than this threshold are never split.
         * @param[in] allow_split_fixed_edges When true, fixed (locked) edges may be split; when
         *                                    false they are never split. Defaults to true.
         */
        explicit SplitOperation(
            MeshElementManager<DIM>& mesh_element_manager,
            double limit_edge_length,
            bool allow_split_fixed_edges = true);

        /**
         * @brief Destroys the SplitOperation.
         * @details Destroys the bound "locked" facet-corner attribute if it is still bound,
         *          releasing the underlying mesh attribute storage.
         */
        ~SplitOperation();

        /**
         * @brief Pops the next edge from the split queue and splits it if possible.
         * @details Reads the longest pending edge from the priority queue. A stale entry (whose
         *          facet timestamp changed since it was inserted) is re-enqueued with the current
         *          timestamp when @p iteratively is true. When fixed-edge splitting is
         *          disallowed, an edge whose facet contains a longer locked edge is itself locked
         *          to prevent infinite loops. The edge is skipped when it no longer passes
         *          is_perform_valid(); otherwise it is split via perform(), followed by
         *          post_process() bookkeeping and a post_check() assertion. The split halves are
         *          re-enqueued with updated timestamps.
         * @param[in] iteratively When true, stale and newly created edges are re-enqueued so
         *                        later calls can re-examine them; when false each edge is
         *                        processed at most once.
         * @return true while the queue is non-empty (more work may be pending); false once the
         *         queue is empty.
         */
        bool do_once(bool iteratively = true);

        /**
         * @brief Runs the edge-split queue to exhaustion over the whole mesh.
         * @details Repeatedly calls do_once(iteratively) until the priority queue is empty,
         *          splitting every eligible edge in order of decreasing length. When
         *          @p iteratively is false, stale edges are not re-enqueued and each edge is
         *          considered at most once.
         * @param[in] iteratively When true, affected edges are re-examined after each split;
         *                        when false a single sweep over the initial queue is performed.
         */
        void run_through(bool iteratively = true);

    private:
        /** @brief A pending edge-split candidate in the priority queue. */
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
         * @details Marks a boundary-split vertex as a boundary vertex, and when the original
         *          edge was tracked as fixed in the manager it is replaced by two fixed edges
         *          incident to the newly created vertex so the same fixed-edge constraints remain
         *          valid after the split.
         * @param[in] f Index of the facet that contained the split edge.
         * @param[in] lv Local vertex index that identified the split edge.
         * @param[in] new_v Index of the newly created vertex.
         * @param[in] original_ev0 First endpoint of the original split edge.
         * @param[in] original_ev1 Second endpoint of the original split edge.
         */
        void post_process(GEO::index_t f, GEO::index_t lv,
                          GEO::index_t new_v,
                          GEO::index_t original_ev0, GEO::index_t original_ev1);

        const double limit_edge_length_;

        const bool ALLOW_SPLIT_FIXED_EDGES_; // When true, splitting of fixed (locked) edges is allowed.

        std::priority_queue<EdgeToSplit> pq_;

        std::unordered_set<std::pair<GEO::index_t, GEO::index_t>, PairHash> locked_edges_; // temporary locked edge should not be split (only used when not allow to split fixed edges)
    };

    extern template class SplitOperation<2>;
    extern template class SplitOperation<3>;
}

#endif //GEOLIO_SPLIT_OPERATION_H
