//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_SWAP_OPERATION_H
#define GEOLIO_SWAP_OPERATION_H

#include "base_operation.h"

namespace geolio
{
    template<GEO::index_t DIM>
    class SwapOperation : public BaseOperation<DIM> {
    public:
        /**
         * @brief Bit flags selecting which validity criterion a candidate edge must satisfy.
         * @details SWAP_BASED_ON_VALENCE accepts a swap only when it decreases the sum of
         *          squared deviations from the ideal valence (6 interior, 4 boundary);
         *          SWAP_BASED_ON_DELAUNAY accepts a swap only when the edge is not locally
         *          Delaunay (the sum of the two opposite cotangent angles is non-negative).
         */
        enum SwapCriterion {
            SWAP_BASED_ON_VALENCE       = 1<<0,
            SWAP_BASED_ON_DELAUNAY      = 1<<1
        };

        /**
         * @brief Constructs a SwapOperation for flipping edges to improve vertex valence.
         * @details Initializes the base operation, stores the swap criterion, and pre-populates
         *          the swap queue with every currently eligible edge; no additional state is
         *          required because swapping only rewires existing elements.
         * @param[in] mesh_element_manager The mesh element manager exposing the mesh and its
         *                                 usage/fixed element attributes.
         * @param[in] swap_criterion Bitwise combination of SwapCriterion flags guiding which
         *                           edges are accepted. Defaults to SWAP_BASED_ON_DELAUNAY.
         */
        explicit SwapOperation(
            MeshElementManager<DIM>& mesh_element_manager,
            GEO::index_t swap_criterion = SWAP_BASED_ON_DELAUNAY);

        /**
         * @brief Pops the next edge from the swap queue and swaps it if possible.
         * @details Reads the next pending edge from the queue. A stale entry (whose facet
         *          timestamp changed since it was inserted) is re-enqueued with the current
         *          timestamp when @p iteratively is true. The edge is skipped when it no longer
         *          passes is_perform_valid(); otherwise it is swapped via perform(), followed by
         *          post_process() bookkeeping and a post_check() assertion. The re-wired edges of
         *          both incident facets are re-enqueued with updated timestamps.
         * @param[in] iteratively When true, stale and newly affected edges are re-enqueued so
         *                        later calls can re-examine them; when false each edge is
         *                        processed at most once.
         * @return true while the queue is non-empty (more work may be pending); false once the
         *         queue is empty.
         */
        bool do_once(bool iteratively = true);

        /**
         * @brief Runs the edge-swap queue to exhaustion over the whole mesh.
         * @details Repeatedly calls do_once(iteratively) until the queue is empty, swapping
         *          every eligible edge. When @p iteratively is false, stale edges are not
         *          re-enqueued and each edge is considered at most once.
         * @param[in] iteratively When true, affected edges are re-examined after each swap; when
         *                        false a single sweep over the initial queue is performed.
         */
        void run_through(bool iteratively = true);

    private:
        /** @brief A pending edge-swap candidate in the swap queue. */
        struct EdgeToSwap {
            EdgeToSwap(
                const GEO::index_t _f,
                const GEO::index_t _lv,
                const GEO::index_t _timestamping
            ) : f(_f), lv(_lv), timestamping(_timestamping)
            {}

            GEO::index_t f = GEO::NO_FACET;
            GEO::index_t lv = GEO::NO_INDEX;
            GEO::index_t timestamping = GEO::NO_INDEX;
        };

        /**
         * @brief Checks whether the edge of facet @p f at local vertex @p lv may be swapped.
         * @details Rejects edges whose facet is disused, fixed edges, boundary edges, edges with
         *          a non-manifold endpoint, and edges rejected by is_tri_edge_swap_valid().
         *          Additionally, computes the vertex valences of the four quad corners and only
         *          accepts the swap if it does not increase the sum of squared deviations from
         *          the ideal valences (6 interior, 4 boundary).
         * @param[in] f Index of the facet adjacent to the candidate edge.
         * @param[in] lv Local vertex index identifying the oriented edge (lv -> lv+1).
         * @return true if the edge may be swapped; false otherwise.
         */
        [[nodiscard]] bool is_perform_valid(GEO::index_t f, GEO::index_t lv) const;

        /**
         * @brief Performs the edge swap on the mesh topology.
         * @details Calls tri_edge_swap() to flip the shared diagonal of the two triangles
         *          adjacent to the edge, rewiring their connectivity in place.
         * @param[in] f Index of the facet adjacent to the edge to swap.
         * @param[in] lv Local vertex index identifying the oriented edge (lv -> lv+1).
         */
        void perform(GEO::index_t f, GEO::index_t lv) const;

        /**
         * @brief Applies post-swap bookkeeping to the manager's element attributes.
         * @details Restores the used flags of the two facets involved in the swap (they may have
         *          been cleared by the underlying operation) so the mesh state stays consistent.
         * @param[in] f Index of the first facet involved in the swap.
         * @param[in] lv Local vertex index that identified the swapped edge.
         * @param[in] nf Index of the adjacent facet involved in the swap.
         */
        void post_process(GEO::index_t f, GEO::index_t lv, GEO::index_t nf) const;

        const GEO::index_t SWAP_CRITERION_;

        std::vector<EdgeToSwap> pq_;
    };

    extern template class SwapOperation<2>;
    extern template class SwapOperation<3>;
}

#endif //GEOLIO_SWAP_OPERATION_H
