//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_SWAP_OPERATION_H
#define GEOLIO_SWAP_OPERATION_H

#include "base_operation.h"

namespace geolio
{
    class SwapOperation : public BaseOperation {
    public:
        /**
         * @brief Constructs a SwapOperation for flipping edges to improve vertex valence.
         * @details Initializes the base operation; no additional state is required because
         *          swapping only rewires existing elements.
         * @param[in] mesh_element_manager The mesh element manager exposing the mesh and its
         *                                 usage/fixed element attributes.
         */
        explicit SwapOperation(MeshElementManager& mesh_element_manager);

        /**
         * @brief Executes a single pass of edge-swapping over the whole mesh.
         * @details Resets the per-facet "processed" flags, then iterates over every facet and
         *          local edge; for each interior edge that passes is_perform_valid(), it swaps
         *          the edge, applies post_process() bookkeeping, asserts post_check(), and marks
         *          both incident facets as processed so they are not revisited in this pass.
         */
        void sweep_mesh();

        void run_iterative_loop();

        enum SwapCriterion {
            SWAP_BASED_ON_VALENCE       = 1<<0,
            SWAP_BASED_ON_DELAUNAY      = 1<<1
        };

        GEO::index_t SWAP_CRITERION = SWAP_BASED_ON_DELAUNAY;

    private:
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
    };
}

#endif //GEOLIO_SWAP_OPERATION_H
