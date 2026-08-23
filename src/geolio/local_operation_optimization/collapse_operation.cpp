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
    template<GEO::index_t DIM>
    CollapseOperation<DIM>::CollapseOperation(
        MeshElementManager<DIM>& mesh_element_manager,
        const double limit_edge_length,
        const bool allow_collapse_fixed_edges
        ) : BaseOperation<DIM>(mesh_element_manager),
            limit_edge_length_(limit_edge_length),
            ALLOW_COLLAPSE_FIXED_EDGES_(allow_collapse_fixed_edges)
    {
        /* Init timestamping */
        this->mesh_f_timestamping_.fill(0);

        /* Init queue */
        auto emplace_to_pq = [&](const GEO::index_t f, const GEO::index_t lv) {
            if (is_perform_valid(f, lv))
                pq_.emplace(f, lv, 0, this->manager_.get_edge_length(f, lv));
        };
        this->for_each_edge(emplace_to_pq);
    }

    template<GEO::index_t DIM>
    bool CollapseOperation<DIM>::do_once(
        const bool iteratively
        ) {
        if (pq_.empty())
            return false;

        const auto edge = pq_.top();
        pq_.pop();

        /* Check validity */
        if (const auto& f_timestamping = this->mesh_f_timestamping_[edge.f];
            edge.timestamping < f_timestamping
            ) { // This edge is not up-to-date. Push again.
            if (iteratively)
                pq_.emplace(edge.f, edge.lv, f_timestamping, this->manager_.get_edge_length(edge.f, edge.lv));
            return true;
        }

        if (!is_perform_valid(edge.f, edge.lv))
            return true;

        /* Collapse */
        get_vertex_incident_facets(this->mesh_, edge.f, edge.lv,       ordered_f_and_lv_0_); // before collapse
        get_vertex_incident_facets(this->mesh_, edge.f, (edge.lv+1)%3, ordered_f_and_lv_1_); // before collapse

        GEO::index_t disuse_v0, disuse_v1, disuse_v2, disuse_f0, disuse_f1;
        perform(edge.f, edge.lv, disuse_v0, disuse_v1, disuse_v2, disuse_f0, disuse_f1);

        post_process(edge.f, edge.lv, disuse_v0, disuse_v1, disuse_v2, disuse_f0, disuse_f1);

        assert(this->post_check());

        /* Push new sub-edges */
        for (const auto& [f, lv] : ordered_f_and_lv_0_) {
            if (f == disuse_f0 || f == disuse_f1)
                continue;

            auto& f_timestamping = this->mesh_f_timestamping_[f];
            ++f_timestamping;

            if (iteratively)
                pq_.emplace(f, lv, f_timestamping, this->manager_.get_edge_length(f, lv));
        }
        for (const auto& [f, lv] : ordered_f_and_lv_1_) {
            if (f == disuse_f0 || f == disuse_f1)
                continue;

            auto& f_timestamping = this->mesh_f_timestamping_[f];
            ++f_timestamping;

            if (iteratively)
                pq_.emplace(f, lv, f_timestamping, this->manager_.get_edge_length(f, lv));
        }

        return true;
    }

    template<GEO::index_t DIM>
    void CollapseOperation<DIM>::run_through(
        const bool iteratively
        ) {
        while (do_once(iteratively)) {}
    }

    template<GEO::index_t DIM>
    bool CollapseOperation<DIM>::is_perform_valid(
        const GEO::index_t f,
        const GEO::index_t lv
        ) const {
        assert(f < this->mesh_.facets.nb());
        assert(lv < 3);

        if (!this->manager_.mesh_f_used[f]) // This facet should not yet exist.
            return false;

        const auto nf = this->mesh_.facets.adjacent(f, lv);
        // const bool EDGE_ON_BOUNDARY = (nf == GEO::NO_FACET);
        const auto ev0 = this->mesh_.facets.vertex(f, lv);
        const auto ev1 = this->mesh_.facets.vertex(f, (lv+1)%3);

        /* Non‑manifold vertices might not collect all adjacent facets, which can lead to errors; hence fixed */
        if (this->manager_.mesh_v_non_manifold[ev0] ||
            this->manager_.mesh_v_non_manifold[ev1])
            return false;

        /* Do not collapse the fixed edge. */
        if (!ALLOW_COLLAPSE_FIXED_EDGES_) {
            if (const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                    this->mesh_.facets.vertex(f, lv),
                    this->mesh_.facets.vertex(f, (lv+1)%3));
                this->manager_.fixed_edges_.contains(edge))
                return false;
        }

        /* Do not collapse edges greater than the limit length. */
        if (const auto edge_length = this->manager_.get_edge_length(f, lv);
            edge_length > limit_edge_length_)
            return false;

        /* Collapse pulls v1 toward v0, no operation is performed when v1 is fixed, so that the vertex indices remain unchanged. */
        if (this->manager_.mesh_v_fixed[ev1])
            return false;
        { // The fixed edge involving ev1 also prevents collapse (because it would remove ev1).
            std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
            get_vertex_incident_facets(this->mesh_, f, (lv+1)%3, ordered_f_and_lv);
            for (const auto& [ff, llv] : ordered_f_and_lv) {
                if (const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                        this->mesh_.facets.vertex(ff, llv),
                        this->mesh_.facets.vertex(ff, (llv+1)%3));
                    this->manager_.fixed_edges_.contains(edge))
                    return false;
            }
        }

        if (this->mesh_.facets.adjacent(f, 0) == GEO::NO_FACET &&
            this->mesh_.facets.adjacent(f, 1) == GEO::NO_FACET &&
            this->mesh_.facets.adjacent(f, 2) == GEO::NO_FACET
            ) { // Isolated facet: collapse removes the facet and all three of its vertices.
                // A vertex shared by other facets (non-manifold) or marked fixed must not be disposed.
            for (GEO::index_t llv = 0; llv < 3; ++llv) {
                const auto& v = this->mesh_.facets.vertex(f, llv);
                if (this->manager_.mesh_v_fixed[v] ||
                    this->manager_.mesh_v_non_manifold[v])
                    return false;
            }
        }

        /* Collapse operation is not valid. */
        if (!is_tri_edge_collapse_valid(this->mesh_, f, lv))
            return false;

        return true;
    }

    template<GEO::index_t DIM>
    void CollapseOperation<DIM>::perform(
        const GEO::index_t f,
        const GEO::index_t lv,
        GEO::index_t& disuse_v0,
        GEO::index_t& disuse_v1,
        GEO::index_t& disuse_v2,
        GEO::index_t& disuse_f0,
        GEO::index_t& disuse_f1
        ) const {
        assert(f < this->mesh_.facets.nb());
        assert(lv < 3);

        const auto ev0 = this->mesh_.facets.vertex(f, lv);
        const auto ep0 = this->mesh_.vertices.template point<DIM>(ev0);

        disuse_v0 = GEO::NO_VERTEX;
        disuse_v1 = GEO::NO_VERTEX;
        disuse_v2 = GEO::NO_VERTEX;
        disuse_f0 = GEO::NO_FACET;
        disuse_f1 = GEO::NO_FACET;

        if (this->mesh_.facets.adjacent(f, 0) == GEO::NO_FACET &&
            this->mesh_.facets.adjacent(f, 1) == GEO::NO_FACET &&
            this->mesh_.facets.adjacent(f, 2) == GEO::NO_FACET
            ) { // For an isolated facet, collapse will directly remove it.
            disuse_v0 = this->mesh_.facets.vertex(f, 0);
            disuse_v1 = this->mesh_.facets.vertex(f, 0);
            disuse_v2 = this->mesh_.facets.vertex(f, 0);
            disuse_f0 = f;
            return;
        }

        bool pull_ev1_to_ev0 = false;
        if (this->manager_.mesh_v_fixed[ev0]) // pull ev1 -> ev0
            pull_ev1_to_ev0 = true;
        { // The fixed edge involving ev0 also pull ev1 -> ev0.
            std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
            get_vertex_incident_facets(this->mesh_, f, lv, ordered_f_and_lv);
            for (const auto& [ff, llv] : ordered_f_and_lv) {
                if (const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                        this->mesh_.facets.vertex(ff, llv),
                        this->mesh_.facets.vertex(ff, (llv+1)%3));
                    this->manager_.fixed_edges_.contains(edge)) {
                    pull_ev1_to_ev0 = true;
                    break;
                }
            }
        }

        tri_edge_collapse<DIM>(this->mesh_, f, lv, disuse_v0, disuse_f0, disuse_f1);

        if (pull_ev1_to_ev0)
            this->mesh_.vertices.template point<DIM>(ev0) = ep0;
    }

    template<GEO::index_t DIM>
    void CollapseOperation<DIM>::post_process(
        const GEO::index_t f,
        const GEO::index_t lv,
        const GEO::index_t disuse_v0,
        const GEO::index_t disuse_v1,
        const GEO::index_t disuse_v2,
        const GEO::index_t disuse_f0,
        const GEO::index_t disuse_f1
        ) const {
        assert(f < this->mesh_.facets.nb());
        assert(lv < 3);
        assert(disuse_v0 != GEO::NO_VERTEX);
        assert(disuse_f0 != GEO::NO_FACET);

        /* After edge collapse on a boundary vertex, it remains a boundary vertex. */
        const auto v0 = this->mesh_.facets.vertex(f, lv);
        if (this->manager_.mesh_v_boundary[disuse_v0])
            this->manager_.mesh_v_boundary[v0] = true;

        /* Update fixed edges */
        {
            // do not need to
        }

        /* Disuse elements */
        this->manager_.disuse_vertex(disuse_v0);
        if (disuse_v1 != GEO::NO_VERTEX)
            this->manager_.disuse_vertex(disuse_v1);
        if (disuse_v2 != GEO::NO_VERTEX)
            this->manager_.disuse_vertex(disuse_v2);
        assert(disuse_f0 != GEO::NO_FACET);
        this->manager_.disuse_facet(disuse_f0);
        if (disuse_f1 != GEO::NO_FACET)
            this->manager_.disuse_facet(disuse_f1);
    }

    template class CollapseOperation<2>;
    template class CollapseOperation<3>;
}
