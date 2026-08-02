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
    CollapseOperation::CollapseOperation(
        MeshElementManager& mesh_element_manager,
        const double limit_edge_length
        ) : BaseOperation(mesh_element_manager),
            limit_edge_length_(limit_edge_length)
    {}

    void CollapseOperation::sweep_mesh(
        ) {
        mesh_f_timestamping_.fill(false);

        for (GEO::index_t f = 0, f_end = mesh_.facets.nb(); f < f_end; ++f) {
            if (mesh_f_timestamping_[f])
                continue;

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (!is_perform_valid(f, lv))
                    continue;

                GEO::index_t disuse_v0, disuse_v1, disuse_v2, disuse_f0, disuse_f1;
                perform(f, lv, disuse_v0, disuse_v1, disuse_v2, disuse_f0, disuse_f1);

                post_process(f, lv, disuse_v0, disuse_v1, disuse_v2, disuse_f0, disuse_f1);

                assert(post_check());
            }
        }
    }

    namespace
    {
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
    }

    void CollapseOperation::run_iterative_loop(
        ) {
        mesh_f_timestamping_.fill(0); // as version timestamping

        std::priority_queue<EdgeToCollapse> pq;

        /* Init queue */
        auto init_func = [&](const GEO::index_t f, const GEO::index_t lv) {
            if (is_perform_valid(f, lv))
                pq.emplace(f, lv, 0, manager_.get_edge_length(f, lv));
        };
        for_each_edge(init_func);

        /* Iteratively perform */
        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv_0; // just pre-allocated
        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv_1; // just pre-allocated
        while (!pq.empty()) {
            const auto edge = pq.top();
            pq.pop();

            /* Check validity */
            if (const auto& f_timestamping = mesh_f_timestamping_[edge.f];
                edge.timestamping < f_timestamping) { // This edge is not up-to-date. Push again.
                pq.emplace(edge.f, edge.lv, f_timestamping, manager_.get_edge_length(edge.f, edge.lv));
                continue;
            }

            if (!is_perform_valid(edge.f, edge.lv))
                continue;

            /* Collapse */
            get_vertex_incident_facets(mesh_, edge.f, edge.lv, ordered_f_and_lv_0); // before collapse
            get_vertex_incident_facets(mesh_, edge.f, (edge.lv+1)%3, ordered_f_and_lv_1); // before collapse

            GEO::index_t disuse_v0, disuse_v1, disuse_v2, disuse_f0, disuse_f1;
            perform(edge.f, edge.lv, disuse_v0, disuse_v1, disuse_v2, disuse_f0, disuse_f1);

            post_process(edge.f, edge.lv, disuse_v0, disuse_v1, disuse_v2, disuse_f0, disuse_f1);

            assert(post_check());

            /* Push new sub-edges */
            for (const auto& [f, lv] : ordered_f_and_lv_0) {
                if (f == disuse_f0 || f == disuse_f1)
                    continue;
                auto& f_timestamping = mesh_f_timestamping_[f];
                ++f_timestamping;
                pq.emplace(f, lv, f_timestamping, manager_.get_edge_length(f, lv));
            }
            for (const auto& [f, lv] : ordered_f_and_lv_1) {
                if (f == disuse_f0 || f == disuse_f1)
                    continue;
                auto& f_timestamping = mesh_f_timestamping_[f];
                ++f_timestamping;
                pq.emplace(f, lv, f_timestamping, manager_.get_edge_length(f, lv));
            }
        }
    }

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

        if (!ALLOW_COLLAPSE_FIXED_EDGES) {
            if (const auto& fc = mesh_.facets.corner(f, lv);
                manager_.mesh_fc_fixed[fc]) // Do not collapse the fixed edge.
                    return false;
        }

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
