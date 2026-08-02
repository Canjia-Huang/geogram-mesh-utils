//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "split_operation.h"
#include <geolio//mesh/tri_operations.h>

#include "geolio/common/log.h"

namespace geolio
{
    SplitOperation::SplitOperation(
        MeshElementManager& mesh_element_manager,
        const double limit_edge_length
        ) : BaseOperation(mesh_element_manager),
            limit_edge_length_(limit_edge_length)
    {}

    void SplitOperation::perform_one_pass(
        ) {
        mesh_f_timestamping_.fill(false);

        for (GEO::index_t f = 0, f_end = mesh_.facets.nb(); f < f_end; ++f) {
            if (mesh_f_timestamping_[f])
                continue;

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (!is_perform_valid(f, lv))
                    continue;

                GEO::index_t new_v, new_f0, new_f1;
                perform(f, lv, new_v, new_f0, new_f1);

                post_process(f, lv, new_v, new_f0, new_f1);

                assert(post_check());

                /* Label processed facets */
                mesh_f_timestamping_[f] = true;
                mesh_f_timestamping_[new_f0] = true;
                if (const auto& nf = mesh_.facets.adjacent(f, lv);
                    nf != GEO::NO_FACET
                    ) {
                    mesh_f_timestamping_[nf] = true;
                    assert(new_f1 != GEO::NO_FACET);
                    mesh_f_timestamping_[new_f1] = true;
                }
            }
        }
    }

    namespace
    {
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
    }

    void SplitOperation::perform_iteratively(
        ) {
        mesh_f_timestamping_.fill(0); // as version timestamping

        std::priority_queue<EdgeToSplit> pq;

        /* Init queue */
        {
            std::vector<bool> processed_edge(mesh_.facet_corners.nb(), false);
            for (const auto& f : mesh_.facets) {
                if (!manager_.mesh_f_used[f])
                    continue;

                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (const auto& fc = mesh_.facets.corner(f, lv);
                        processed_edge[fc])
                        continue;
                    else
                        processed_edge[fc] = true;

                    if (is_perform_valid(f, lv))
                        pq.emplace(f, lv, 0, manager_.get_edge_length(f, lv));

                    if (const auto& nf = mesh_.facets.adjacent(f, lv);
                        nf != GEO::NO_FACET
                        ) {
                        const auto& nlv = mesh_.facets.find_vertex(nf, mesh_.facets.vertex(f, lv));
                        assert(nlv != GEO::NO_INDEX);
                        processed_edge[mesh_.facets.corner(nf, (nlv+2)%3)] = true;
                    }
                }
            }
        }

        GEO::Attribute<bool> mesh_fc_locked; // locked edge should not be split (only used when not allow to split fixed edges)
        if (!ALLOW_SPLIT_FIXED_EDGES) {
            mesh_fc_locked.bind(mesh_.facet_corners.attributes(), attribute_name_+"locked");
            mesh_fc_locked.fill(false);
            for (const auto& fc : mesh_.facet_corners) {
                if (manager_.mesh_fc_fixed[fc])
                    mesh_fc_locked[fc] = true;
            }
        }

        /* Iteratively perform */
        while (!pq.empty()) {
            const auto edge = pq.top();
            pq.pop();

            /* Check validity */
            if (const auto& f_timestamping = mesh_f_timestamping_[edge.f];
                edge.timestamping < f_timestamping) { // This edge is not up-to-date. Push again.
                pq.emplace(edge.f, edge.lv, f_timestamping, manager_.get_edge_length(edge.f, edge.lv));
                continue;
            }
            assert(std::abs(edge.length - manager_.get_edge_length(edge.f, edge.lv)) < 1e-10); // because at least one adj_facet is unchange

            /* In the current triangle, if there exist a longer edge that cannot be split; splitting is prohibited,
             * otherwise it will cause an infinite loop. */
            if (!ALLOW_SPLIT_FIXED_EDGES) {
                assert(mesh_fc_locked.is_bound());

                const auto& cur_fc = mesh_.facets.corner(edge.f, edge.lv);
                for (const auto llv : {(edge.lv+1)%3, (edge.lv+2)%3}) {
                    if (const auto& fc = mesh_.facets.corner(edge.f, llv);
                        mesh_fc_locked[fc] &&
                        manager_.get_edge_length(edge.f, llv) > edge.length
                        ) {
                        mesh_fc_locked[cur_fc] = true;
                        break;
                    }
                }
                if (mesh_fc_locked[cur_fc])
                    continue;

                if (const auto& nf = mesh_.facets.adjacent(edge.f, edge.lv);
                    nf != GEO::NO_FACET) {
                    for (GEO::index_t llv = 0; llv < 3; ++llv) {
                        if (mesh_.facets.adjacent(nf, llv) == edge.f)
                            continue;
                        if (const auto& fc = mesh_.facets.corner(nf, llv);
                            mesh_fc_locked[fc] &&
                            manager_.get_edge_length(nf, llv) > edge.length
                            ) {
                            mesh_fc_locked[cur_fc] = true;
                            break;
                        }
                    }
                }
                if (mesh_fc_locked[cur_fc])
                    continue;
            }

            if (!is_perform_valid(edge.f, edge.lv))
                continue;

            /* Split */
            GEO::index_t new_v, new_f0, new_f1;
            perform(edge.f, edge.lv, new_v, new_f0, new_f1);

            post_process(edge.f, edge.lv, new_v, new_f0, new_f1);

            assert(post_check());

            /* Push new sub-edges */
            auto& f_timestamping = mesh_f_timestamping_[edge.f]; // mesh_f_timestamping_ may re-allocated before
            ++f_timestamping;
            pq.emplace(edge.f, edge.lv, f_timestamping, 0.5*edge.length);
            {
                auto& new_f0_timestamping = mesh_f_timestamping_[new_f0];
                new_f0_timestamping = f_timestamping;
                const auto lv = mesh_.facets.find_vertex(new_f0, new_v);
                assert(lv != GEO::NO_INDEX);
                pq.emplace(new_f0, lv, new_f0_timestamping, 0.5*edge.length);
                pq.emplace(new_f0, (lv+1)%3, new_f0_timestamping, manager_.get_edge_length(new_f0, (lv+1)%3));
                pq.emplace(new_f0, (lv+2)%3, new_f0_timestamping, manager_.get_edge_length(new_f0, (lv+2)%3));
            }
            if (const auto& nf = mesh_.facets.adjacent(edge.f, edge.lv);
                nf != GEO::NO_FACET
                ) {
                assert(new_f1 != GEO::NO_FACET);

                auto& nf_timestamping = mesh_f_timestamping_[nf];
                ++nf_timestamping;
                auto& new_f1_timestamping = mesh_f_timestamping_[new_f1];
                new_f1_timestamping = nf_timestamping;

                const auto lv = mesh_.facets.find_vertex(new_f1, new_v);
                assert(lv != GEO::NO_INDEX);
                pq.emplace(new_f1, lv, new_f1_timestamping, manager_.get_edge_length(new_f1, lv));
                pq.emplace(new_f1, (lv+1)%3, new_f1_timestamping, manager_.get_edge_length(new_f1, (lv+1)%3));
            }
        }

        /* Destroy temporary attribute. */
        if (!ALLOW_SPLIT_FIXED_EDGES) {
            assert(mesh_fc_locked.is_bound());
            mesh_fc_locked.destroy();
        }
    }

    bool SplitOperation::is_perform_valid(
        const GEO::index_t f,
        const GEO::index_t lv
        ) const {
        assert(f < mesh_.facets.nb());
        assert(lv < 3);

        if (!manager_.mesh_f_used[f]) // This facet should not yet exist.
            return false;

        if (!ALLOW_SPLIT_FIXED_EDGES) {
            if (const auto& fc = mesh_.facets.corner(f, lv);
                manager_.mesh_fc_fixed[fc]) // Splitting fixed edges is not allowed.
                return false;
        }

        if (const auto edge_length = manager_.get_edge_length(f, lv);
            edge_length < limit_edge_length_) // Do not split edges lesser than the limit length.
            return false;

        return true;
    }

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
