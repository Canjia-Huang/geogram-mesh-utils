//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "split_operation.h"
#include <geolio//mesh/tri_operations.h>

#include "geolio/common/log.h"

namespace geolio
{
    template <GEO::index_t DIM>
    SplitOperation<DIM>::SplitOperation(
        MeshElementManager<DIM>& mesh_element_manager,
        const double limit_edge_length,
        const bool allow_split_fixed_edges
        ) : BaseOperation<DIM>(mesh_element_manager),
            limit_edge_length_(limit_edge_length),
            ALLOW_SPLIT_FIXED_EDGES_(allow_split_fixed_edges)
    {
        /* Bind attribute */
        if (!ALLOW_SPLIT_FIXED_EDGES_) {
            mesh_fc_locked_.bind(this->mesh_.facet_corners.attributes(), this->attribute_name_+"locked");
            mesh_fc_locked_.fill(false);
            for (const auto& fc : this->mesh_.facet_corners) {
                if (this->manager_.mesh_fc_fixed[fc])
                    mesh_fc_locked_[fc] = true;
            }
        }

        /* Init timestamping */
        this->mesh_f_timestamping_.fill(0);

        /* Init queue */
        auto emplace_to_pq = [&](const GEO::index_t f, const GEO::index_t lv) {
            if (is_perform_valid(f, lv))
                pq_.emplace(f, lv, 0, this->manager_.get_edge_length(f, lv));
        };
        this->for_each_edge(emplace_to_pq);
    }

    template <GEO::index_t DIM>
    SplitOperation<DIM>::~SplitOperation(
        ) {
        /* Destroy attributes */
        if (mesh_fc_locked_.is_bound())
            mesh_fc_locked_.destroy();
    }

    template <GEO::index_t DIM>
    bool SplitOperation<DIM>::do_once(
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
        assert(std::abs(edge.length - this->manager_.get_edge_length(edge.f, edge.lv)) < 1e-10); // because at least one adj_facet is unchange

        /* In the current triangle, if there exist a longer edge that cannot be split; splitting is prohibited,
             * otherwise it will cause an infinite loop. */
        if (!ALLOW_SPLIT_FIXED_EDGES_) {
            assert(mesh_fc_locked_.is_bound());

            const auto& cur_fc = this->mesh_.facets.corner(edge.f, edge.lv);
            for (const auto llv : {(edge.lv+1)%3, (edge.lv+2)%3}) {
                if (const auto& fc = this->mesh_.facets.corner(edge.f, llv);
                    mesh_fc_locked_[fc] &&
                    this->manager_.get_edge_length(edge.f, llv) > edge.length
                    ) {
                    mesh_fc_locked_[cur_fc] = true;
                    break;
                }
            }
            if (mesh_fc_locked_[cur_fc])
                return true;

            if (const auto& nf = this->mesh_.facets.adjacent(edge.f, edge.lv);
                nf != GEO::NO_FACET) {
                for (GEO::index_t llv = 0; llv < 3; ++llv) {
                    if (this->mesh_.facets.adjacent(nf, llv) == edge.f)
                        continue;
                    if (const auto& fc = this->mesh_.facets.corner(nf, llv);
                        mesh_fc_locked_[fc] &&
                        this->manager_.get_edge_length(nf, llv) > edge.length
                        ) {
                        mesh_fc_locked_[cur_fc] = true;
                        break;
                    }
                }
            }
            if (mesh_fc_locked_[cur_fc])
                return true;
        }

        /* Check validity */
        if (!is_perform_valid(edge.f, edge.lv))
            return true;

        /* Split */
        GEO::index_t new_v, new_f0, new_f1;
        perform(edge.f, edge.lv, new_v, new_f0, new_f1);

        post_process(edge.f, edge.lv, new_v, new_f0, new_f1);

        assert(this->post_check());

        /* Push new sub-edges */
        {
            auto& f_timestamping = this->mesh_f_timestamping_[edge.f]; // mesh_f_timestamping_ may re-allocated before
            ++f_timestamping;
            auto& new_f0_timestamping = this->mesh_f_timestamping_[new_f0];
            new_f0_timestamping = f_timestamping;

            if (iteratively) {
                pq_.emplace(edge.f, edge.lv, f_timestamping, 0.5*edge.length);

                const auto lv = this->mesh_.facets.find_vertex(new_f0, new_v);
                assert(lv != GEO::NO_INDEX);
                pq_.emplace(new_f0, lv, new_f0_timestamping, 0.5*edge.length);
                pq_.emplace(new_f0, (lv+1)%3, new_f0_timestamping, this->manager_.get_edge_length(new_f0, (lv+1)%3));
                pq_.emplace(new_f0, (lv+2)%3, new_f0_timestamping, this->manager_.get_edge_length(new_f0, (lv+2)%3));
            }
        }

        if (const auto& nf = this->mesh_.facets.adjacent(edge.f, edge.lv);
            nf != GEO::NO_FACET
            ) {
            assert(new_f1 != GEO::NO_FACET);

            auto& nf_timestamping = this->mesh_f_timestamping_[nf];
            ++nf_timestamping;
            auto& new_f1_timestamping = this->mesh_f_timestamping_[new_f1];
            new_f1_timestamping = nf_timestamping;

            if (iteratively) {
                const auto lv = this->mesh_.facets.find_vertex(new_f1, new_v);
                assert(lv != GEO::NO_INDEX);
                pq_.emplace(new_f1, lv, new_f1_timestamping, this->manager_.get_edge_length(new_f1, lv));
                pq_.emplace(new_f1, (lv+1)%3, new_f1_timestamping, this->manager_.get_edge_length(new_f1, (lv+1)%3));
            }
        }

        return true;
    }

    template <GEO::index_t DIM>
    void SplitOperation<DIM>::run_through(
        const bool iteratively
        ) {
        while (do_once(iteratively)) {}
    }

    template <GEO::index_t DIM>
    bool SplitOperation<DIM>::is_perform_valid(
        const GEO::index_t f,
        const GEO::index_t lv
        ) const {
        assert(f < this->mesh_.facets.nb());
        assert(lv < 3);

        if (!this->manager_.mesh_f_used[f]) // This facet should not yet exist.
            return false;

        if (!ALLOW_SPLIT_FIXED_EDGES_) {
            if (const auto& fc = this->mesh_.facets.corner(f, lv);
                this->manager_.mesh_fc_fixed[fc]) // Splitting fixed edges is not allowed.
                return false;
        }

        if (const auto edge_length = this->manager_.get_edge_length(f, lv);
            edge_length < limit_edge_length_) // Do not split edges lesser than the limit length.
            return false;

        return true;
    }

    template <GEO::index_t DIM>
    void SplitOperation<DIM>::perform(
        const GEO::index_t f,
        const GEO::index_t lv,
        GEO::index_t& new_v,
        GEO::index_t& new_f0,
        GEO::index_t& new_f1
        ) const {
        assert(f < this->mesh_.facets.nb());
        assert(lv < 3);

        const bool EDGE_ON_BOUNDARY = this->mesh_.facets.adjacent(f, lv) == GEO::NO_FACET;

        /* Split */
        new_v = this->manager_.require_new_vertex();
        new_f0 = this->manager_.require_new_facet();
        new_f1 = EDGE_ON_BOUNDARY ? GEO::NO_FACET : this->manager_.require_new_facet();
        tri_edge_split<DIM>(this->mesh_, f, lv, new_v, new_f0, new_f1);
    }

    template <GEO::index_t DIM>
    void SplitOperation<DIM>::post_process(
        const GEO::index_t f,
        const GEO::index_t lv,
        const GEO::index_t new_v,
        const GEO::index_t new_f0,
        const GEO::index_t new_f1
        ) const {
        assert(f < this->mesh_.facets.nb());
        assert(lv < 3);
        assert(new_v < this->mesh_.vertices.nb());
        assert(new_f0 < this->mesh_.facets.nb());

        const auto nf = this->mesh_.facets.adjacent(f, lv);
        const bool EDGE_ON_BOUNDARY = (nf == GEO::NO_FACET);

        if (EDGE_ON_BOUNDARY) // Split edge inherits boundary attribute.
            this->manager_.mesh_v_boundary[new_v] = true;
    }

    template class SplitOperation<2>;
    template class SplitOperation<3>;
}
