//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "split_operation.h"
#include "geolio/tri_operations.h"

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

    bool SplitOperation::is_perform_valid(
        const GEO::index_t f,
        const GEO::index_t lv
        ) const {
        assert(f < mesh_.facets.nb());
        assert(lv < 3);

        if (!manager_.mesh_f_used[f]) // This facet should not yet exist.
            return false;

        if (const auto& fc = mesh_.facets.corner(f, lv);
            !manager_.ALLOW_SPLIT_FIXED_EDGES
            && manager_.mesh_fc_fixed[fc]) // Splitting fixed edges is not allowed.
            return false;

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

        const bool EDGE_ON_BOUNDARY = mesh_.facets.adjacent(f, lv) == GEO::NO_FACET;

        if (EDGE_ON_BOUNDARY) // Split edge inherits boundary attribute.
            manager_.mesh_v_boundary[new_v] = true;

        if (const auto& fc = mesh_.facets.corner(f, lv);
            manager_.mesh_fc_fixed[fc]
            ) { // Split edge inherits fixed attribute.
            const auto& new_f0_fc = mesh_.facets.corner(new_f0, lv);
            assert(mesh_.facet_corners.vertex(new_f0_fc) == new_v);
            manager_.mesh_fc_fixed[new_f0_fc] = true;

            if (!EDGE_ON_BOUNDARY) {
                assert(new_f1 != GEO::NO_FACET);
                const auto nlv = mesh_.facets.find_vertex(new_f1, new_v);
                assert(nlv != GEO::NO_INDEX);
                const auto& new_f1_fc = mesh_.facets.corner(new_f1, (nlv+2)%3);
                manager_.mesh_fc_fixed[new_f1_fc] = true;
            }
        }
    }
}
