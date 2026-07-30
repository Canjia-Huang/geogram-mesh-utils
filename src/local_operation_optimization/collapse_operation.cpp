//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "collapse_operation.h"
#include <cassert>
#include <geogram/mesh/mesh_io.h>
#include "geolio/log.h"
#include "geolio/tri_operations.h"

namespace geolio
{
    CollapseOperation::CollapseOperation(
        MeshElementManager& mesh_element_manager,
        const double limit_edge_length
        ) : BaseOperation(mesh_element_manager),
            limit_edge_length_(limit_edge_length)
    {}

    void CollapseOperation::perform_one_pass(
        ) {
        mesh_f_processed_.fill(false);

        for (GEO::index_t f = 0, f_end = mesh_.facets.nb(); f < f_end; ++f) {
            if (mesh_f_processed_[f])
                continue;

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (!is_perform_valid(f, lv))
                    continue;

                const auto v = mesh_.facets.vertex(f, lv);

                GEO::index_t disuse_v0, disuse_v1, disuse_v2, disuse_f0, disuse_f1;
                perform(f, lv, disuse_v0, disuse_v1, disuse_v2, disuse_f0, disuse_f1);

                post_process(v, disuse_v0, disuse_v1, disuse_v2, disuse_f0, disuse_f1);

                assert(post_check());
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

        const auto ev0 = mesh_.facets.vertex(f, lv);
        const auto ev1 = mesh_.facets.vertex(f, (lv+1)%3);
        if (!manager_.mesh_v_fixed[ev1]) /* Collapse pulls v1 toward v0, no operation is performed when v1
                        is fixed, so that the vertex indices remain unchanged. */
            return false;

        if (manager_.ALLOW_COLLAPSE_FIXED_EDGES) {
            // TODO: Sliding along collinear edges is permitted.
        }
        else if (const auto& fc = mesh_.facets.corner(f, lv);
                manager_.mesh_fc_fixed[fc]) // Do not collapse the fixed edge.
            return false;

        if (const auto nf = mesh_.facets.adjacent(f, lv);
            manager_.mesh_v_boundary[ev0] && manager_.mesh_v_boundary[ev1]
            && nf != GEO::NO_FACET) // Collapse will produce non-manifold vertices.
            return false;

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
            manager_.mesh_v_fixed[ev0])
            R = 0; // pull ev1 -> ev0

        tri_edge_collapse(mesh_, f, lv, disuse_v0, disuse_f0, disuse_f1, R);
    }

    void CollapseOperation::post_process(
        const GEO::index_t v,
        const GEO::index_t disuse_v0,
        const GEO::index_t disuse_v1,
        const GEO::index_t disuse_v2,
        const GEO::index_t disuse_f0,
        const GEO::index_t disuse_f1
        ) const {
        assert(v < mesh_.vertices.nb());

        if (manager_.mesh_v_boundary[disuse_v0]) // After edge collapse on a boundary vertex, it remains a boundary vertex.
            manager_.mesh_v_boundary[v] = true;

        /* Disuse elements */
        assert(disuse_v0 != GEO::NO_VERTEX);
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
