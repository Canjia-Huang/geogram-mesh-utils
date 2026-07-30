//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "geolio/tri_local_operation_optimization.h"
#include <ranges>
#include <geogram/mesh/mesh_io.h>
#include "geolio/log.h"
#include "geolio/tri_operations.h"
#include "geolio/utils.h"
#include <unordered_set>
#include "geolio/detect_mesh_defects.h"

namespace geolio
{
    TriLocalOperationOptimization::TriLocalOperationOptimization(
        GEO::Mesh& mesh
        ) : mesh_(mesh),
            mesh_2d_(mesh.vertices.dimension() == 2),
            attribute_name_(generate_random_string(22))
    {
        assert(mesh.facets.are_simplices());

        bind_attributes();
        label_boundary_vertices();
        label_non_manifold_vertices();

        if (!mesh_2d_) {
            original_mesh_.copy(mesh_);
            original_mesh_facet_AABB_.initialize(original_mesh_);
        }
    }

    TriLocalOperationOptimization::~TriLocalOperationOptimization(
        ) {
        unbind_attributes();
    }

    void TriLocalOperationOptimization::optimize(
        double target_edge_length,
        GEO::index_t rounds_nb
        ) {
        LOG::TRACE(__FUNCTION__);

        /* Options */
        if (!ALLOW_SMOOTH_FIXED_EDGE_VERTICES_) { // == fix fixed edge's vertices
            for (const auto& f : mesh_.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (mesh_fc_fixed_[mesh_.facets.corner(f, lv)]) {
                        fix_vertex(mesh_.facets.vertex(f, lv));
                        fix_vertex(mesh_.facets.vertex(f, (lv+1)%3));
                    }
                }
            }
        }

        /* Compute target edge length */
        if (target_edge_length < 0) {
            target_edge_length = compute_average_mesh_edge_length();
            LOG::DEBUG("Automatically set the target edge length to the average edge length {}.", target_edge_length);
        }
        assert(target_edge_length > 0);
        const double SPLIT_EDGE_LENGTH = 4.0/3.0 * target_edge_length;
        const double COLLAPSE_EDGE_LENGTH = 4.0/5.0 * target_edge_length;

        /* Let's go! */
        for (GEO::index_t round = 0; round < rounds_nb; ++round) {
            LOG::TRACE("round: {}/{}", round+1, rounds_nb);

            const auto PREV_VERTICES_NB = mesh_.vertices.nb();
            const auto PREV_FACETS_NB = mesh_.facets.nb();

            split_edges(SPLIT_EDGE_LENGTH);
            collapse_edges(COLLAPSE_EDGE_LENGTH);
            swap_edges();
            smooth_vertices(1);

            LOG::DEBUG("#V: {} -> {}, #F: {} -> {}", PREV_VERTICES_NB, mesh_.vertices.nb(), PREV_FACETS_NB, mesh_.facets.nb());
        }

        /* Make output mesh valid */
        clean_unused_elements();
    }

    void TriLocalOperationOptimization::fix_boundary_edges(
        ) {
        LOG::TRACE(__FUNCTION__);
        assert(mesh_fc_fixed_.is_bound());

        for (const auto& f : mesh_.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (mesh_.facets.adjacent(f, lv) == GEO::NO_FACET)
                    fix_edge(f, lv);
            }
        }
    }

    void TriLocalOperationOptimization::bind_attributes(
        ) {
        LOG::TRACE(__FUNCTION__);

        mesh_v_boundary_.bind(mesh_.vertices.attributes(), attribute_name_+":boundary");
        mesh_v_boundary_.fill(false);
        mesh_v_fixed_.bind(mesh_.vertices.attributes(), attribute_name_+":fixed");
        mesh_v_fixed_.fill(false);
        mesh_v_non_manifold_.bind(mesh_.vertices.attributes(), attribute_name_+":non-manifold");
        mesh_v_non_manifold_.fill(false);
        mesh_v_used_.bind(mesh_.vertices.attributes(), attribute_name_+":used");
        mesh_v_used_.fill(true);

        mesh_f_processed_.bind(mesh_.facets.attributes(), attribute_name_+":processed");
        mesh_f_processed_.fill(false);
        mesh_f_used_.bind(mesh_.facets.attributes(), attribute_name_+":used");
        mesh_f_used_.fill(true);

        mesh_fc_fixed_.bind(mesh_.facet_corners.attributes(), attribute_name_+":fixed");
        mesh_fc_fixed_.fill(false);
    }

    void TriLocalOperationOptimization::unbind_attributes(
        ) {
        LOG::TRACE(__FUNCTION__);

        if (mesh_v_boundary_.is_bound())
            mesh_v_boundary_.destroy();
        if (mesh_v_fixed_.is_bound())
            mesh_v_fixed_.destroy();
        if (mesh_v_used_.is_bound());
            mesh_v_used_.destroy();

        if (mesh_f_processed_.is_bound())
            mesh_f_processed_.destroy();
        if (mesh_f_used_.is_bound())
            mesh_f_used_.destroy();

        if (mesh_fc_fixed_.is_bound())
            mesh_fc_fixed_.destroy();
    }

    void TriLocalOperationOptimization::label_boundary_vertices(
        ) {
        LOG::TRACE(__FUNCTION__);
        assert(mesh_v_boundary_.is_bound());

        for (const auto& f : mesh_.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (mesh_.facets.adjacent(f, lv) == GEO::NO_FACET) {
                    mesh_v_boundary_[mesh_.facets.vertex(f, lv)] = true;
                    mesh_v_boundary_[mesh_.facets.vertex(f, (lv+1)%3)] = true;
                }
            }
        }
    }

    void TriLocalOperationOptimization::label_non_manifold_vertices(
        ) {
        LOG::TRACE(__FUNCTION__);
        assert(mesh_v_non_manifold_.is_bound());
        assert(mesh_v_fixed_.is_bound());

        std::vector<GEO::index_t> non_manifold_vertices;
        const auto NON_MANIFOLD_VERTICES_NB = detect_non_manifold_vertices(mesh_, non_manifold_vertices);

        mesh_v_non_manifold_.fill(false);
        for (const auto& v : non_manifold_vertices) {
            mesh_v_non_manifold_[v] = true;
            fix_vertex(v); // fix it, prevent errors in collapse operations involving this vertex.
        }

        LOG::WARN("Detected {} non-manifold vertices in the input mesh; "
                  "they have been fixed to prevent unexpected errors.", NON_MANIFOLD_VERTICES_NB);
    }

    double TriLocalOperationOptimization::get_edge_length(
        const GEO::index_t f,
        const GEO::index_t lv
        ) const {
        assert(f < mesh_.facets.nb());
        assert(lv < 3);
        if (mesh_2d_)
            return GEO::distance(mesh_.facets.point<2>(f, lv), mesh_.facets.point<2>(f, (lv+1)%3));
        return GEO::distance(mesh_.facets.point(f, lv), mesh_.facets.point(f, (lv+1)%3));
    }

    double TriLocalOperationOptimization::compute_average_mesh_edge_length(
        ) const {
        LOG::TRACE(__FUNCTION__);

        double l = 0;
        GEO::index_t edges_nb = 0;
        for (const auto& f : mesh_.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                l += get_edge_length(f, lv);
                ++edges_nb;
            }
        }

        return l / edges_nb;
    }

    GEO::index_t TriLocalOperationOptimization::require_a_new_vertex(
        ) {
        if (free_vertices_.empty())
            allocate_new_vertices();
        assert(!free_vertices_.empty());

        const GEO::index_t new_v = free_vertices_.back();
        free_vertices_.pop_back();
        assert(new_v < mesh_.vertices.nb());
        mesh_v_used_[new_v] = true;

        return new_v;
    }

    void TriLocalOperationOptimization::disuse_a_vertex(
        const GEO::index_t v
        ) {
        /* Recycle */
        assert(v < mesh_.vertices.nb());
        free_vertices_.push_back(v);

        /* Init attributes */
        mesh_v_boundary_[v]     = false;
        mesh_v_fixed_[v]        = false;
        mesh_v_non_manifold_[v] = false;
        mesh_v_used_[v]         = false;
    }

    GEO::index_t TriLocalOperationOptimization::require_a_new_facet(
        ) {
        if (free_facets_.empty())
            allocate_new_facets();
        assert(!free_facets_.empty());

        const GEO::index_t new_f = free_facets_.back();
        free_facets_.pop_back();
        assert(new_f < mesh_.facets.nb());
        mesh_f_used_[new_f] = true;

        return new_f;
    }

    void TriLocalOperationOptimization::disuse_a_facet(
        const GEO::index_t f
        ) {
        /* Recycle */
        assert(f < mesh_.facets.nb());
        free_facets_.push_back(f);

        /* Init attributes */
        mesh_f_used_[f] = false;
        for (GEO::index_t lv = 0; lv < 3; ++lv)
            mesh_fc_fixed_[3*f+lv] = false;
    }

    void TriLocalOperationOptimization::allocate_new_vertices(
        ) {
        LOG::TRACE(__FUNCTION__);
        assert(mesh_.vertices.nb() > 0);

        const GEO::index_t PREV_MESH_VERTICES_NB = mesh_.vertices.nb();
        const GEO::index_t ALLOCATE_MESH_VERTICES_NB = PREV_MESH_VERTICES_NB;

        mesh_.vertices.create_vertices(ALLOCATE_MESH_VERTICES_NB);
        free_vertices_.reserve(free_vertices_.size() + ALLOCATE_MESH_VERTICES_NB);
        for (GEO::index_t v = PREV_MESH_VERTICES_NB, v_end = mesh_.vertices.nb(); v < v_end; ++v)
            free_vertices_.push_back(v);
    }

    void TriLocalOperationOptimization::allocate_new_facets(
        ) {
        LOG::TRACE(__FUNCTION__);
        assert(mesh_.facets.nb() > 0);

        const GEO::index_t PREV_MESH_FACETS_NB = mesh_.facets.nb();
        const GEO::index_t ALLOCATE_MESH_FACETS_NB = PREV_MESH_FACETS_NB;

        mesh_.facets.create_triangles(ALLOCATE_MESH_FACETS_NB);
        free_facets_.reserve(free_facets_.size() + ALLOCATE_MESH_FACETS_NB);
        for (GEO::index_t f = PREV_MESH_FACETS_NB, f_end = mesh_.facets.nb(); f < f_end; ++f)
            free_facets_.push_back(f);
    }

    void TriLocalOperationOptimization::split_edges(
        const double limit_edge_length
        ) {
        LOG::TRACE("{}({})", __FUNCTION__, limit_edge_length);

        mesh_f_processed_.fill(false);
        for (GEO::index_t f = 0, f_end = mesh_.facets.nb(); f < f_end; ++f) {
            if (mesh_f_processed_[f]
                || !mesh_f_used_[f]) // free facet
                continue;

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (!ALLOW_SPLIT_FIXED_EDGES_ && mesh_fc_fixed_[mesh_.facets.corner(f, lv)])
                    continue;

                if (const auto edge_length = get_edge_length(f, lv);
                    edge_length < limit_edge_length)
                    continue;

                const auto nf = mesh_.facets.adjacent(f, lv);

                /* Split */
                const GEO::index_t new_v = require_a_new_vertex();
                const GEO::index_t new_f0 = require_a_new_facet();
                const GEO::index_t new_f1 = (nf == GEO::NO_FACET) ? GEO::NO_FACET : require_a_new_facet();
                tri_edge_split(mesh_, f, lv, new_v, new_f0, new_f1);

                /* Label processed facets */
                mesh_f_processed_[f] = true;
                mesh_f_processed_[new_f0] = true;
                if (nf != GEO::NO_FACET) {
                    mesh_f_processed_[nf] = true;
                    assert(new_f1 != GEO::NO_FACET);
                    mesh_f_processed_[new_f1] = true;
                }

                /* Label new vertex */
                if (nf == GEO::NO_FACET)
                    mesh_v_boundary_[new_v] = true;
                if (mesh_fc_fixed_[mesh_.facets.corner(f, lv)]) {
                    // TODO
                }

                break;
            }
        }
    }

    void TriLocalOperationOptimization::collapse_edges(
        const double limit_edge_length
        ) {
        LOG::TRACE("{}({})", __FUNCTION__, limit_edge_length);

        mesh_f_processed_.fill(false);
        for (GEO::index_t f = 0, f_end = mesh_.facets.nb(); f < f_end; ++f) {
            if (mesh_f_processed_[f]
                || !mesh_f_used_[f]) // free facet
                continue;

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (ALLOW_COLLAPSE_FIXED_EDGES_) {
                    // TODO
                }
                else if (mesh_fc_fixed_[mesh_.facets.corner(f, lv)])
                    continue;

                if (const auto edge_length = get_edge_length(f, lv);
                    edge_length > limit_edge_length)
                    continue;

                const auto ev0 = mesh_.facets.vertex(f, lv);
                const auto ev1 = mesh_.facets.vertex(f, (lv+1)%3);
                const auto nf = mesh_.facets.adjacent(f, lv);

                if ((mesh_v_boundary_[ev0] && mesh_v_boundary_[ev1] && nf != GEO::NO_FACET)
                    || mesh_v_non_manifold_[ev0]
                    || mesh_v_non_manifold_[ev1]) // Prevent creation of new non-manifold vertex.
                    continue;

                if (mesh_v_fixed_[ev1]) /* Because collapse pulls v1 toward v0, no operation is performed when v1
                        is fixed, so that the vertex indices remain unchanged. */
                    continue;

                if (const bool ISOLATED_FACET = mesh_.facets.adjacent(f, 0) == GEO::NO_FACET &&
                                                mesh_.facets.adjacent(f, 1) == GEO::NO_FACET &&
                                                mesh_.facets.adjacent(f, 2) == GEO::NO_FACET;
                    ISOLATED_FACET
                    ) {
                    disuse_a_vertex(ev0);
                    disuse_a_vertex(ev1);
                    disuse_a_vertex(mesh_.facets.vertex(f, (lv+2)%3));
                    disuse_a_facet(f); // simply remove this facet
                    break;
                }

                if (!is_tri_edge_collapse_valid(mesh_, f, lv))
                    continue;

                /* Collapse */
                GEO::index_t disuse_v = GEO::NO_VERTEX;
                GEO::index_t disuse_f0 = GEO::NO_FACET;
                GEO::index_t disuse_f1 = GEO::NO_FACET;

                double R = 0.5; // mid point
                if (mesh_v_fixed_[ev0])
                    R = 0; // pull ev1 -> ev0
                tri_edge_collapse(mesh_, f, lv, disuse_v, disuse_f0, disuse_f1, R);
                assert(disuse_v == ev1);

                /* Label vertices */
                if (mesh_v_boundary_[ev1])
                    mesh_v_boundary_[ev0] = true;

                /* Disuse */
                disuse_a_vertex(disuse_v);
                disuse_a_facet(disuse_f0);
                if (disuse_f1 != GEO::NO_FACET)
                    disuse_a_facet(disuse_f1);

                break;
            }
        }
    }

    void TriLocalOperationOptimization::swap_edges(
        ) {
        LOG::TRACE(__FUNCTION__);

        mesh_f_processed_.fill(false);
        for (GEO::index_t f = 0, f_end = mesh_.facets.nb(); f < f_end; ++f) {
            if (mesh_f_processed_[f]
                || !mesh_f_used_[f]) // free facet
                    continue;

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (mesh_fc_fixed_[mesh_.facets.corner(f, lv)])
                    continue;

                const auto v0 = mesh_.facets.vertex(f, lv);
                const auto lv1 = (lv+1)%3;
                const auto v1 = mesh_.facets.vertex(f, lv1);
                const auto lv2 = (lv+2)%3;
                const auto v2 = mesh_.facets.vertex(f, lv2);
                const auto nf = mesh_.facets.adjacent(f, lv);
                if (nf == GEO::NO_FACET)
                    continue;
                const auto nlv = (mesh_.facets.find_vertex(nf, v0) + 1)%3;
                assert(nlv != GEO::NO_INDEX);
                const auto v3 = mesh_.facets.vertex(nf, nlv);
                assert(nf != GEO::NO_FACET);
                assert(mesh_f_used_[nf]);

                if (mesh_v_non_manifold_[v0] || mesh_v_non_manifold_[v1] || mesh_v_non_manifold_[v2] || mesh_v_non_manifold_[v3])
                    continue;

                if (!is_tri_edge_swap_valid(mesh_, f, lv))
                    continue;

                /* Valence diff */
                int valence0, valence1, valence2, valence3;
                std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
                {
                    get_vertex_incident_facets(mesh_, f, lv, ordered_f_and_lv);
                    valence0 = static_cast<int>(ordered_f_and_lv.size());
                }
                {
                    get_vertex_incident_facets(mesh_, f, lv1, ordered_f_and_lv);
                    valence1 = static_cast<int>(ordered_f_and_lv.size());
                }
                {
                    get_vertex_incident_facets(mesh_, f, lv2, ordered_f_and_lv);
                    valence2 = static_cast<int>(ordered_f_and_lv.size());
                }
                {
                    get_vertex_incident_facets(mesh_, nf, nlv, ordered_f_and_lv);
                    valence3 = static_cast<int>(ordered_f_and_lv.size());
                }
                constexpr int INTERIOR_IDEAL_VALENCE = 6;
                constexpr int BOUNDARY_IDEAL_VALENCE = 4;
                const auto v0_ideal_valence = mesh_v_boundary_[v0] ? BOUNDARY_IDEAL_VALENCE : INTERIOR_IDEAL_VALENCE;
                const auto v1_ideal_valence = mesh_v_boundary_[v1] ? BOUNDARY_IDEAL_VALENCE : INTERIOR_IDEAL_VALENCE;
                const auto v2_ideal_valence = mesh_v_boundary_[v2] ? BOUNDARY_IDEAL_VALENCE : INTERIOR_IDEAL_VALENCE;
                const auto v3_ideal_valence = mesh_v_boundary_[v3] ? BOUNDARY_IDEAL_VALENCE : INTERIOR_IDEAL_VALENCE;
                const auto prev_valence = std::pow(valence0-v0_ideal_valence, 2) +
                                          std::pow(valence1-v1_ideal_valence, 2) +
                                          std::pow(valence2-v2_ideal_valence, 2) +
                                          std::pow(valence3-v3_ideal_valence, 2);
                --valence0;
                --valence1;
                ++valence2;
                ++valence3;
                const auto post_valence = std::pow(valence0-v0_ideal_valence, 2) +
                                          std::pow(valence1-v1_ideal_valence, 2) +
                                          std::pow(valence2-v2_ideal_valence, 2) +
                                          std::pow(valence3-v3_ideal_valence, 2);
                if (post_valence > prev_valence)
                    continue;

                /* Swap */
                tri_edge_swap(mesh_, f, lv);

                /* Label processed facets */
                mesh_f_processed_[f] = true;
                mesh_f_processed_[nf] = true;
            }
        }
    }

    void TriLocalOperationOptimization::smooth_vertices(
        const GEO::index_t iterations_nb
        ) const {
        LOG::TRACE("{}({})", __FUNCTION__, iterations_nb);

        /* Get adjacency */
        std::vector<std::vector<GEO::index_t>> mesh_v_adjacent_v(mesh_.vertices.nb());
        for (const auto& f : mesh_.facets) {
            if (!mesh_f_used_[f])
                continue;

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                const auto& v = mesh_.facets.vertex(f, lv);
                assert(mesh_v_used_[v]);
                const auto& nv = mesh_.facets.vertex(f, (lv+1)%3);
                assert(mesh_v_used_[nv]);

                mesh_v_adjacent_v[v].push_back(nv);
                if (mesh_.facets.adjacent(f, lv) == GEO::NO_FACET)
                    mesh_v_adjacent_v[nv].push_back(v);
            }
        }

        if (mesh_2d_) {
            std::vector<GEO::vec2> mesh_v_new_pos(mesh_.vertices.nb()); // pre-allocated
            for (GEO::index_t iter = 0; iter < iterations_nb; ++iter) {
                // LOG::TRACE("iter: {}", iter);

                /* Compute average position */
                for (const auto& v : mesh_.vertices) {
                    if (!mesh_v_used_[v])
                        continue;

                    assert(!mesh_v_adjacent_v[v].empty());
                    mesh_v_new_pos[v] = GEO::vec2(0, 0);
                    for (const auto& nv : mesh_v_adjacent_v[v])
                        mesh_v_new_pos[v] += mesh_.vertices.point<2>(nv);
                    mesh_v_new_pos[v] /= mesh_v_adjacent_v[v].size();
                }

                /* Update */
                if (ALLOW_SMOOTH_FIXED_EDGE_VERTICES_) {
                    // TODO
                }
                for (const auto& v : mesh_.vertices) {
                    if (!mesh_v_used_[v] || mesh_v_fixed_[v])
                        continue;

                    mesh_.vertices.point<2>(v) = mesh_v_new_pos[v];
                }
            }
        }
        else {
            assert(mesh_.vertices.dimension() == 3);

            std::vector<GEO::vec3> mesh_v_new_pos(mesh_.vertices.nb()); // pre-allocated
            for (GEO::index_t iter = 0; iter < iterations_nb; ++iter) {
                // LOG::TRACE("iter: {}", iter);

                /* Compute average position */
                for (const auto& v : mesh_.vertices) {
                    if (!mesh_v_used_[v])
                        continue;

                    assert(!mesh_v_adjacent_v[v].empty());
                    mesh_v_new_pos[v] = GEO::vec3(0, 0, 0);
                    for (const auto& nv : mesh_v_adjacent_v[v])
                        mesh_v_new_pos[v] += mesh_.vertices.point(nv);
                    mesh_v_new_pos[v] /= mesh_v_adjacent_v[v].size();
                }

                /* Project to original mesh */
                for (const auto& v : mesh_.vertices) {
                    if (!mesh_v_used_[v])
                        continue;

                    GEO::vec3 nearest_pos;
                    double sq_dist;
                    original_mesh_facet_AABB_.nearest_facet(mesh_v_new_pos[v], nearest_pos, sq_dist);

                    mesh_v_new_pos[v] = nearest_pos;
                }

                /* Update */
                if (ALLOW_SMOOTH_FIXED_EDGE_VERTICES_) {
                    // TODO
                }
                for (const auto& v : mesh_.vertices) {
                    if (!mesh_v_used_[v] || mesh_v_fixed_[v])
                        continue;

                    mesh_.vertices.point(v) = mesh_v_new_pos[v];
                }
            }
        }
    }

    void TriLocalOperationOptimization::clean_unused_elements(
        ) {
        LOG::TRACE(__FUNCTION__);
        assert(mesh_v_used_.is_bound());
        assert(mesh_f_used_.is_bound());

        if (free_facets_.empty() && free_vertices_.empty())
            return;

        GEO::vector<GEO::index_t> facets_to_delete(mesh_.facets.nb(), 0);
        for (const auto& f : free_facets_) {
            if (f < mesh_.facets.nb())
                facets_to_delete[f] = 1;
        }
        mesh_.facets.delete_elements(facets_to_delete, true);

        free_vertices_.clear();
        free_facets_.clear();

        assert(std::ranges::all_of(mesh_v_used_.get_vector(), [](const auto& b){ return b; }));
        assert(std::ranges::all_of(mesh_f_used_.get_vector(), [](const auto& b){ return b; }));
    }
}
