//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "geolio/tri_local_operation_optimization.h"

#include <geogram/mesh/mesh_io.h>

#include "geolio/log.h"
#include "geolio/tri_operations.h"

namespace
{
    const std::string MESH_VERTICES_USED_ATTRIBUTE_NAME = "TriLocalOperationOptimization_used";
    const std::string MESH_FACETS_USED_ATTRIBUTE_NAME = "TriLocalOperationOptimization_used";
    const std::string MESH_VERTICES_FIXED_ATTRIBUTE_NAME = "TriLocalOperationOptimization_fixed";
}

namespace geolio
{
    TriLocalOperationOptimization::TriLocalOperationOptimization(
        GEO::Mesh& mesh
        ) : mesh_(mesh),
            mesh_2d_(mesh.vertices.dimension() == 2)
    {
        assert(mesh.facets.are_simplices());

        if (!mesh_2d_) {
            original_mesh_.copy(mesh_);
            original_mesh_facet_AABB_.initialize(original_mesh_);
        }
    }

    void TriLocalOperationOptimization::optimize(
        double target_edge_length,
        GEO::index_t rounds_nb
        ) {
        LOG::TRACE(__FUNCTION__);

        if (target_edge_length < 0) {
            target_edge_length = compute_average_mesh_edge_length();
            LOG::DEBUG("Automatically set the target edge length to the average edge length {}.", target_edge_length);
        }
        const double SPLIT_EDGE_LENGTH = 4.0/3.0 * target_edge_length;
        const double COLLAPSE_EDGE_LENGTH = 4.0/5.0 * target_edge_length;

        bind_attributes();

        fix_boundary_vertices();

        for (GEO::index_t round = 0; round < rounds_nb; ++round) {
            LOG::TRACE("round: {}/{}", round, rounds_nb);

            split_edges(SPLIT_EDGE_LENGTH);
            collapse_edges(COLLAPSE_EDGE_LENGTH);
            swap_edges();
            smooth_vertices(1);
        }

        // unbind_attributes();

        clean_unused_elements();
    }

    void TriLocalOperationOptimization::bind_attributes(
        ) {
        LOG::TRACE(__FUNCTION__);

        mesh_v_used_.bind(mesh_.vertices.attributes(), MESH_VERTICES_USED_ATTRIBUTE_NAME);
        mesh_v_used_.fill(true);
        mesh_f_used_.bind(mesh_.facets.attributes(), MESH_FACETS_USED_ATTRIBUTE_NAME);
        mesh_f_used_.fill(true);
        mesh_v_fixed_.bind(mesh_.vertices.attributes(), MESH_VERTICES_FIXED_ATTRIBUTE_NAME);
        mesh_v_fixed_.fill(false);
    }

    void TriLocalOperationOptimization::unbind_attributes(
        ) {
        LOG::TRACE(__FUNCTION__);

        assert(mesh_v_used_.is_bound());
        mesh_v_used_.destroy();
        assert(mesh_f_used_.is_bound());
        mesh_f_used_.destroy();
        assert(mesh_v_fixed_.is_bound());
        mesh_v_fixed_.destroy();
    }

    void TriLocalOperationOptimization::fix_boundary_vertices(
        ) {
        LOG::TRACE(__FUNCTION__);

        for (const auto& f : mesh_.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (mesh_.facets.adjacent(f, lv) == GEO::NO_FACET) {
                    mesh_v_fixed_[mesh_.facets.vertex(f, lv)] = true;
                    mesh_v_fixed_[mesh_.facets.vertex(f, (lv+1)%3)] = true;
                }
            }
        }
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
        if (v == GEO::NO_VERTEX)
            return;
        free_vertices_.push_back(v);
        mesh_v_used_[v] = false;
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
        if (f == GEO::NO_FACET)
            return;
        free_facets_.push_back(f);
        mesh_f_used_[f] = false;
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

        std::vector<bool> facet_processed(mesh_.facets.nb(), false);
        for (GEO::index_t f = 0, f_end = mesh_.facets.nb(); f < f_end; ++f) {
            if (facet_processed[f]
                || !mesh_f_used_[f]) // free facet
                continue;

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (const auto edge_length = get_edge_length(f, lv);
                    edge_length > limit_edge_length
                    ) {
                    const auto& nf = mesh_.facets.adjacent(f, lv);

                    /* Split */
                    const GEO::index_t new_v = require_a_new_vertex();
                    const GEO::index_t new_f0 = require_a_new_facet();
                    const GEO::index_t new_f1 = (nf == GEO::NO_FACET) ? GEO::NO_FACET : require_a_new_facet();
                    tri_edge_split(mesh_, f, lv, new_v, new_f0, new_f1);

                    /* Label processed facets */
                    facet_processed[f] = true;
                    facet_processed[new_f0] = true;
                    if (nf != GEO::NO_FACET) {
                        facet_processed[nf] = true;
                        assert(new_f1 < mesh_.facets.nb());
                        facet_processed[new_f1] = true;
                    }

                    /* Label boundary vertex */
                    if (nf == GEO::NO_FACET)
                        mesh_v_fixed_[new_v] = true;

                    break;
                }
            }
        }
    }

    void TriLocalOperationOptimization::collapse_edges(
        const double limit_edge_length
        ) {
        LOG::TRACE(__FUNCTION__);

        std::vector<bool> facet_processed(mesh_.facets.nb(), false);
        for (GEO::index_t f = 0, f_end = mesh_.facets.nb(); f < f_end; ++f) {
            if (facet_processed[f]
                || !mesh_f_used_[f]) // free facet
                continue;

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (const auto edge_length = get_edge_length(f, lv);
                    edge_length < limit_edge_length
                    ) {
                    const auto& ev0 = mesh_.facets.vertex(f, lv);
                    const auto& ev1 = mesh_.facets.vertex(f, (lv+1)%3);

                    if (mesh_v_fixed_[ev0] && mesh_v_fixed_[ev1]) // do not collapse fixed edge
                        continue;

                    double R = 0.5;
                    if (mesh_v_fixed_[ev0])
                        R = 0;
                    else if (mesh_v_fixed_[ev1]) {
                        R = 1;
                        mesh_v_fixed_[ev0] = true; // ev1 -> ev0
                    }

                    if (!is_tri_edge_collapse_valid(mesh_, f, lv, R))
                        continue;

                    /* Collapse */
                    GEO::index_t disuse_v = GEO::NO_VERTEX;
                    GEO::index_t disuse_f0 = GEO::NO_FACET;
                    GEO::index_t disuse_f1 = GEO::NO_FACET;
                    tri_edge_collapse(mesh_, f, lv, disuse_v, disuse_f0, disuse_f1, R);
                    disuse_a_vertex(disuse_v);
                    disuse_a_facet(disuse_f0);
                    disuse_a_facet(disuse_f1);

                    /* Label processed facets */
                    facet_processed[disuse_f0] = true;
                    if (disuse_f1 != GEO::NO_FACET)
                        facet_processed[disuse_f1] = true;

                    break;
                }
            }
        }
    }

    void TriLocalOperationOptimization::swap_edges(
        ) const {
        LOG::TRACE(__FUNCTION__);

        std::vector<bool> facet_processed(mesh_.facets.nb(), false);
        for (GEO::index_t f = 0, f_end = mesh_.facets.nb(); f < f_end; ++f) {
            if (facet_processed[f]
                || !mesh_f_used_[f]) // free facet
                    continue;

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (mesh_v_fixed_[mesh_.facets.vertex(f, lv)] &&
                    mesh_v_fixed_[mesh_.facets.vertex(f, (lv+1)%3)]) // do not swap fixed edge
                    continue;

                if (!is_tri_edge_swap_valid(mesh_, f, lv))
                    continue;

                /* Valence diff */
                const auto& v = mesh_.facets.vertex(f, lv);
                const auto& nf = mesh_.facets.adjacent(f, lv);
                assert(nf != GEO::NO_FACET);

                GEO::index_t valence0, valence1, valence2, valence3;
                std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv;
                {
                    get_vertex_incident_facets(mesh_, f, lv, ordered_f_and_lv);
                    valence0 = ordered_f_and_lv.size();
                }
                {
                    get_vertex_incident_facets(mesh_, f, (lv+1)%3, ordered_f_and_lv);
                    valence1 = ordered_f_and_lv.size();
                }
                {
                    get_vertex_incident_facets(mesh_, f, (lv+2)%3, ordered_f_and_lv);
                    valence2 = ordered_f_and_lv.size();
                }
                {
                    const auto nlv = mesh_.facets.find_vertex(nf, v);
                    get_vertex_incident_facets(mesh_, nf, (nlv+1)%3, ordered_f_and_lv);
                    valence3 = ordered_f_and_lv.size();
                }
                constexpr GEO::index_t IDEAL_DEGREE = 4;
                const auto prev_valence = std::pow(valence0-IDEAL_DEGREE, 2) + std::pow(valence1-IDEAL_DEGREE, 2) +
                                          std::pow(valence2-IDEAL_DEGREE, 2) + std::pow(valence3-IDEAL_DEGREE, 2);
                --valence0;
                --valence1;
                ++valence2;
                ++valence3;
                const auto post_valence = std::pow(valence0-IDEAL_DEGREE, 2) + std::pow(valence1-IDEAL_DEGREE, 2) +
                                          std::pow(valence2-IDEAL_DEGREE, 2) + std::pow(valence3-IDEAL_DEGREE, 2);
                if (post_valence > prev_valence)
                    continue;

                /* Swap */
                tri_edge_swap(mesh_, f, lv);

                /* Label processed facets */
                facet_processed[f] = true;
                facet_processed[nf] = true;
            }
        }
    }

    void TriLocalOperationOptimization::smooth_vertices(
        const GEO::index_t iterations_nb
        ) const {
        LOG::TRACE(__FUNCTION__);

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

                    mesh_v_new_pos[v] = GEO::vec2(0, 0);
                    for (const auto& nv : mesh_v_adjacent_v[v])
                        mesh_v_new_pos[v] += mesh_.vertices.point<2>(nv);
                    assert(!mesh_v_adjacent_v[v].empty());
                    mesh_v_new_pos[v] /= mesh_v_adjacent_v[v].size();
                }

                /* Update */
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

                    mesh_v_new_pos[v] = GEO::vec3(0, 0, 0);
                    for (const auto& nv : mesh_v_adjacent_v[v])
                        mesh_v_new_pos[v] += mesh_.vertices.point(nv);
                    assert(!mesh_v_adjacent_v[v].empty());
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
                for (const auto& v : mesh_.vertices) {
                    if (!mesh_v_used_[v] || mesh_v_fixed_[v])
                        continue;

                    mesh_.vertices.point(v) = mesh_v_new_pos[v];
                }
            }
        }
    }

    void TriLocalOperationOptimization::clean_unused_elements(
        ) const {
        LOG::TRACE(__FUNCTION__);

        GEO::vector<GEO::index_t> facets_to_delete(mesh_.facets.nb(), 0);
        for (const auto& f : free_facets_)
            facets_to_delete[f] = 1;
        mesh_.facets.delete_elements(facets_to_delete, true);
    }
}
