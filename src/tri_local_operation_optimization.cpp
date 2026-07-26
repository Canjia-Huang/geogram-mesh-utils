//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "geolio/tri_local_operation_optimization.h"
#include "geolio/log.h"
#include "geolio/tri_operations.h"

namespace geolio
{
    TriLocalOperationOptimization::TriLocalOperationOptimization(
        GEO::Mesh& mesh,
        const double target_length
        ) : mesh_(mesh),
            target_length_(target_length)
    {
        assert(mesh.facets.are_simplices());

        if (target_length_ < 0) {
            target_length_ = compute_average_mesh_edge_length();
            LOG::DEBUG("Automatically set the target edge length to the average edge length {}.", target_length_);
        }

        // DEBUG
        split_edges(4.0/3.0 * target_length_);

        clean_unused_elements();
    }

    void TriLocalOperationOptimization::split_edges(
        const double limit_length
        ) {
        LOG::TRACE("{}({})", __FUNCTION__, limit_length);

        std::vector<bool> facet_processed(mesh_.facets.nb(), false);
        for (GEO::index_t f = 0, f_end = mesh_.facets.nb(); f < f_end; ++f) {
            if (facet_processed[f])
                continue;

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (const auto edge_length = GEO::distance(mesh_.facets.point(f, lv), mesh_.facets.point(f, (lv+1)%3));
                    edge_length > limit_length
                    ) {
                    const auto& nf = mesh_.facets.adjacent(f, lv);

                    /* Allocate new elements */
                    if (free_vertices_.size() < 1)
                        allocate_new_vertices();
                    assert(free_vertices_.size() >= 1);

                    if (free_facets_.size() < 2)
                        allocate_new_facets();
                    assert(free_facets_.size() >= 2);

                    const GEO::index_t new_v = free_vertices_.back();
                    free_vertices_.pop_back();

                    const GEO::index_t new_f0 = free_facets_.back();
                    free_facets_.pop_back();
                    GEO::index_t new_f1 = GEO::NO_FACET;
                    if (nf != GEO::NO_FACET) {
                        new_f1 = free_facets_.back();
                        free_facets_.pop_back();
                    }

                    /* Split */
                    tri_edge_split(mesh_, f, lv, new_v, new_f0, new_f1);

                    /* Label processed facets */
                    facet_processed[f] = true;
                    facet_processed[new_f0] = true;
                    if (nf != GEO::NO_FACET) {
                        facet_processed[nf] = true;
                        facet_processed[new_f1] = true;
                    }

                    break;
                }
            }
        }
    }

    double TriLocalOperationOptimization::compute_average_mesh_edge_length(
        ) const {
        LOG::TRACE(__FUNCTION__);

        double l = 0;
        GEO::index_t edges_nb = 0;
        for (const auto& f : mesh_.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                l += GEO::distance(mesh_.facets.point(f, lv), mesh_.facets.point(f, (lv+1)%3));
                ++edges_nb;
            }
        }

        return l / edges_nb;
    }

    void TriLocalOperationOptimization::allocate_new_vertices(
        ) {
        LOG::TRACE(__FUNCTION__);

        const GEO::index_t PREV_MESH_VERTICES_NB = mesh_.vertices.nb();

        mesh_.vertices.create_vertices(PREV_MESH_VERTICES_NB);
        free_vertices_.reserve(free_vertices_.size()+PREV_MESH_VERTICES_NB);
        for (GEO::index_t v = PREV_MESH_VERTICES_NB, v_end = mesh_.vertices.nb(); v < v_end; ++v)
            free_vertices_.push_back(v);
    }

    void TriLocalOperationOptimization::allocate_new_facets(
        ) {
        LOG::TRACE(__FUNCTION__);

        const GEO::index_t PREV_MESH_FACETS_NB = mesh_.facets.nb();

        mesh_.facets.create_triangles(PREV_MESH_FACETS_NB);
        free_facets_.reserve(free_facets_.size()+PREV_MESH_FACETS_NB);
        for (GEO::index_t f = PREV_MESH_FACETS_NB, f_end = mesh_.facets.nb(); f < f_end; ++f)
            free_facets_.push_back(f);
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
