//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

#include "mesh_element_manager.h"
#include <cassert>
#include "geolio/log.h"

namespace geolio
{
    MeshElementManager::MeshElementManager(
        GEO::Mesh& mesh
        ) : mesh(mesh),
            mesh_2d_(mesh.vertices.dimension() == 2),
            attribute_name_(generate_random_string(22))
    {
        /* Bind attributes */
        mesh_v_boundary.bind(mesh.vertices.attributes(), attribute_name_+":boundary");
        mesh_v_boundary.fill(false);
        mesh_v_fixed.bind(mesh.vertices.attributes(), attribute_name_+":fixed");
        mesh_v_fixed.fill(false);
        mesh_v_non_manifold.bind(mesh.vertices.attributes(), attribute_name_+":non-manifold");
        mesh_v_non_manifold.fill(false);
        mesh_v_used.bind(mesh.vertices.attributes(), attribute_name_+":used");
        mesh_v_used.fill(true);
        mesh_f_used.bind(mesh.facets.attributes(), attribute_name_+":used");
        mesh_f_used.fill(true);
        mesh_fc_fixed.bind(mesh.facet_corners.attributes(), attribute_name_+":fixed");
        mesh_fc_fixed.fill(false);
    }

    MeshElementManager::~MeshElementManager(
        ) {
        /* Destroy attributes */
        if (mesh_v_boundary.is_bound())
            mesh_v_boundary.destroy();
        if (mesh_v_fixed.is_bound())
            mesh_v_fixed.destroy();
        if (mesh_v_non_manifold.is_bound())
            mesh_v_non_manifold.destroy();
        if (mesh_v_used.is_bound())
            mesh_v_used.destroy();
        if (mesh_f_used.is_bound())
            mesh_f_used.destroy();
        if (mesh_fc_fixed.is_bound())
            mesh_fc_fixed.destroy();
    }

    GEO::index_t MeshElementManager::require_a_new_vertex(
        ) {
        if (free_vertices_.empty())
            allocate_new_vertices();
        assert(!free_vertices_.empty());

        const GEO::index_t new_v = free_vertices_.back();
        free_vertices_.pop_back();
        assert(new_v < mesh.vertices.nb());
        mesh_v_used[new_v] = true;

        return new_v;
    }

    void MeshElementManager::disuse_a_vertex(
        const GEO::index_t v
        ) {
        /* Recycle */
        assert(v < mesh.vertices.nb());
        free_vertices_.push_back(v);

        /* Init attributes */
        mesh_v_boundary[v]     = false;
        mesh_v_fixed[v]        = false;
        mesh_v_non_manifold[v] = false;
        mesh_v_used[v]         = false;
    }

    GEO::index_t MeshElementManager::require_a_new_facet(
        ) {
        if (free_facets_.empty())
            allocate_new_facets();
        assert(!free_facets_.empty());

        const GEO::index_t new_f = free_facets_.back();
        free_facets_.pop_back();
        assert(new_f < mesh.facets.nb());
        mesh_f_used[new_f] = true;

        return new_f;
    }

    void MeshElementManager::disuse_a_facet(
        const GEO::index_t f
        ) {
        /* Recycle */
        assert(f < mesh.facets.nb());
        free_facets_.push_back(f);

        /* Init attributes */
        mesh_f_processed[f] = false;
        mesh_f_used[f] = false;
        for (GEO::index_t lv = 0; lv < 3; ++lv)
            mesh_fc_fixed[3*f+lv] = false;
    }

    double MeshElementManager::get_edge_length(
        const GEO::index_t f,
        const GEO::index_t lv
        ) const {
        assert(f < mesh.facets.nb());
        assert(lv < 3);
        if (mesh_2d_)
            return GEO::distance(mesh.facets.point<2>(f, lv), mesh.facets.point<2>(f, (lv+1)%3));
        return GEO::distance(mesh.facets.point(f, lv), mesh.facets.point(f, (lv+1)%3));
    }

    void MeshElementManager::allocate_new_vertices(
        ) {
        assert(mesh.vertices.nb() > 0);

        const GEO::index_t PREV_MESH_VERTICES_NB = mesh.vertices.nb();
        const GEO::index_t ALLOCATE_MESH_VERTICES_NB = PREV_MESH_VERTICES_NB;

        mesh.vertices.create_vertices(ALLOCATE_MESH_VERTICES_NB);
        free_vertices_.reserve(free_vertices_.size() + ALLOCATE_MESH_VERTICES_NB);
        for (GEO::index_t v = PREV_MESH_VERTICES_NB, v_end = mesh.vertices.nb(); v < v_end; ++v)
            free_vertices_.push_back(v);
    }

    void MeshElementManager::allocate_new_facets(
        ) {
        assert(mesh.facets.nb() > 0);

        const GEO::index_t PREV_MESH_FACETS_NB = mesh.facets.nb();
        const GEO::index_t ALLOCATE_MESH_FACETS_NB = PREV_MESH_FACETS_NB;

        mesh.facets.create_triangles(ALLOCATE_MESH_FACETS_NB);
        free_facets_.reserve(free_facets_.size() + ALLOCATE_MESH_FACETS_NB);
        for (GEO::index_t f = PREV_MESH_FACETS_NB, f_end = mesh.facets.nb(); f < f_end; ++f)
            free_facets_.push_back(f);
    }
}