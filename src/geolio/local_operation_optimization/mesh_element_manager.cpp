//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "mesh_element_manager.h"
#include <cassert>

namespace geolio
{
    /**
     * @brief Constructs a MeshElementManager over the given triangle mesh.
     * @details Records the mesh reference, whether it is 2D (vertex dimension 2), and
     *          generates a random attribute name prefix via generate_random_string().
     *          Binds and initializes the per-vertex, per-facet and per-corner usage/fixed
     *          attributes under that prefix: vertices and facets start as used, boundary,
     *          fixed and non-manifold flags start false.
     * @param[in] _mesh The triangle mesh to manage; it must contain only simplex facets.
     */
    MeshElementManager::MeshElementManager(
        GEO::Mesh& _mesh
        ) : mesh(_mesh),
            mesh_2d(_mesh.vertices.dimension() == 2),
            attribute_name_(generate_random_string(22))
    {
        assert(mesh.facets.are_simplices());

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

    /**
     * @brief Destroys the MeshElementManager.
     * @details Destroys all bound element attributes (boundary, fixed, non-manifold, used
     *          and fixed-corner) if they are still bound, releasing their mesh storage.
     */
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

    /**
     * @brief Physically removes disused facets (and optionally isolated vertices) from the mesh.
     * @details If both free pools are empty, returns immediately. Otherwise builds a delete
     *          mask over the free-facet indices and calls mesh.facets.delete_elements(),
     *          forwarding remove_isolated_vertices, then clears both free pools. Asserts that
     *          all remaining used flags are consistent after deletion.
     * @param[in] remove_isolated_vertices When true, vertices that become isolated after the
     *                                     facet deletion are removed as well.
     */
    void MeshElementManager::clean_unused_elements(
        const bool remove_isolated_vertices
        ) {
        if (free_facets_.empty() && free_vertices_.empty())
            return;

        GEO::vector<GEO::index_t> facets_to_delete(mesh.facets.nb(), 0);
        for (const auto& f : free_facets_) {
            if (f < mesh.facets.nb())
                facets_to_delete[f] = 1;
        }
        mesh.facets.delete_elements(facets_to_delete, remove_isolated_vertices);

        free_vertices_.clear();
        free_facets_.clear();

        assert(!remove_isolated_vertices ||
            std::ranges::all_of(mesh_v_used.get_vector(), [](const auto& b){ return b; }));
        assert(std::ranges::all_of(mesh_f_used.get_vector(), [](const auto& b){ return b; }));
    }

    /**
     * @brief Computes the average edge length over all facets of the mesh.
     * @details Iterates over every facet and accumulates the length of all three local edges
     *          via get_edge_length(), then divides the sum by the total number of edges.
     * @return The average mesh edge length.
     */
    double MeshElementManager::compute_average_mesh_edge_length(
        ) const {
        double l = 0;
        GEO::index_t edges_nb = 0;
        for (const auto& f : mesh.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                l += get_edge_length(f, lv);
                ++edges_nb;
            }
        }

        return l / edges_nb;
    }

    /**
     * @brief Allocates a batch of new mesh vertices for future reuse.
     * @details Appends as many new vertices as the mesh currently has via create_vertices()
     *          and pushes their indices onto the free-vertex pool so require_new_vertex()
     *          can hand them out later.
     */
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

    /**
     * @brief Allocates a batch of new triangle facets for future reuse.
     * @details Appends as many new triangles as the mesh currently has via create_triangles()
     *          and pushes their indices onto the free-facet pool so require_new_facet()
     *          can hand them out later.
     */
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