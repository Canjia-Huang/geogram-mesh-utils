//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_ELEMENT_MANAGER_H
#define GEOLIO_MESH_ELEMENT_MANAGER_H

#include <geogram/mesh/mesh.h>
#include "geolio/utils.h"
#include <cassert>

namespace geolio
{
    class MeshElementManager {
    public:
        explicit MeshElementManager(GEO::Mesh& _mesh);

        ~MeshElementManager();

        [[nodiscard]] GEO::index_t require_new_vertex(
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

        void disuse_vertex(GEO::index_t v) {
            assert(v < mesh.vertices.nb());

            /* Recycle */
            free_vertices_.push_back(v);

            /* Init attributes */
            mesh_v_boundary[v]     = false;
            mesh_v_fixed[v]        = false;
            mesh_v_non_manifold[v] = false;
            mesh_v_used[v]         = false;
        }

        [[nodiscard]] GEO::index_t require_new_facet(
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

        void disuse_facet(GEO::index_t f) {
            assert(f < mesh.facets.nb());

            /* Recycle */
            free_facets_.push_back(f);

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                if (const auto& v = mesh.facets.vertex(f, lv);
                    v != GEO::NO_VERTEX && mesh_v_used[v])
                    disuse_vertex(v);
            }

            /* Init attributes */
            mesh_f_used[f] = false;
            for (GEO::index_t lv = 0; lv < 3; ++lv)
                mesh_fc_fixed[3*f+lv] = false;
        }

        void clean_unused_elements();

        [[nodiscard]] double get_edge_length(GEO::index_t f, GEO::index_t lv) const {
            assert(f < mesh.facets.nb());
            assert(lv < 3);
            if (mesh_2d)
                return GEO::distance(mesh.facets.point<2>(f, lv), mesh.facets.point<2>(f, (lv+1)%3));
            return GEO::distance(mesh.facets.point(f, lv), mesh.facets.point(f, (lv+1)%3));
        }

        [[nodiscard]] double compute_average_mesh_edge_length() const;

        GEO::Mesh& mesh;
        const bool mesh_2d; // mesh.vertices.dimension() == 2
        GEO::Attribute<bool> mesh_v_boundary; // v -> on boundary
        GEO::Attribute<bool> mesh_v_fixed; // v -> fixed
        GEO::Attribute<bool> mesh_v_non_manifold; // v -> non manifold
        GEO::Attribute<bool> mesh_v_used; // v -> used
        GEO::Attribute<bool> mesh_f_used; // f -> used
        GEO::Attribute<bool> mesh_fc_fixed; // fc (edge) -> fixed

        bool ALLOW_SPLIT_FIXED_EDGES               = false;
        bool ALLOW_COLLAPSE_FIXED_EDGES            = false;
        bool ALLOW_SMOOTH_FIXED_EDGE_VERTICES      = false;

    private:
        void allocate_new_vertices();

        void allocate_new_facets();

        const std::string attribute_name_; // Prevent anyone from using these attributes externally (unsafety).

        std::vector<GEO::index_t> free_vertices_;
        std::vector<GEO::index_t> free_facets_;
    };
}

#endif //GEOLIO_MESH_ELEMENT_MANAGER_H
