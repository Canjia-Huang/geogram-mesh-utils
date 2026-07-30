//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_KERNEL_H
#define GEOLIO_KERNEL_H

#include <geogram/mesh/mesh.h>
#include "geolio/utils.h"

namespace geolio
{
    class MeshElementManager {
    public:
        explicit MeshElementManager(GEO::Mesh& mesh);

        ~MeshElementManager();

        [[nodiscard]] inline GEO::index_t require_a_new_vertex();

        inline void disuse_vertex(GEO::index_t v);

        [[nodiscard]] inline GEO::index_t require_a_new_facet();

        inline void disuse_facet(GEO::index_t f);

        [[nodiscard]] inline double get_edge_length(GEO::index_t f, GEO::index_t lv) const;

        GEO::Mesh& mesh;
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

        const bool mesh_2d_; // mesh.vertices.dimension() == 2
        const std::string attribute_name_; // Prevent anyone from using these attributes externally (unsafety).

        std::vector<GEO::index_t> free_vertices_;
        std::vector<GEO::index_t> free_facets_;
    };
}

#endif //GEOLIO_KERNEL_H
