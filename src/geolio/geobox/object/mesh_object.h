//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_OBJECT_H
#define GEOLIO_MESH_OBJECT_H

#include "base_object.h"
#include <geogram/mesh/mesh.h>
#include <geogram_gfx/mesh/mesh_gfx.h>

namespace geolio::geobox
{
    class MeshObject : public BaseObject {
    public:
        MeshObject();

        void draw(bool lighting) override;

    protected:
        // ref <geogram_gfx/gui/simple_mesh_application.cpp> draw_points()
        void draw_points();

        // ref <geogram_gfx/gui/simple_mesh_application.cpp> draw_surface()
        void draw_surface();

        // ref <geogram_gfx/gui/simple_mesh_application.cpp> draw_edges()
        void draw_edges();

        // ref <geogram_gfx/gui/simple_mesh_application.cpp> draw_volume()
        void draw_volume(bool lighting);

        GEO::Mesh mesh_;
        GEO::MeshGfx mesh_gfx_;

        bool show_vertices_ = false;
        bool show_vertices_selection_ = false;
        float vertices_size_;
        GEO::vec4f vertices_color_;
        float vertices_transparency_;

        bool show_surface_;
        bool show_surface_sides_;
        GEO::vec4f surface_color_;
        GEO::vec4f surface_color_2_;

        bool show_mesh_;
        float mesh_width_;
        GEO::vec4f mesh_color_;

        bool show_surface_borders_;
        float surface_borders_width_;
        GEO::vec4f surface_borders_color_;

        bool show_volume_;
        float cells_shrink_;
        GEO::vec4f volume_color_;
        bool show_colored_cells_;
        bool show_hexes_;
        bool show_connectors_;

        struct ColormapInfo {
            ColormapInfo() : texture(0) {
            }
            GLuint texture;
            std::string name;
        };
        GEO::vector<ColormapInfo> colormaps_;

        bool show_attributes_ = false;
        GEO::index_t current_colormap_index_ = 0;
        std::string       attribute_;
        GEO::MeshElementsFlags attribute_subelements_;
        std::string       attribute_name_;
        float             attribute_min_ = 0;
        float             attribute_max_ = 0;
    };
}

#endif //GEOLIO_MESH_OBJECT_H
