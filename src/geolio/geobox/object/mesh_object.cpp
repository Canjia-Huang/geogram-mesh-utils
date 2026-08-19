//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "mesh_object.h"

namespace geolio::geobox
{
    MeshObject::MeshObject(
        ) {

    }

    void MeshObject::draw(
        const bool lighting
        ) {
        if (mesh_gfx_.mesh() == nullptr)
            return;

        mesh_gfx_.set_lighting(lighting);

        if (show_attributes_) {
            mesh_gfx_.set_scalar_attribute(
                attribute_subelements_,
                attribute_name_,
                static_cast<double>(attribute_min_),
                static_cast<double>(attribute_max_),
                colormaps_[current_colormap_index_].texture,
                1);
        }
        else
            mesh_gfx_.unset_scalar_attribute();

        draw_points();
        draw_surface();
        draw_edges();
        draw_volume(lighting);
    }

    void MeshObject::draw_points(
        ) {
        if(show_vertices_) {
            if(vertices_transparency_ != 0.0f) {
                glDepthMask(GL_FALSE);
                glEnable(GL_BLEND);
                glBlendEquation(GL_FUNC_ADD);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE);
            }
            mesh_gfx_.set_points_color(
                vertices_color_.x, vertices_color_.y, vertices_color_.z,
                1.0f - vertices_transparency_
            );
            mesh_gfx_.set_points_size(vertices_size_);
            mesh_gfx_.draw_vertices();

            if(vertices_transparency_ != 0.0f) {
                glDisable(GL_BLEND);
                glDepthMask(GL_TRUE);
            }
        }

        if(show_vertices_selection_) {
            mesh_gfx_.set_points_color(1.0, 0.0, 0.0);
            mesh_gfx_.set_points_size(2.0f * vertices_size_);
            mesh_gfx_.set_vertices_selection("selection");
            mesh_gfx_.draw_vertices();
            mesh_gfx_.set_vertices_selection("");
        }
    }

    void MeshObject::draw_surface(
        ) {
        mesh_gfx_.set_mesh_color(0.0, 0.0, 0.0);

        mesh_gfx_.set_surface_color(
            surface_color_.x, surface_color_.y, surface_color_.z);
        if (show_surface_sides_) {
            mesh_gfx_.set_backface_surface_color(
                surface_color_2_.x, surface_color_2_.y, surface_color_2_.z
            );
        }

        mesh_gfx_.set_show_mesh(show_mesh_);
        mesh_gfx_.set_mesh_color(mesh_color_.x, mesh_color_.y, mesh_color_.z);
        mesh_gfx_.set_mesh_width(static_cast<GEO::index_t>(mesh_width_ * 10.0f));

        if (show_surface_) {
            const float specular_backup = glupGetSpecular();
            glupSetSpecular(0.4f);
            mesh_gfx_.draw_surface();
            glupSetSpecular(specular_backup);
        }

        if (show_surface_borders_) {
            mesh_gfx_.set_mesh_color(
                surface_borders_color_.x,
                surface_borders_color_.y,
                surface_borders_color_.z);
            mesh_gfx_.set_mesh_border_width(
                static_cast<GEO::index_t>(surface_borders_width_ * 10.0f));
            mesh_gfx_.draw_surface_borders();
        }
    }

    void MeshObject::draw_edges(
        ) {
        if (show_mesh_)
            mesh_gfx_.draw_edges();
    }

    void MeshObject::draw_volume(
        const bool lighting
        ) {
        if (show_volume_) {
            if (glupIsEnabled(GLUP_CLIPPING) &&
                glupGetClipMode() == GLUP_CLIP_SLICE_CELLS)
                mesh_gfx_.set_lighting(false);

            mesh_gfx_.set_shrink(static_cast<double>(cells_shrink_));
            mesh_gfx_.set_draw_cells(GEO::MESH_HEX, show_hexes_);
            mesh_gfx_.set_draw_cells(GEO::MESH_CONNECTOR, show_connectors_);

            if(show_colored_cells_)
                mesh_gfx_.set_cells_colors_by_type();
            else
                mesh_gfx_.set_cells_color(
                    volume_color_.x, volume_color_.y, volume_color_.z);

            mesh_gfx_.draw_volume();

            mesh_gfx_.set_lighting(lighting);
        }
    }
}
