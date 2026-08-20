//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/18.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "application.h"
#include "geolio/common/log.h"
#include "object/mesh_object.h"

namespace geolio::geobox
{
    GeoBoxApplication::GeoBoxApplication(
        ) : SimpleMeshApplication("GeoBox")
    {}

    void GeoBoxApplication::draw_gui(
        ) {
        draw_menu_bar();
        draw_dock_space();
        draw_viewer_properties_window();
        draw_object_properties_window();
        // draw_console();
        // draw_command_window();
        // draw_command_line_editor();

        if (text_editor_visible_)
            text_editor_.draw();
        if (ImGui::FileDialog("##load_dlg", filename_, GEO::geo_imgui_string_length))
            load(filename_);

        if (ImGui::FileDialog("##save_dlg", filename_, GEO::geo_imgui_string_length))
            save(filename_);

        if (status_bar_->active()) {
            const auto w = static_cast<float>(get_frame_buffer_width());
            const auto h = static_cast<float>(get_frame_buffer_height());
            float STATUS_HEIGHT = status_bar_->get_window_height();
            if(STATUS_HEIGHT == 0.0f)
                STATUS_HEIGHT = static_cast<float>(get_font_size());

            STATUS_HEIGHT *= 1.5f;
            ImGui::SetNextWindowPos(
                ImVec2(0.0f, h-STATUS_HEIGHT),
                ImGuiCond_Always
            );
            ImGui::SetNextWindowSize(
                ImVec2(w,STATUS_HEIGHT-1.0f),
                ImGuiCond_Always
            );
            status_bar_->draw();
        }
    }

    void GeoBoxApplication::draw_viewer_properties(
        ) {
        if (ImGui::Button((GEO::icon_UTF8("home")).c_str(), ImVec2(-1.0, 0.0)))
            home();

        ImGui::Separator();
        if(three_D_) {
            ImGui::Checkbox("Lighting", &lighting_);
            if(lighting_) {
                ImGui::Checkbox("Edit light", &edit_light_);
            }
            ImGui::Separator();
            ImGui::Checkbox("Clipping", &clipping_);
            if(clipping_) {
                ImGui::Combo(
                    "##mode", (int*)&clip_mode_,
                    "std. GL\0cells\0stradd.\0slice\0\0"
                );
                ImGui::Checkbox(
                    "edit clip", &edit_clip_
                );
                ImGui::Checkbox(
                    "fixed clip", &fixed_clip_
                );
            }
            ImGui::Separator();
        }
        ImGui::ColorEdit3WithPalette("Backgnd", background_color_.data());
    }

    void GeoBoxApplication::draw_controller_properties(
        ) {
        if (ImGui::CollapsingHeader("Viewer"))
            draw_viewer_properties();
    }

    void GeoBoxApplication::draw_object_properties() {
        if (base_objects_.empty())
            return;

        for (const auto& base_object : base_objects_)
            base_object->draw_object_properties();
    }

    void GeoBoxApplication::draw_scene(
        ) {
        if (base_objects_.empty())
            return;

        for (const auto& base_object : base_objects_)
            base_object->draw_scene(lighting_);
    }

    bool GeoBoxApplication::load(
        const std::string& filename
        ) {
        home();

        GEO::Mesh mesh;
        if (!mesh.load(filename)) {
            LOG::ERROR("Cannot load mesh from `{}`!", filename);
            return false;
        }

        double xyzmin[3];
        double xyzmax[3];
        get_bbox(mesh, xyzmin, xyzmax, false);
        set_region_of_interest(
                xyzmin[0], xyzmin[1], xyzmin[2],
                xyzmax[0], xyzmax[1], xyzmax[2]);

        base_objects_.push_back(std::make_shared<MeshObject>(mesh));

        return true;
    }
}
