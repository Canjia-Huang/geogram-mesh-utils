//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/18.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "application.h"
#include <algorithm>
#include <geogram/basic/command_line.h>
#include "geolio/common/log.h"
#include "geolio/common/parse_filepath.h"
#include "object/mesh_object.h"

namespace geolio::geobox
{
    GeoBoxApplication::GeoBoxApplication(
        ) : SimpleMeshApplication("Geolio - GeoBox")
    {
        init();
    }

    void GeoBoxApplication::draw_gui(
        ) {
        draw_menu_bar();
        draw_controller_properties_window();
        // draw_viewer_properties_window();
        // draw_object_properties_window();
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

    void GeoBoxApplication::init(
        ) {
        for (GEO::coord_index_t i = 0; i < 3; ++i) {
            xyzmin_[i] = std::numeric_limits<double>::max();
            xyzmax_[i] = -std::numeric_limits<double>::max();
        }
    }

    void GeoBoxApplication::init_colormaps(
        ) {
        geolio::geobox::init_colormaps(my_colormaps_);
    }

    void GeoBoxApplication::draw_controller_properties_window(
        ) {
        const ImVec2 viewport_size = ImGui::GetMainViewport()->Size;
        ImGui::SetNextWindowPos(
            ImVec2(0.0f, ImGui::GetFrameHeight()), ImGuiCond_FirstUseEver
        );
        ImGui::SetNextWindowSize(
            ImVec2(viewport_size.x * 0.25f, viewport_size.y * 0.5f),
            ImGuiCond_FirstUseEver
        );
        ImGui::SetNextWindowBgAlpha(0.6f);
        if (ImGui::Begin("Controller", nullptr, ImGuiWindowFlags_NoDocking))
            draw_controller_properties();

        ImGui::End();
    }

    void GeoBoxApplication::draw_controller_properties(
        ) {
        if (ImGui::CollapsingHeader("Viewer"))
            draw_viewer_properties();
        if (ImGui::CollapsingHeader("Object", ImGuiTreeNodeFlags_DefaultOpen))
            draw_objects_properties();
    }

    void GeoBoxApplication::draw_viewer_properties(
        ) {
        if (ImGui::Button((GEO::icon_UTF8("home")).c_str(), ImVec2(-1.0, 0.0)))
            home();

        ImGui::Separator();
        if (three_D_) {
            ImGui::Checkbox("Lighting", &lighting_);
            if(lighting_) {
                ImGui::Checkbox("Edit light", &edit_light_);
            }
            ImGui::Separator();
            ImGui::Checkbox("Clipping", &clipping_);
            if (clipping_) {
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

    void GeoBoxApplication::draw_objects_properties(
        ) {
        // Geogram's icon font is monospaced (advance = 1.5*font_size), which can
        // exceed the default button height. Size the button to the icon's actual
        // text extent so ImGui's (0.5,0.5) text alignment centers the glyph.
        const float icon_text_width =
            ImGui::CalcTextSize(GEO::icon_UTF8("xmark").c_str()).x;
        const auto icon_button_size = 0.75f * std::max(
            ImGui::GetFrameHeight(),
            icon_text_width + 2.0f * ImGui::GetStyle().FramePadding.x);

        // With the box shrunken to 75%, tighten the buttons' inner padding so
        // the glyph still fits and stays centered inside it.
        const ImVec2 icon_frame_padding(
            std::max(0.0f, (icon_button_size - icon_text_width) * 0.5f),
            std::max(0.0f, (icon_button_size - ImGui::GetTextLineHeight()) * 0.5f));

        for (auto it = base_objects_.begin(); it != base_objects_.end();) {
            const auto& base_object = *it;

            ImGui::PushID(base_object.get());

            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, icon_frame_padding);
            if (ImGui::Button(
                GEO::icon_UTF8(
                    base_object->visible() ? "eye" : "eye-slash").c_str(),
                ImVec2(icon_button_size, icon_button_size)
                ))
                base_object->set_visible(!base_object->visible());

            ImGui::SameLine();
            if (ImGui::Button(
                GEO::icon_UTF8("xmark").c_str(),
                ImVec2(icon_button_size, icon_button_size)
                )) {
                ImGui::PopStyleVar();
                it = base_objects_.erase(it);
                ImGui::PopID();

                if (base_objects_.empty())
                    init();

                continue;
            }
            ImGui::PopStyleVar();

            ImGui::SameLine();
            // The rest of the row (name + trailing space) is clickable and
            // selects the object; the text is vertically centered like the buttons.
            ImGui::PushStyleVar(
                ImGuiStyleVar_SelectableTextAlign, ImVec2(0.0f, 0.5f));
            const bool is_selected =
                (selected_object_.lock() == base_object);
            if (ImGui::Selectable(
                base_object->name().c_str(),
                is_selected,
                0,
                ImVec2(0.0f, icon_button_size)
                ))
                selected_object_ = base_object;
            ImGui::PopStyleVar();

            ++it;
            ImGui::PopID();

            ImGui::Separator();
        }
    }

    void GeoBoxApplication::draw_object_properties(
        ) {
        if (base_objects_.empty())
            return;

        for (const auto& base_object : base_objects_)
            base_object->draw_object_properties();
    }

    void GeoBoxApplication::draw_scene(
        ) {
        for (const auto& base_object : base_objects_) {
            if (base_object->visible())
                base_object->draw_scene(lighting_);
        }
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

        const auto object = std::make_shared<MeshObject>(
            get_filename(filename),
            my_colormaps_,
            mesh);

        double xyzmin[3], xyzmax[3];
        object->get_bbox(xyzmin, xyzmax);
        for (GEO::coord_index_t i = 0; i < 3; ++i) {
            xyzmin_[i] = std::min(xyzmin_[i], xyzmin[i]);
            xyzmax_[i] = std::max(xyzmax_[i], xyzmax[i]);
        }
        set_region_of_interest(
            xyzmin_[0], xyzmin_[1], xyzmin_[2],
            xyzmax_[0], xyzmax_[1], xyzmax_[2]);

        base_objects_.push_back(object);

        return true;
    }
}
