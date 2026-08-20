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
    {}

    void GeoBoxApplication::draw_gui(
        ) {
        draw_menu_bar();
        draw_controller_properties_window();
        // draw_viewer_properties_window();
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

    void GeoBoxApplication::init_colormaps(
        ) {
        // The base's SimpleApplication::init_colormaps() (non-virtual) already
        // created the 10 colormap textures into its own colormaps_ member
        // during GL_initialize(). Reuse those texture IDs instead of creating a
        // second set of GL textures, so no extra GL state is touched at startup.
        my_colormaps_.clear();
        my_colormaps_.reserve(colormaps_.size());
        for (const auto& cm : colormaps_) {
            geolio::geobox::ColormapInfo info;
            info.texture = cm.texture;
            info.name = cm.name;
            my_colormaps_.push_back(info);
        }
    }

    void GeoBoxApplication::GL_initialize(
        ) {
        // The base's SimpleApplication::init_colormaps() is non-virtual and
        // fills its own colormaps_ member, so call ours explicitly to fill
        // my_colormaps_ (needs a GL context, hence this override).
        SimpleMeshApplication::GL_initialize();
        init_colormaps();
    }

    void GeoBoxApplication::ImGui_initialize(
        ) {
        SimpleMeshApplication::ImGui_initialize();
        set_style("Light");

        // set_background_color(GEO::vec4f(0.08f, 0.12f, 0.22f, 1.0f)); // dark blue
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
        // the glyph still fits and stays centered inside it. FramePadding.y must
        // stay 0: it becomes the button's text baseline, and a nonzero value
        // makes SameLine() shift the following Selectable down by that amount,
        // so its text/highlight would stick out below the buttons.
        const ImVec2 icon_button_padding(
            std::max(0.0f, (icon_button_size - icon_text_width) * 0.5f),
            0.0f);

        // Master row: actions that apply to all objects.
        // Give the master row a distinct background so it stands out from object rows.
        ImGui::GetWindowDrawList()->AddRectFilled(
            ImGui::GetCursorScreenPos(),
            ImVec2(
                ImGui::GetCursorScreenPos().x + ImGui::GetContentRegionAvail().x,
                ImGui::GetCursorScreenPos().y + icon_button_size),
            ImGui::GetColorU32(ImVec4(0.35f, 0.61f, 0.80f, 0.8f)));
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, icon_button_padding);

        // Show / hide all objects.
        const bool all_visible = std::all_of(
            objects_.begin(), objects_.end(),
            [](const std::shared_ptr<BaseObject>& object) {
                return object->visible();
            });
        if (ImGui::Button(
            GEO::icon_UTF8(all_visible ? "eye" : "eye-slash").c_str(),
            ImVec2(icon_button_size, icon_button_size)
            )) {
            for (const auto& object : objects_)
                object->set_visible(!all_visible);
        }

        ImGui::SameLine();
        // Focus the camera on all objects.
        if (ImGui::Button(
            GEO::icon_UTF8("camera").c_str(),
            ImVec2(icon_button_size, icon_button_size)
            ) && !objects_.empty()) {
            home();
            double xyzmin[3] = {
                std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max()
            };
            double xyzmax[3] = {
                -std::numeric_limits<double>::max(),
                -std::numeric_limits<double>::max(),
                -std::numeric_limits<double>::max()
            };
            for (const auto& object : objects_) {
                double bmin[3], bmax[3];
                object->get_bbox(bmin, bmax);
                for (GEO::coord_index_t i = 0; i < 3; ++i) {
                    xyzmin[i] = std::min(xyzmin[i], bmin[i]);
                    xyzmax[i] = std::max(xyzmax[i], bmax[i]);
                }
            }
            set_region_of_interest(
                xyzmin[0], xyzmin[1], xyzmin[2],
                xyzmax[0], xyzmax[1], xyzmax[2]);
        }

        ImGui::SameLine();
        // Delete all objects.
        if (ImGui::Button(
            GEO::icon_UTF8("xmark").c_str(),
            ImVec2(icon_button_size, icon_button_size)
            )) {
            objects_.clear();
            selected_object_.reset();
        }
        ImGui::PopStyleVar();

        ImGui::SameLine();
        // Clicking the rest of the row clears the current object selection.
        ImGui::PushStyleVar(
            ImGuiStyleVar_SelectableTextAlign, ImVec2(0.0f, 0.5f));
        if (ImGui::Selectable(
            "Clear Selection", false, 0, ImVec2(0.0f, icon_button_size)
            ))
            selected_object_.reset();
        ImGui::PopStyleVar();

        ImGui::Separator();

        for (auto it = objects_.begin(); it != objects_.end();) {
            const auto& base_object = *it;

            ImGui::PushID(base_object.get());

            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, icon_button_padding);
            if (ImGui::Button(
                GEO::icon_UTF8(
                    base_object->visible() ? "eye" : "eye-slash").c_str(),
                ImVec2(icon_button_size, icon_button_size)
                ))
                base_object->set_visible(!base_object->visible());

            ImGui::SameLine();
            // Focus the camera on this object's bounding box.
            if (ImGui::Button(
                GEO::icon_UTF8("camera").c_str(),
                ImVec2(icon_button_size, icon_button_size)
                ))
                camera_focus(*base_object);

            ImGui::SameLine();
            if (ImGui::Button(
                GEO::icon_UTF8("xmark").c_str(),
                ImVec2(icon_button_size, icon_button_size)
                )) {
                ImGui::PopStyleVar();
                it = objects_.erase(it);
                ImGui::PopID();

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

    void GeoBoxApplication::draw_object_properties_window(
        ) {
        if (selected_object_.expired())
            return;

        constexpr float WINDOWS_WIDTH = 0.2f;

        const ImVec2 viewport_size = ImGui::GetMainViewport()->Size;
        if (object_properties_size_.x <= 0.0f)
            object_properties_size_ = ImVec2(
                viewport_size.x * WINDOWS_WIDTH, viewport_size.y * 0.5f);

        // Anchor the window's right edge to the viewport's right edge, so it
        // stays docked in the top-right corner when the viewport is resized
        // while keeping the tracked (constant) width and height.
        ImGui::SetNextWindowPos(
            ImVec2(
                viewport_size.x - object_properties_size_.x,
                ImGui::GetFrameHeight()),
            ImGuiCond_Always
        );
        ImGui::SetNextWindowSize(object_properties_size_, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowBgAlpha(0.6f);
        if (ImGui::Begin(
            "Object Properties", nullptr,
            ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoMove)) {
            draw_object_properties();
            object_properties_size_ = ImGui::GetWindowSize();
        }

        ImGui::End();
    }

    void GeoBoxApplication::draw_object_properties(
        ) {
        const auto selected_object = selected_object_.lock();
        if (!selected_object)
            return;

        selected_object->draw_object_properties();
    }

    void GeoBoxApplication::draw_scene(
        ) {
        for (const auto& base_object : objects_) {
            if (base_object->visible())
                base_object->draw_scene(lighting_);
        }
    }

    void GeoBoxApplication::camera_focus(
        const BaseObject& object
        ) {
        home();

        double xyzmin[3];
        double xyzmax[3];
        object.get_bbox(xyzmin, xyzmax);
        set_region_of_interest(
            xyzmin[0], xyzmin[1], xyzmin[2],
            xyzmax[0], xyzmax[1], xyzmax[2]);
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

        const auto object_ptr = std::make_shared<MeshObject>(
            get_filename(filename),
            my_colormaps_,
            mesh);

        objects_.push_back(object_ptr);

        camera_focus(*object_ptr);

        return true;
    }
}
