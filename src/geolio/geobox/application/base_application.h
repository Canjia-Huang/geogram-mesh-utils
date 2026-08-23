//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_BASE_APPLICATION_H
#define GEOLIO_BASE_APPLICATION_H

#include <string>
#include <utility>

#include <geogram_gfx/third_party/imgui/imgui.h>
#include <geogram_gfx/imgui_ext/icon_font.h>

namespace geolio::geobox
{
    /**
     * @brief Base class for plugin-style sub-applications (e.g. mesh apps)
     *        embedded in the main GeoBox application.
     * @details Each sub-application contributes one menu entry
     *          (draw_menu()) and one floating window (draw_window()), whose
     *          title is application_name_ and whose visibility is shared
     *          between the menu checkbox and the window's close button.
     */
    class BaseApplication {
    public:
        explicit BaseApplication(std::string application_name) : application_name_(std::move(application_name)) {}

        virtual ~BaseApplication() = default;

        /**
         * @brief Draws this application's entries in the main menu bar.
         * @details Template: a checkbox item that toggles the floating
         *          window, then hands off to draw_menu_items() so derived
         *          classes can append their own entries.
         */
        virtual void draw_menu() {
            ImGui::MenuItem(application_name_.c_str(),
                nullptr,
                &window_visible_
            );
            // Show the menu description as a tooltip when the mouse hovers
            // over this menu item.
            if (ImGui::IsItemHovered() && !menu_tooltip_.empty())
                ImGui::SetTooltip("%s", menu_tooltip_.c_str());
        }

        /**
         * @brief Draws the application's floating window.
         * @details The window is titled after application_name_ and is drawn
         *          only while window_visible_ is true. Its close button
         *          clears window_visible_ (which also unchecks the menu
         *          item); the body is provided by draw_window_contents().
         */
        virtual void draw_window() {
            if (!window_visible_)
                return;

            const ImVec2 viewport_size = ImGui::GetMainViewport()->Size;
            ImGui::SetNextWindowSize(
                ImVec2(viewport_size.x * 0.3f, viewport_size.y * 0.5f),
                ImGuiCond_FirstUseEver
            );
            // Center the window on the viewport the first time it is shown;
            // afterwards the user may drag it anywhere.
            ImGui::SetNextWindowPos(
                ImGui::GetMainViewport()->GetCenter(),
                ImGuiCond_FirstUseEver,
                ImVec2(0.5f, 0.5f)
            );
            ImGui::SetNextWindowBgAlpha(0.6f);
            if (ImGui::Begin(application_name_.c_str(), &window_visible_))
                draw_window_contents();

            ImGui::End();
        }

    protected:
        /**
         * @brief Hook: derived classes draw the window body here.
         */
        virtual void draw_window_contents() = 0;

        std::string application_name_;
        std::string menu_tooltip_;
        bool window_visible_ = false;
    };
}

#endif //GEOLIO_BASE_APPLICATION_H
