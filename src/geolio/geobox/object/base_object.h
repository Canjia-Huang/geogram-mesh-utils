//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_BASE_OBJECT_H
#define GEOLIO_BASE_OBJECT_H

#include <geolio/common/utils.h>
#include <geolio/common/parse_filepath.h>
#include <geogram_gfx/imgui_ext/imgui_ext.h>
#include <geogram_gfx/third_party/imgui/imgui.h>

namespace geolio::geobox
{
    class BaseObject {
    public:
        explicit BaseObject(
            const std::string& filepath
            ) : filepath_(filepath),
                name_(get_filename(filepath)),
                unique_id_(generate_random_string(22))
        {}

        virtual ~BaseObject() = default;

        [[nodiscard]] const std::string& name() const { return name_; }

        [[nodiscard]] bool visible() const { return visible_; }

        void set_visible(const bool visible) { visible_ = visible; }

        virtual void draw_object_properties(
            ) {
            if (ImGui::Button("Reload", ImVec2(-1, 0)))
                reload();
        }

        virtual void draw_scene(bool lighting) = 0;

        virtual void reload() = 0;

        virtual void get_bbox(double* xyzmin, double* xyzmax) const = 0;

    protected:
        const std::string filepath_;
        const std::string name_;
        const std::string unique_id_;
        bool visible_ = true;
    };
}

#endif //GEOLIO_BASE_OBJECT_H
