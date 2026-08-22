//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/18.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_APPLICATION_H
#define GEOLIO_APPLICATION_H

#include <geogram_gfx/gui/simple_mesh_application.h>
#include "colormap.h"
#include "geolio/geobox/object/base_object.h"

namespace geolio::geobox
{
    class GeoBoxApplication : public GEO::SimpleMeshApplication {
    public:
        GeoBoxApplication();

        void draw_gui() override;

    protected:
        // fills my_colormaps_; must run after the GL context is created
        // (the base's SimpleApplication::init_colormaps() is non-virtual and
        // fills the base's own member, so it is called explicitly here).
        void init_colormaps();

        void GL_initialize() override;

        // apply the Polyscope style once ImGui has been initialized
        // (SimpleApplication::ImGui_initialize forces gui:style=Light).
        void ImGui_initialize() override;

        void draw_controller_properties_window();

        void draw_controller_properties();

        void draw_viewer_properties() override;

        void draw_objects_properties();

        void draw_object_properties_window() override;

        void draw_object_properties() override;

        void draw_scene() override;

        void draw_about() override;

        void camera_focus(const BaseObject& object);

        bool load(const std::string& filepath) override;

        std::vector<geolio::geobox::ColormapInfo> my_colormaps_;

        // Current size of the Object Properties window, tracked so its
        // right edge can be kept flush against the viewport's right edge.
        ImVec2 object_properties_size_{0.0f, 0.0f};

        std::vector<std::shared_ptr<BaseObject>> objects_;
        std::weak_ptr<BaseObject> selected_object_;
    };
}

#endif //GEOLIO_APPLICATION_H
