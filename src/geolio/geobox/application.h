//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/18.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_APPLICATION_H
#define GEOLIO_APPLICATION_H

#include <geogram_gfx/gui/simple_mesh_application.h>
#include "geolio/geobox/object/base_object.h"

namespace geolio::geobox
{
    class GeoBoxApplication : public GEO::SimpleMeshApplication {
    public:
        GeoBoxApplication();

        void draw_gui() override;

    protected:
        void draw_viewer_properties() override;

        void draw_controller_properties();

        void draw_object_properties() override;

        void draw_scene() override;

        bool load(const std::string& filename) override;

        std::vector<std::shared_ptr<BaseObject>> base_objects_;
    };
}

#endif //GEOLIO_APPLICATION_H
