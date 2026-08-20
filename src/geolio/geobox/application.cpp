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
