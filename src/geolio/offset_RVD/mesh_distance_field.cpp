//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "mesh_distance_field.h"

namespace geolio
{
    MeshDistanceField::MeshDistanceField(
        GEO::Mesh& mesh
        ) : mesh_(mesh)
    {
        AABB_.initialize(mesh_);
    }

    double MeshDistanceField::query(
        const double x,
        const double y,
        const double z
        ) const {
        return AABB_.squared_distance(GEO::vec3(x, y, z));;
    }
}
