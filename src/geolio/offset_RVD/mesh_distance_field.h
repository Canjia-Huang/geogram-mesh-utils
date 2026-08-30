//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_MESH_DISTANCE_FIELD_H
#define GEOLIO_MESH_DISTANCE_FIELD_H
#include "distance_field.h"
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_AABB.h>

namespace geolio
{
    class MeshDistanceField : public DistanceField {
    public:
        explicit MeshDistanceField(GEO::Mesh& mesh);

        [[nodiscard]] double query(double x, double y, double z) const override;

    private:
        GEO::Mesh& mesh_;
        GEO::MeshFacetsAABB AABB_;
    };
}

#endif //GEOLIO_MESH_DISTANCE_FIELD_H
