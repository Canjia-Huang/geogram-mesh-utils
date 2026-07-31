//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_SMOOTH_OPERATION_H
#define GEOLIO_SMOOTH_OPERATION_H

#include "base_operation.h"
#include <geogram/mesh/mesh_AABB.h>

namespace geolio
{
    class SmoothOperation : public BaseOperation {
    public:
        explicit SmoothOperation(MeshElementManager& mesh_element_manager);

        void perform_one_pass(GEO::index_t iterations_nb) const;

        bool ALLOW_SMOOTH_FIXED_EDGE_VERTICES = false;

    private:
        [[nodiscard]] bool is_perform_valid(GEO::index_t v) const;

        GEO::Mesh original_mesh_; // a copy of original input mesh
        GEO::MeshFacetsAABB original_mesh_facet_AABB_;
    };
}

#endif //GEOLIO_SMOOTH_OPERATION_H
