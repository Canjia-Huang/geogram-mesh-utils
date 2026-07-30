//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_BASE_OPERATION_H
#define GEOLIO_BASE_OPERATION_H

#include "mesh_element_manager.h"

namespace geolio
{
    class BaseOperation {
    public:
        explicit BaseOperation(MeshElementManager& mesh_element_manager);

        ~BaseOperation();

    protected:
        MeshElementManager& manager_;
        const std::string attribute_name_; // Prevent anyone from using these attributes externally (unsafety).

        GEO::Mesh& mesh_;
        GEO::Attribute<bool> mesh_f_processed_; // f -> processed (just pre-allocated)
    };
}

#endif //GEOLIO_BASE_OPERATION_H
