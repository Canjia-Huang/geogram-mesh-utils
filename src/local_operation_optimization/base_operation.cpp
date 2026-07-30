//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "base_operation.h"

namespace geolio
{
    BaseOperation::BaseOperation(
        MeshElementManager& mesh_element_manager
        ) : manager_(mesh_element_manager),
            mesh_(mesh_element_manager.mesh),
            attribute_name_(generate_random_string(22))
    {
        /* Bind attributes */
        mesh_f_processed.bind(mesh_.facets.attributes(), attribute_name_+":processed");
        mesh_f_processed.fill(false);
    }

    BaseOperation::~BaseOperation(
        ) {
        /* Destroy attributes */
        if (mesh_f_processed.is_bound())
            mesh_f_processed.destroy();
    }
}