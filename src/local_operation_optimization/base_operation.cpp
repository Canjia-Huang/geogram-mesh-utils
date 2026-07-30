//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "base_operation.h"
#include <geogram/mesh/mesh_io.h>
#include "geolio/log.h"

namespace geolio
{
    BaseOperation::BaseOperation(
        MeshElementManager& mesh_element_manager
        ) : manager_(mesh_element_manager),
            mesh_(mesh_element_manager.mesh),
            attribute_name_(generate_random_string(22))
    {
        /* Bind attributes */
        mesh_f_processed_.bind(mesh_.facets.attributes(), attribute_name_+":processed");
        mesh_f_processed_.fill(false);
    }

    BaseOperation::~BaseOperation(
        ) {
        /* Destroy attributes */
        if (mesh_f_processed_.is_bound())
            mesh_f_processed_.destroy();
    }

    bool BaseOperation::post_check(
        ) {
        {
            for (const auto& f : mesh_.facets) {
                if (!manager_.mesh_f_used[f])
                    continue;
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (const auto& v = mesh_.facets.vertex(f, lv);
                        !manager_.mesh_v_used[v]
                        ) {
                        // temp
                        LOG::DEBUG("v:{}", v);
                        GEO::Attribute<bool> mesh_v_label(mesh_.vertices.attributes(), "label");
                        mesh_v_label[v] = true;
                        manager_.clean_unused_elements(false);
                        GEO::mesh_save(mesh_, "debug.geogram");

                        return false;
                    }
                }
            }
        }

        return true;
    }
}
