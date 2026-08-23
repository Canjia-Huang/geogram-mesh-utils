//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LOCAL_OPERATION_OPTIMIZATION_APPLICATION_H
#define GEOLIO_LOCAL_OPERATION_OPTIMIZATION_APPLICATION_H

#include "base_application.h"
#include <geolio/geobox/object/base_object.h>
#include <geogram/mesh/mesh.h>

namespace geolio::geobox
{
    class LocalOperationOptimizationApplication : public BaseApplication {
    public:
        LocalOperationOptimizationApplication(
            std::string application_name,
            const std::vector<std::shared_ptr<BaseObject>>& objects);

    protected:
        void draw_window_contents() override;

        template <GEO::index_t DIM>
        static void perform(GEO::Mesh& mesh);

        const std::vector<std::shared_ptr<BaseObject>>& objects_;

        /** The mesh object currently selected in the combo box. */
        std::weak_ptr<BaseObject> selected_mesh_object_;
    };
}

#endif //GEOLIO_LOCAL_OPERATION_OPTIMIZATION_APPLICATION_H
