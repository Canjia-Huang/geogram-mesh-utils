//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "local_operation_optimization_application.h"
#include <utility>

namespace geolio::geobox
{
    LocalOperationOptimizationApplication::LocalOperationOptimizationApplication(
        std::string application_name,
        const std::vector<std::shared_ptr<BaseObject>>& objects
        ) : BaseApplication(std::move(application_name)),
            objects_(objects)
    {
        menu_tooltip_ = "Triangular mesh optimization based on local operations.";
    }

    void LocalOperationOptimizationApplication::draw_window_contents(
        ) {

    }
}