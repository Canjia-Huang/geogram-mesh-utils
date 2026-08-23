//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/23.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LOCAL_OPERATION_OPTIMIZATION_APPLICATION_H
#define GEOLIO_LOCAL_OPERATION_OPTIMIZATION_APPLICATION_H

#include "base_application.h"
#include <geolio/geobox/object/base_object.h>

namespace geolio::geobox
{
    class LocalOperationOptimizationApplication : public BaseApplication {
    public:
        LocalOperationOptimizationApplication(
            std::string application_name,
            const std::vector<std::shared_ptr<BaseObject>>& objects);

    protected:
        void draw_window_contents() override;

        const std::vector<std::shared_ptr<BaseObject>>& objects_;
    };
}

#endif //GEOLIO_LOCAL_OPERATION_OPTIMIZATION_APPLICATION_H
