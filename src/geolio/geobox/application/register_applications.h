//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/29.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_APPLICATIONS_H
#define GEOLIO_APPLICATIONS_H

#include "application_registry.h"
#include "local_operation_optimization_application.h"
#include <geolio/common/utils.h>

namespace geolio::geobox
{
    /**
     * @brief Registers every built-in sub-application (BaseApplication
     *        subclasses) into the application registry.
     * @details Mirrors register_additional_MeshIOHandlers() from
     *          geolio/io/io.h: each concrete application is declared here
     *          with its stable id and display name, and GeoBoxApplication
     *          instantiates all of them by iterating the registry. Adding a
     *          new sub-application = include its header here and add one
     *          geolio_register_BaseApplication_creator() line; the
     *          GeoBoxApplication constructor no longer needs to change.
     */
    inline void register_additional_Applications() {
        geolio_register_BaseApplication_creator(
            LocalOperationOptimizationApplication,
            generate_random_string(22),
            "Mesh optimization");
    }
}

#endif //GEOLIO_APPLICATIONS_H
