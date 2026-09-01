//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_DISTANCE_FIELD_H
#define GEOLIO_DISTANCE_FIELD_H

namespace geolio
{
    class DistanceField {
    public:
        DistanceField() = default;

        virtual ~DistanceField() = default;

        [[nodiscard]] virtual double query(double x, double y, double z) const = 0;
    };
}

#endif //GEOLIO_DISTANCE_FIELD_H
