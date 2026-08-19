//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_BASE_OBJECT_H
#define GEOLIO_BASE_OBJECT_H

namespace geolio::geobox
{
    class BaseObject {
    public:
        BaseObject() = default;

        virtual ~BaseObject() = default;

        virtual void draw(bool lighting) = 0;
    };
}

#endif //GEOLIO_BASE_OBJECT_H
