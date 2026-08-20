//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/19.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_BASE_OBJECT_H
#define GEOLIO_BASE_OBJECT_H

#include <geolio/common/utils.h>

namespace geolio::geobox
{
    class BaseObject {
    public:
        explicit BaseObject(const std::string& name) : name_(name), unique_id_(generate_random_string(22)), visible_(true) {}

        virtual ~BaseObject() = default;

        [[nodiscard]] const std::string& name() const { return name_; }

        [[nodiscard]] bool visible() const { return visible_; }

        void set_visible(const bool visible) { visible_ = visible; }

        virtual void draw_object_properties() = 0;

        virtual void draw_scene(bool lighting) = 0;

    protected:
        const std::string name_;
        const std::string unique_id_;
        bool visible_;
    };
}

#endif //GEOLIO_BASE_OBJECT_H
