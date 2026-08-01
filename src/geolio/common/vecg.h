//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/25.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_VECG_H
#define GEOLIO_VECG_H

#include <geogram/basic/geometry.h>

namespace geolio
{
    /**
     * @brief Computes the dot product of 2 vectors.
     * @details Multiplies the corresponding components of the two vectors and
     *          sums them, i.e. returns v1.x * v2.x + v1.y * v2.y.
     * @param[in] v1 The first vector.
     * @param[in] v2 The second vector.
     * @return The dot product (v1 . v2).
     */
    inline double dot(
        const GEO::vec2& v1,
        const GEO::vec2& v2
        ) {
        return v1.x*v2.x + v1.y*v2.y;
    }

    /**
     * @brief Computes the cross product of 2 vectors.
     * @details Returns the signed 2D cross product v1.x * v2.y - v1.y * v2.x,
     *          whose magnitude equals the area of the parallelogram spanned by
     *          the two vectors.
     * @param[in] v1 The first vector.
     * @param[in] v2 The second vector.
     * @return The cross product (v1 x v2).
     */
    inline double cross(
        const GEO::vec2& v1,
        const GEO::vec2& v2
        ) {
        return v1.x*v2.y - v1.y*v2.x;
    }
}

#endif //GEOLIO_VECG_H
