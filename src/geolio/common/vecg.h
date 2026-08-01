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
     * \brief Computes the dot product of 2 vectors
     * \param[in] v1 the first vector
     * \param[in] v2 the second vector
     * \return the dot product (\p v1 . \p v2)
     */
    inline double dot(
        const GEO::vec2& v1,
        const GEO::vec2& v2
        ) {
        return v1.x*v2.x + v1.y*v2.y;
    }

    /**
     * \brief Computes the cross product of 2 vectors
     * \param[in] v1 the first vector
     * \param[in] v2 the second vector
     * \return the cross product (\p v1 x \p v2)
     */
    inline double cross(
        const GEO::vec2& v1,
        const GEO::vec2& v2
        ) {
        return v1.x*v2.y - v1.y*v2.x;
    }
}

#endif //GEOLIO_VECG_H
