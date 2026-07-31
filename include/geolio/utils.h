//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/29.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_UTILS_H
#define GEOLIO_UTILS_H

#include <random>

namespace geolio
{
    inline std::string generate_random_string(
        const std::size_t length
        ) {
        static const std::string CHARACTERS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

        static std::random_device rd;
        static std::mt19937 generator(rd());

        std::uniform_int_distribution<std::size_t> distribution(0, CHARACTERS.size() - 1);

        std::string random_string;
        random_string.reserve(length);
        for (std::size_t i = 0; i < length; ++i)
            random_string += CHARACTERS[distribution(generator)];

        return random_string;
    }
}

#endif //GEOLIO_UTILS_H
