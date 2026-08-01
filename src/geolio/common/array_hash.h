//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_TUPLE_HASH_H
#define GEOLIO_TUPLE_HASH_H

#include <array>
#include <cstddef>
#include <functional>
#include <iostream>

namespace geolio
{
    /**
     * @brief Combines a value into an existing hash seed.
     * @details Applies the standard boost-style mixing step
     *          seed ^= hash(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2),
     *          so that multiple values can be folded into one hash result
     *          in a well-distributed way.
     * @tparam T Type of the value to hash.
     * @param[in, out] seed The running hash seed, updated in place.
     * @param[in] val The value to combine into the seed.
     */
    template <typename T>
    void hash_combine(std::size_t& seed, const T& val) {
        std::hash<T> hasher;
        seed ^= hasher(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }

    /**
     * @brief Hash functor for a sorted 3-element array.
     * @tparam T Element type of the array.
     *
     * @note The input array must be sorted in advance.
     */
    template <typename T>
    struct Array3Hash {
        /**
         * @brief Computes the hash value for a 3-element array.
         * @details Seeds a hash from zero and folds the three elements into it
         *          one by one through hash_combine.
         * @param[in] arr The sorted array to hash.
         * @return The combined hash value.
         */
        std::size_t operator()(const std::array<T, 3>& arr) const {
            std::size_t seed = 0;
            hash_combine(seed, arr[0]);
            hash_combine(seed, arr[1]);
            hash_combine(seed, arr[2]);
            return seed;
        }
    };

    /**
     * @brief Hash functor for a sorted 4-element array.
     * @tparam T Element type of the array.
     *
     * @note The input array must be sorted in advance.
     */
    template <typename T>
    struct Array4Hash {
        /**
         * @brief Computes the hash value for a 4-element array.
         * @details Seeds a hash from zero and folds the four elements into it
         *          one by one through hash_combine.
         * @param[in] arr The sorted array to hash.
         * @return The combined hash value.
         */
        std::size_t operator()(const std::array<T, 4>& arr) const {
            std::size_t seed = 0;
            hash_combine(seed, arr[0]);
            hash_combine(seed, arr[1]);
            hash_combine(seed, arr[2]);
            hash_combine(seed, arr[3]);
            return seed;
        }
    };
}

#endif //GEOLIO_TUPLE_HASH_H
