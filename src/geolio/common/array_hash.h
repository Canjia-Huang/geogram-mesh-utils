//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_TUPLE_HASH_H
#define GEOLIO_TUPLE_HASH_H

#include <cstddef>
#include <iostream>

namespace geolio
{
    /**
     * @brief Combines a value into an existing hash seed.
     * @tparam T Type of the value to hash.
     * @param seed The running hash seed to be updated.
     * @param val The value to combine into the seed.
     *
     * This helper applies a standard hash-combine mixing step so multiple
     * values can be folded into one hash result.
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
         * @param arr The sorted array to hash.
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
         * @param arr The sorted array to hash.
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
