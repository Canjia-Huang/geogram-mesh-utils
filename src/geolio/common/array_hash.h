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
     * @brief Hash functor for a fixed-size array.
     * @tparam T Element type of the array.
     * @tparam N Number of elements in the array.
     * @details Seeds the hash from zero and folds every element into the
     *          running value using hash_combine. This allows std::array
     *          instances to be used as keys in unordered containers.
     */
    template <typename T, int N>
    struct ArrayHash {
        /**
         * @brief Computes the hash value for a fixed-size array.
         * @details Initializes the seed to zero and combines each element in
         *          order into the final hash.
         * @param[in] arr The array to hash.
         * @return The combined hash value.
         */
        std::size_t operator()(const std::array<T, N>& arr) const {
            std::size_t seed = 0;
            for (GEO::index_t i = 0; i < N; ++i)
                hash_combine(seed, arr[i]);
            return seed;
        }
    };
}

#endif //GEOLIO_TUPLE_HASH_H
