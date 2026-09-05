//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_QUAD_MOTORCYCLE_GRAPH_H
#define GEOLIO_QUAD_MOTORCYCLE_GRAPH_H
#include <geogram/mesh/mesh.h>
#include "quad_motorcycle_block.h"

namespace geolio
{
    class QuadMotorCycleGraph {
    public:
        explicit QuadMotorCycleGraph(const GEO::Mesh& mesh);

        ~QuadMotorCycleGraph();

        /**
         * Motorcycle complex variants.
         */
        enum QuadMotorCycleGraphComplexType {
            BASE_COMPLEX,        ///< Compute the base complex.
            MOTORCYCLE_COMPLEX   ///< Compute the motorcycle complex.
        };

        GEO::index_t compute(QuadMotorCycleGraphComplexType complex_type = BASE_COMPLEX);

    private:
        void find_all_singular_and_border_vertices();

        struct Fire {
            GEO::index_t d;
            GEO::index_t f;
            GEO::index_t lv;

            bool operator<(const Fire& other) const {
                return d > other.d;
            }
        };

        void ignite(std::priority_queue<Fire>& queue) const;

        GEO::index_t decompose_into_blocks();

        const std::string attribute_id_;

        const GEO::Mesh& mesh_; // Input quad mesh
        GEO::Attribute<GEO::index_t> mesh_fc_tagged_; // [4*f+lv] -> distance tag or GEO::NO_INDEX
        std::vector<bool> mesh_v_singular_; // [v] -> singular vertex
        std::vector<bool> mesh_v_border_; // [v] -> border vertex

        std::vector<QuadMotorCycleBlock> blocks_;
    };
}

#endif //GEOLIO_QUAD_MOTORCYCLE_GRAPH_H
