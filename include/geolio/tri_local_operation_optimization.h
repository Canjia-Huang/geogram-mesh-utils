//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_TRI_LOCAL_OPERATION_OPTIMIZATION_H
#define GEOLIO_TRI_LOCAL_OPERATION_OPTIMIZATION_H

#include <geogram/mesh/mesh.h>

namespace geolio
{
    class TriLocalOperationOptimization {
    public:
        explicit TriLocalOperationOptimization(
            GEO::Mesh& mesh,
            double target_length = -1);

        void split_edges(double limit_length);

    private:
        double compute_average_mesh_edge_length() const;

        void allocate_new_vertices();

        void allocate_new_facets();

        void clean_unused_elements() const;

        GEO::Mesh& mesh_;
        double target_length_;

        std::vector<GEO::index_t> free_vertices_;
        std::vector<GEO::index_t> free_facets_;
    };
}

#endif //GEOLIO_TRI_LOCAL_OPERATION_OPTIMIZATION_H
