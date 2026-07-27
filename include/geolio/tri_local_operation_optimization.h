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

        void collapse_edges(double limit_length);

        void swap_edges() const;

        void smooth_vertices(GEO::index_t iterations_nb) const;

    private:
        [[nodiscard]] double compute_average_mesh_edge_length() const;

        [[nodiscard]] GEO::index_t require_a_new_vertex();

        void disuse_a_vertex(GEO::index_t v);

        [[nodiscard]] GEO::index_t require_a_new_facet();

        void disuse_a_facet(GEO::index_t f);

        void allocate_new_vertices();

        void allocate_new_facets();

        void clean_unused_elements() const;

        GEO::Mesh& mesh_;
        double target_length_;

        std::vector<char> mesh_v_used_;
        std::vector<char> mesh_f_used_;
        std::vector<GEO::index_t> free_vertices_;
        std::vector<GEO::index_t> free_facets_;
    };
}

#endif //GEOLIO_TRI_LOCAL_OPERATION_OPTIMIZATION_H
