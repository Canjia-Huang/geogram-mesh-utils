//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_COLLAPSE_OPERATION_H
#define GEOLIO_COLLAPSE_OPERATION_H

#include "base_operation.h"

namespace geolio
{
    class CollapseOperation : public BaseOperation {
    public:
        explicit CollapseOperation(
            MeshElementManager& mesh_element_manager,
            double limit_edge_length);

        void perform_one_pass();

    private:
        [[nodiscard]] bool is_perform_valid(GEO::index_t f, GEO::index_t lv) const;

        void perform(GEO::index_t f, GEO::index_t lv,
                     GEO::index_t& disuse_v0, GEO::index_t& disuse_v1, GEO::index_t& disuse_v2,
                     GEO::index_t& disuse_f0, GEO::index_t& disuse_f1) const;

        void post_process(GEO::index_t v,
                          GEO::index_t disuse_v0, GEO::index_t disuse_v1, GEO::index_t disuse_v2,
                          GEO::index_t disuse_f0, GEO::index_t disuse_f1) const;

        const double limit_edge_length_;
    };
}

#endif //GEOLIO_COLLAPSE_OPERATION_H
