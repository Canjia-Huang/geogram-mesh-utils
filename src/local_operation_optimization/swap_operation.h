//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_SWAP_OPERATION_H
#define GEOLIO_SWAP_OPERATION_H

#include "base_operation.h"

namespace geolio
{
    class SwapOperation : public BaseOperation {
    public:
        explicit SwapOperation(MeshElementManager& mesh_element_manager);

        void perform_one_pass();

    private:
        [[nodiscard]] bool is_perform_valid(GEO::index_t f, GEO::index_t lv) const;

        void perform(GEO::index_t f, GEO::index_t lv) const;

        void post_process(GEO::index_t f, GEO::index_t lv) const;
    };
}

#endif //GEOLIO_SWAP_OPERATION_H
