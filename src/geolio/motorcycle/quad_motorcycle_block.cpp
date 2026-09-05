//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "quad_motorcycle_block.h"
#include <cassert>

namespace geolio
{
    QuadMotorCycleBlock::QuadMotorCycleBlock(
        const GEO::Mesh& mesh,
        const GEO::Attribute<GEO::index_t>& mesh_fc_tagged
        ) : mesh_(mesh),
            mesh_fc_tagged_(mesh_fc_tagged)
    {
        assert([&]() {
                for (const auto& f : mesh_.facets) {
                    if (mesh_.facets.nb_vertices(f) != 4)
                        return false;
                }
                return true;
            }()); // check all-quad mesh
        assert(mesh_fc_tagged_.is_bound());
    }

    void QuadMotorCycleBlock::flood_fill_facets(
        const GEO::index_t start_f
        ) {
        // TODO
    }

   GEO::index_t QuadMotorCycleBlock::facet_corner_vertex(
       const GEO::index_t lv
       ) const {
        // TODO
    }

    void QuadMotorCycleBlock::rebuild_ordered_facets(
        ) {
        // TODO
    }
}