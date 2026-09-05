//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_QUAD_MOTORCYCLE_BLOCK_H
#define GEOLIO_QUAD_MOTORCYCLE_BLOCK_H
#include <geogram/mesh/mesh.h>

namespace geolio
{
    class QuadMotorCycleBlock {
    public:
        QuadMotorCycleBlock(
            const GEO::Mesh& mesh,
            const GEO::Attribute<GEO::index_t>& mesh_fc_tagged);

        void flood_fill_facets(GEO::index_t start_f);

        [[nodiscard]] const auto& block_facets() const { return block_facets_; }

        [[nodiscard]] GEO::index_t facet_corner_vertex(GEO::index_t lv) const;

    private:
        void rebuild_ordered_facets();

        const GEO::Mesh& mesh_;
        const GEO::Attribute<GEO::index_t>& mesh_fc_tagged_; // [4*f+lv] -> distance tag or GEO::NO_INDEX

        struct BlockFacet {
            GEO::index_t f{GEO::NO_INDEX};
            GEO::vec2i coord;
            std::array<GEO::index_t, 4> lvs{
                GEO::NO_INDEX, GEO::NO_INDEX, GEO::NO_INDEX, GEO::NO_INDEX
                }; // same as the start_f's lv
        };

        /**
         *   +Y           3 --- 2
         *   |            |     |    (x, y) -> block_facets_[x + y*len_x]
         *   o --- +X     0 --- 1
         */
        std::vector<BlockFacet> block_facets_;
        GEO::index_t len_x_{GEO::NO_INDEX}, len_y_{GEO::NO_INDEX};
    };
}

#endif //GEOLIO_QUAD_MOTORCYCLE_BLOCK_H
