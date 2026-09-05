//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_HEX_MOTORCYCLE_BLOCK_H
#define GEOLIO_HEX_MOTORCYCLE_BLOCK_H
#include <geogram/mesh/mesh.h>

namespace geolio
{
    class HexMotorCycleBlock {
    public:
        /**
         * Creates a block builder on a hexahedral mesh.
         *
         * @param[in] mesh Input mesh.
         * where facet `(c, lf)` is paired with facet `(nc, nlf)`.
         * @param[in] mesh_cf_tagged Cell-facet tag map: `[8*c + lf]` is a tag value or `GEO::NO_INDEX`
         * for untagged facets.
         */
        HexMotorCycleBlock(
            const GEO::Mesh& mesh,
            const GEO::Attribute<GEO::index_t>& mesh_cf_tagged);

        /**
         * Flood-fills connected cells from a start cell and builds an ordered block layout.
         *
         * Tagged facets in `M_cf_tagged_` are treated as stopping boundaries for propagation.
         *
         * @param[in] start_c Start cell index.
         */
        void flood_fill_cells(GEO::index_t start_c);

        /**
         * Gets the ordered block cells.
         *
         * @return Read-only reference to internal ordered cells.
         */
        [[nodiscard]] const auto& cells() const { return cells_; }

        /**
         * Gets the mesh vertex index of a block corner in the current local cell orientation.
         *
         * @param[in] lv Local corner id in `[0, 7]`.
         * @return Vertex index in `M_` corresponding to the queried corner.
         */
        [[nodiscard]] GEO::index_t cell_corner_vertex(GEO::index_t lv) const;

    private:
        /**
         * Reorders collected cells into a dense `(x, y, z)` layout and updates block dimensions.
         */
        void rebuild_ordered_cells();

        const GEO::Mesh& mesh_;
        const GEO::Attribute<GEO::index_t>& mesh_cf_tagged_; // [8*c+lf] -> tagged (distance) or not (GEO::NO_INDEX)

        struct BlockCell {
            GEO::index_t c{GEO::NO_INDEX};
            GEO::vec3i coord;
            std::array<GEO::index_t, 6> lfs{
                GEO::NO_INDEX, GEO::NO_INDEX, GEO::NO_INDEX, GEO::NO_INDEX, GEO::NO_INDEX, GEO::NO_INDEX
                }; // same as the start_c's lf
        };

        /*
             +Z                4-------6
             |                /|      /|
             o --- +Y        5-------7 |
            /                | 0-----|-2    (x, y, z) -> cells_[x + y*len_x + z*len_x*len_y]
          +X                 |/      |/
                             1-------3
         */
        std::vector<BlockCell> cells_;
        GEO::index_t len_x_{GEO::NO_INDEX}, len_y_{GEO::NO_INDEX}, len_z_{GEO::NO_INDEX};
    };
}

#endif //GEOLIO_HEX_MOTORCYCLE_BLOCK_H
