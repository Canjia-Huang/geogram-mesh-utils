//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "quad_motorcycle_block.h"
#include <cassert>
#include <geogram/mesh/mesh_io.h>

namespace
{
    const std::array<GEO::vec2i, 4> QUAD_LV_EXT_COORD = {
        {
            GEO::vec2i(0, -1),
            GEO::vec2i(1, 0),
            GEO::vec2i(0, 1),
            GEO::vec2i(-1, 0)
        }
    };
}

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
        std::vector<bool> prcessed_facets(mesh_.facets.nb(), false);

        std::vector<BlockFacet> stack;

        /* Initialize */
        {
            BlockFacet BF;
            BF.f = start_f;
            BF.coord = GEO::vec2i(0, 0);
            BF.lvs[0] = 0;
            BF.lvs[0] = 1;
            BF.lvs[0] = 2;
            BF.lvs[0] = 3; // use the lv order of facet start_f as the reference

            stack.push_back(BF);
            prcessed_facets[start_f] = true;
        }

        /* Start */
        while (!stack.empty()) {
            const auto BF = stack.back();
            stack.pop_back();
            block_facets_.push_back(BF);

            const auto f = BF.f;
            for (GEO::index_t lv = 0; lv < 4; ++lv) {
                const auto& f_lv = BF.lvs[lv];
                assert(f_lv < 4);

                if (mesh_fc_tagged_[mesh_.facets.corner(f, f_lv)] != GEO::NO_INDEX) // a wall
                    continue;

                const auto nf = mesh_.facets.adjacent(f, f_lv);
                if (nf == GEO::NO_FACET) // on border
                    continue;

                if (prcessed_facets[nf])
                    continue;

                BlockFacet nBF;
                nBF.f = nf;
                nBF.coord = BF.coord + QUAD_LV_EXT_COORD[lv];

                /* Match lv order
                 *      x--------x    0--------3
                 *      |   nf   |    |   f    |
                 *      |        |    |        |
                 *      x-------v0    1--------2
                 */
                const auto nlv = mesh_.facets.find_vertex(nf, mesh_.facets.vertex(f, (f_lv+1)%4));
                assert(nlv != GEO::NO_INDEX);
                nBF.lvs[nlv] = 2;
                nBF.lvs[(nlv+1)%4] = 3;
                nBF.lvs[(nlv+2)%4] = 0;
                nBF.lvs[(nlv+3)%4] = 1;

                stack.push_back(nBF);
                prcessed_facets[nf] = true;
            }
        }

        // DEBUG
        if constexpr (true) {
            GEO::Mesh mesh_out;
            mesh_out.copy(mesh_);
            GEO::Attribute<int> mesh_out_f_coord_x(mesh_out.facets.attributes(), "coord_x");
            GEO::Attribute<int> mesh_out_f_coord_y(mesh_out.facets.attributes(), "coord_y");
            GEO::vector<GEO::index_t> facets_to_delete(mesh_out.facets.nb(), 1);
            for (const auto& BF : block_facets_) {
                facets_to_delete[BF.f] = 0;
                mesh_out_f_coord_x[BF.f] = BF.coord.x;
                mesh_out_f_coord_y[BF.f] = BF.coord.y;
            }
            mesh_out.facets.delete_elements(facets_to_delete);
            GEO::mesh_save(mesh_out, "debug.geogram");
            throw std::logic_error("im here");
        }

        rebuild_ordered_facets();
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