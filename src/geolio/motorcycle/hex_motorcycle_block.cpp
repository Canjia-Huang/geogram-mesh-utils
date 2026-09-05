//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "hex_motorcycle_block.h"
#include <cassert>
#include <geogram/mesh/mesh_io.h>
#include <stack>
#include <geolio/mesh/hex_descriptor.h>
#include <geolio/mesh/hex_operations.h>

namespace
{
    const std::array<GEO::vec3i, 6> HEX_LF_EXT_COORD = {
        {
            GEO::vec3i(-1, 0, 0),
            GEO::vec3i(1, 0, 0),
            GEO::vec3i(0, -1, 0),
            GEO::vec3i(0, 1, 0),
            GEO::vec3i(0, 0, -1),
            GEO::vec3i(0, 0, 1)
        }
    };
}

namespace geolio
{
    HexMotorCycleBlock::HexMotorCycleBlock(
        const GEO::Mesh& mesh,
        const GEO::Attribute<GEO::index_t>& mesh_cf_tagged
        ) : mesh_(mesh),
            mesh_cf_tagged_(mesh_cf_tagged)
    {
        assert(std::all_of(
            mesh.cells.cell_type_ptr(0),
            mesh.cells.cell_type_ptr(0)+mesh.cells.nb(),
            [&](const auto cell_type) { return cell_type == GEO::MESH_HEX; })); // check all-hex mesh
        assert(mesh_cf_tagged_.is_bound());
    }

    void HexMotorCycleBlock::flood_fill_cells(
        const GEO::index_t start_c
        ) {
        // LOG::TRACE("{}({})", __FUNCTION__, start_c);

        std::vector<bool> processed_cells(mesh_.cells.nb(), false);

        std::stack<BlockCell> stack;

        /* Start */
        {
            BlockCell BC;
            BC.c = start_c;
            BC.coord = GEO::vec3i(0, 0, 0);
            BC.lfs[0] = 0;
            BC.lfs[1] = 1;
            BC.lfs[2] = 2;
            BC.lfs[3] = 3;
            BC.lfs[4] = 4;
            BC.lfs[5] = 5; // use the lf order of cell start_c as the reference

            stack.push(BC);
            processed_cells[start_c] = true;
        }

        while (!stack.empty()) {
            const auto BC = stack.top();
            stack.pop();
            cells_.push_back(BC);

            const auto c = BC.c;
            for (GEO::index_t lf = 0; lf < 6; ++lf) {
                const auto& c_lf = BC.lfs[lf];
                assert(c_lf < 6);

                if (mesh_cf_tagged_[8*c+c_lf] != GEO::NO_INDEX) // a wall
                    continue;

                const auto nc = mesh_.cells.adjacent(c, c_lf);
                if (nc == GEO::NO_CELL) // on border
                    continue;
                const auto nlf = geolio::find_hex_facet(
                    mesh_,
                    nc,
                    mesh_.cells.facet_vertex(c, c_lf, 2),
                    mesh_.cells.facet_vertex(c, c_lf, 1),
                    mesh_.cells.facet_vertex(c, c_lf, 0));
                assert(nlf != GEO::NO_INDEX);

                if (processed_cells[nc])
                    continue;

                BlockCell nBC;
                nBC.c = nc;
                nBC.coord = BC.coord + HEX_LF_EXT_COORD[lf];

                /* Match lf order
                 *       x-------x                      4-------6
                 *      /|  nc  /|                     /|   c  /|
                 *     x-------x |             clf    5-------7 |     oppo_clf
                 *     | x-----|-v0 <-- nlf    lf --> | 0-----|-2 <-- oppo_lf
                 *     |/      |/                     |/      |/
                 *     x-------x                      1-------3
                 */
                const auto nc_v0 = mesh_.cells.facet_vertex(nc, nlf, 0);
                GEO::index_t c_lv0 = GEO::NO_INDEX; // c's c_lv0 is matched to nc's v0
                for (GEO::index_t i = 0; i < 4; ++i) {
                    if (mesh_.cells.facet_vertex(c, c_lf, i) == nc_v0) {
                        const auto& f_lv0 = geolio::HEX_LF_INCIDENT_LV[c_lf][i];
                        const auto& f_lv1 = geolio::HEX_LF_INCIDENT_LV[c_lf][(i+1)%4];
                        const auto& f_lv3 = geolio::HEX_LF_INCIDENT_LV[c_lf][(i+3)%4];
                        for (const auto& adj_v : geolio::HEX_LV_ADJACENT_LV[f_lv0]) {
                            if (adj_v != f_lv1 && adj_v != f_lv3) {
                                c_lv0 = adj_v;
                                break;
                            }
                        }
                        break;
                    }
                }
                assert(c_lv0 != GEO::NO_INDEX);

                const auto& c_lfs = geolio::HEX_LV_INCIDENT_LF[c_lv0];
                const auto oppo_lf = geolio::HEX_LF_OPPOSITE_LF[c_lf];
                GEO::index_t c_lfs_i = GEO::NO_INDEX;
                for (GEO::index_t i = 0; i < 3; ++i) {
                    if (c_lfs[i] == oppo_lf) {
                        c_lfs_i = i;
                        break;
                    }
                }
                assert(c_lfs_i != GEO::NO_INDEX);

                const auto& nc_lfs = geolio::HEX_LV_INCIDENT_LF[geolio::HEX_LF_INCIDENT_LV[nlf][0]];
                GEO::index_t nc_lfs_i = GEO::NO_INDEX;
                for (GEO::index_t i = 0; i < 3; ++i) {
                    if (nc_lfs[i] == nlf) {
                        nc_lfs_i = i;
                        break;
                    }
                }
                assert(nc_lfs_i != GEO::NO_INDEX);

                std::array<GEO::index_t, 6> clf_to_lf{
                    GEO::NO_INDEX, GEO::NO_INDEX, GEO::NO_INDEX, GEO::NO_INDEX, GEO::NO_INDEX, GEO::NO_INDEX};
                for (GEO::index_t i = 0; i < 6; ++i)
                    clf_to_lf[BC.lfs[i]] = i;
                assert(std::ranges::all_of(clf_to_lf, [&](const GEO::index_t i) { return clf_to_lf[i] < 6; }));

                for (GEO::index_t i = 0; i < 3; ++i) {
                    const auto c_lfs_lf = c_lfs[(c_lfs_i+i)%3];
                    const auto nc_lfs_lf = nc_lfs[(nc_lfs_i+i)%3];
                    nBC.lfs[clf_to_lf[c_lfs_lf]] = nc_lfs_lf;
                    nBC.lfs[clf_to_lf[geolio::HEX_LF_OPPOSITE_LF[c_lfs_lf]]] = geolio::HEX_LF_OPPOSITE_LF[nc_lfs_lf];
                }
                assert(std::ranges::all_of(nBC.lfs, [&](const GEO::index_t i) { return nBC.lfs[i] != GEO::NO_INDEX; }));
                assert(nBC.lfs[geolio::HEX_LF_OPPOSITE_LF[lf]] == nlf);

                stack.push(nBC);
                processed_cells[nc] = true;
            }
        }

        // DEBUG
        {
            // GEO::Mesh M_out;
            // M_out.copy(M_);
            // M_out.facets.clear();
            // M_out.edges.clear();
            // GEO::Attribute<int> M_out_c_coord_x(M_out.cells.attributes(), "coord_x");
            // GEO::Attribute<int> M_out_c_coord_y(M_out.cells.attributes(), "coord_y");
            // GEO::Attribute<int> M_out_c_coord_z(M_out.cells.attributes(), "coord_z");
            // GEO::vector<GEO::index_t> cells_to_delete(M_out.cells.nb(), 1);
            // for (const auto& BC : cells_) {
            //     cells_to_delete[BC.c] = 0;
            //     M_out_c_coord_x[BC.c] = BC.coord.x;
            //     M_out_c_coord_y[BC.c] = BC.coord.y;
            //     M_out_c_coord_z[BC.c] = BC.coord.z;
            // }
            // M_out.cells.delete_elements(cells_to_delete);
            // GEO::mesh_save(M_out, "debug.geogram");
            // THROW_RUNTIME_ERROR("im here");
        }

        rebuild_ordered_cells();
    }

    void HexMotorCycleBlock::rebuild_ordered_cells(
        ) {
        // LOG::TRACE(__FUNCTION__);
        assert(!cells_.empty());

        int min_x = std::numeric_limits<int>::max();
        int max_x = std::numeric_limits<int>::min();
        int min_y = std::numeric_limits<int>::max();
        int max_y = std::numeric_limits<int>::min();
        int min_z = std::numeric_limits<int>::max();
        int max_z = std::numeric_limits<int>::min();
        for (const auto& BC : cells_) {
            min_x = std::min(min_x, BC.coord.x);
            max_x = std::max(max_x, BC.coord.x);
            min_y = std::min(min_y, BC.coord.y);
            max_y = std::max(max_y, BC.coord.y);
            min_z = std::min(min_z, BC.coord.z);
            max_z = std::max(max_z, BC.coord.z);
        }
        len_x_ = max_x - min_x + 1;
        len_y_ = max_y - min_y + 1;
        len_z_ = max_z - min_z + 1;
        if (len_x_*len_y_*len_z_ != cells_.size()) {
            throw std::logic_error("This block is likely toroidal rather than cuboidal! But the split function has not yet been implemented.");
            // TODO: need to split this block
            return;
        }

        /* Fill orderly */
        std::vector<BlockCell> new_cells(len_x_*len_y_*len_z_);
        for (auto& BC : cells_) {
            BC.coord.x -= min_x;
            BC.coord.y -= min_y;
            BC.coord.z -= min_z;
            new_cells[BC.coord.x + BC.coord.y*len_x_ + BC.coord.z*len_x_*len_y_] = BC;
        }
        cells_.swap(new_cells);
        assert(std::ranges::all_of(cells_, [&](const auto& BC){ return BC.c != GEO::NO_INDEX; }));

        // DEBUG
        {
            // GEO::Mesh M_out;
            // M_out.copy(M_);
            // M_out.edges.clear();
            // M_out.facets.clear();
            // GEO::Attribute<int> M_out_c_coord_x(M_out.cells.attributes(), "coord_x");
            // GEO::Attribute<int> M_out_c_coord_y(M_out.cells.attributes(), "coord_y");
            // GEO::Attribute<int> M_out_c_coord_z(M_out.cells.attributes(), "coord_z");
            // GEO::Attribute<GEO::index_t> M_out_c_idx(M_out.cells.attributes(), "idx");
            // GEO::vector<GEO::index_t> cells_to_delete(M_out.cells.nb(), 1);
            // for (GEO::index_t i = 0; i < cells_.size(); ++i) {
            //     const auto& BC = cells_[i];
            //     M_out_c_coord_x[BC.c] = BC.coord.x;
            //     M_out_c_coord_y[BC.c] = BC.coord.y;
            //     M_out_c_coord_z[BC.c] = BC.coord.z;
            //     M_out_c_idx[BC.c] = i;
            //     cells_to_delete[BC.c] = 0;
            // }
            // M_out.cells.delete_elements(cells_to_delete);
            // GEO::mesh_save(M_out, "debug.geogram");
            // THROW_RUNTIME_ERROR("im here");
        }
    }

    GEO::index_t HexMotorCycleBlock::cell_corner_vertex(
        const GEO::index_t lv
        ) const {
        assert(!cells_.empty());
        assert(lv < 8);

        switch (lv) {
            case 0: {
                const auto& BC = cells_[0];
                assert(geolio::HEX_LF_LF_LF_COMMON_LV(BC.lfs[0], BC.lfs[2], BC.lfs[4]) != GEO::NO_INDEX);
                return mesh_.cells.vertex(
                    BC.c,
                    geolio::HEX_LF_LF_LF_COMMON_LV(BC.lfs[0], BC.lfs[2], BC.lfs[4]));
            }
            case 1: {
                const auto& BC = cells_[len_x_-1];
                assert(geolio::HEX_LF_LF_LF_COMMON_LV(BC.lfs[1], BC.lfs[2], BC.lfs[4]) != GEO::NO_INDEX);
                return mesh_.cells.vertex(
                    BC.c,
                    geolio::HEX_LF_LF_LF_COMMON_LV(BC.lfs[1], BC.lfs[2], BC.lfs[4]));
            }
            case 2: {
                const auto& BC = cells_[len_x_*(len_y_-1)];
                assert(geolio::HEX_LF_LF_LF_COMMON_LV(BC.lfs[0], BC.lfs[3], BC.lfs[4]) != GEO::NO_INDEX);
                return mesh_.cells.vertex(
                    BC.c,
                    geolio::HEX_LF_LF_LF_COMMON_LV(BC.lfs[0], BC.lfs[3], BC.lfs[4]));
            }
            case 3: {
                const auto& BC = cells_[len_x_*len_y_-1];
                assert(geolio::HEX_LF_LF_LF_COMMON_LV(BC.lfs[1], BC.lfs[3], BC.lfs[4]) != GEO::NO_INDEX);
                return mesh_.cells.vertex(
                    BC.c,
                    geolio::HEX_LF_LF_LF_COMMON_LV(BC.lfs[1], BC.lfs[3], BC.lfs[4]));
            }
            case 4: {
                const auto& BC = cells_[len_x_*len_y_*(len_z_-1)];
                assert(geolio::HEX_LF_LF_LF_COMMON_LV(BC.lfs[0], BC.lfs[2], BC.lfs[5]) != GEO::NO_INDEX);
                return mesh_.cells.vertex(
                    BC.c,
                    geolio::HEX_LF_LF_LF_COMMON_LV(BC.lfs[0], BC.lfs[2], BC.lfs[5]));
            }
            case 5: {
                const auto& BC = cells_[len_x_*len_y_*(len_z_-1)+len_x_-1];
                assert(geolio::HEX_LF_LF_LF_COMMON_LV(BC.lfs[1], BC.lfs[2], BC.lfs[5]) != GEO::NO_INDEX);
                return mesh_.cells.vertex(
                    BC.c,
                    geolio::HEX_LF_LF_LF_COMMON_LV(BC.lfs[1], BC.lfs[2], BC.lfs[5]));
            }
            case 6: {
                const auto& BC = cells_[len_x_*len_y_*(len_z_-1)+len_x_*(len_y_-1)];
                assert(geolio::HEX_LF_LF_LF_COMMON_LV(BC.lfs[0], BC.lfs[3], BC.lfs[5]) != GEO::NO_INDEX);
                return mesh_.cells.vertex(
                    BC.c,
                    geolio::HEX_LF_LF_LF_COMMON_LV(BC.lfs[0], BC.lfs[3], BC.lfs[5]));
            }
            case 7: {
                const auto& BC = cells_[len_x_*len_y_*(len_z_-1)+len_x_*len_y_-1];
                assert(geolio::HEX_LF_LF_LF_COMMON_LV(BC.lfs[1], BC.lfs[3], BC.lfs[5]) != GEO::NO_INDEX);
                return mesh_.cells.vertex(
                    BC.c,
                    geolio::HEX_LF_LF_LF_COMMON_LV(BC.lfs[1], BC.lfs[3], BC.lfs[5]));
            }
            default:
                assert(0);
                return GEO::NO_INDEX;
        }
    }
}