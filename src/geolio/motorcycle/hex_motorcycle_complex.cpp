//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "hex_motorcycle_complex.h"
#include <cassert>
#include <geogram/mesh/mesh_io.h>
#include <geolio/common/utils.h>
#include <geolio/mesh/hex_operations.h>
#include "geolio/mesh/mesh_operations.h"
#include <geolio/common/pair_hash.h>

namespace geolio
{
    HexMotorCycleComplex::HexMotorCycleComplex(
        const GEO::Mesh& mesh
        ) : attribute_id_(generate_random_string(22)),
            mesh_(mesh)
    {
        assert(std::all_of(
            mesh_.cells.cell_type_ptr(0),
            mesh_.cells.cell_type_ptr(0)+mesh_.cells.nb(),
            [&](const auto cell_type) { return cell_type == GEO::MESH_HEX; })); // check all-hex mesh

        mesh_cf_tagged_.bind(mesh_.cell_facets.attributes(), attribute_id_+":tagged");
        mesh_cf_tagged_.fill(GEO::NO_INDEX);

        find_all_singular_and_border_edges();
    }

    HexMotorCycleComplex::~HexMotorCycleComplex(
        ) {
        if (mesh_cf_tagged_.is_bound())
            mesh_cf_tagged_.destroy();
    }

    GEO::index_t HexMotorCycleComplex::compute(
        const HexMotorCycleComplexType complex_type
        ) {
        std::priority_queue<Fire> queue;

        /* Ignite */
        ignite(queue);

        /* Burning */
        while (!queue.empty()) {
            const auto& fire = queue.top();
            const auto F_d = fire.d;
            const auto F_c = fire.c;
            const auto F_le = fire.le;
            const auto F_lf = fire.lf;
            queue.pop();

            /* Alive */
            bool alive = false;
            if (complex_type == BASE_COMPLEX)
                alive = true;
            else {
                assert(complex_type == MOTORCYCLE_COMPLEX);
                if (mesh_ce_singular_[12*F_c+F_le])
                    alive = true;
                else {
                    /* Find all incident facets */
                    std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> ordered_c_le_lf;
                    if (get_edge_incident_cells(mesh_, F_c, F_le, ordered_c_le_lf)) { // append the preceding border facet
                        const auto& pre_lf = get<2>(ordered_c_le_lf[0]);
                        const auto& lf0 = HEX_LE_INCIDENT_LF[F_le][0];
                        const auto& lf1 = HEX_LE_INCIDENT_LF[F_le][1];
                        if (pre_lf == lf0) // append lf1
                            ordered_c_le_lf.emplace_back(F_c, F_le, lf1);
                        else { // append lf0
                            assert(pre_lf == lf1);
                            ordered_c_le_lf.emplace_back(F_c, F_le, lf0);
                        }
                    }

                    GEO::index_t tagged_facets_nb = 0;
                    for (const auto& [c, _, lf] : ordered_c_le_lf) {
                        if (mesh_cf_tagged_[8*c+lf] != GEO::NO_INDEX)
                            ++tagged_facets_nb;
                    }

                    if (tagged_facets_nb < 3)
                        alive = true;
                }
            }
            if (!alive)
                continue;

            /* Mark facet as burnt */
            mesh_cf_tagged_[8*F_c+F_lf] = F_d;
            if (const auto& nc = mesh_.cells.adjacent(F_c, F_lf);
                nc != GEO::NO_CELL) {
                const auto& nlf = find_hex_facet(
                    mesh_,
                    nc,
                    mesh_.cells.facet_vertex(F_c, F_lf, 2),
                    mesh_.cells.facet_vertex(F_c, F_lf, 1),
                    mesh_.cells.facet_vertex(F_c, F_lf, 0));
                assert(nlf != GEO::NO_INDEX);
                mesh_cf_tagged_[8*nc+nlf] = F_d;
            }

            /* Burning */
            for (const auto& F_le1 : HEX_LF_INCIDENT_LE[F_lf]) {
                if (F_le1 == F_le || mesh_ce_singular_[12*F_c+F_le1] || mesh_ce_border_[12*F_c+F_le1]) // need to be regular and interior
                    continue;

                /* Find all incident facets */
                std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> ordered_c_le_lf;
                const auto on_border = get_edge_incident_cells(mesh_, F_c, F_le1, ordered_c_le_lf);
                assert(!on_border);
                assert(get<0>(ordered_c_le_lf[0]) == F_c);
                assert(ordered_c_le_lf.size() == 4); // regular

                const GEO::index_t opp_i = mesh_.cells.edge_adjacent_facet(F_c, F_le1, 0) == F_lf ? 2 : 1; // let ordered_c_le_lf[opp_i] the next to burn

                if (const auto& [opp_c, opp_le, opp_lf] = ordered_c_le_lf[opp_i];
                    mesh_cf_tagged_[8*opp_c+opp_lf] == GEO::NO_INDEX
                    ) {
                    assert((mesh_.cells.edge_vertex(F_c, F_le1, 0) == mesh_.cells.edge_vertex(opp_c, opp_le, 0) &&
                            mesh_.cells.edge_vertex(F_c, F_le1, 1) == mesh_.cells.edge_vertex(opp_c, opp_le, 1)) ||
                            mesh_.cells.edge_vertex(F_c, F_le1, 0) == mesh_.cells.edge_vertex(opp_c, opp_le, 1) &&
                            mesh_.cells.edge_vertex(F_c, F_le1, 1) == mesh_.cells.edge_vertex(opp_c, opp_le, 0));
                    assert(mesh_.cells.adjacent(F_c, F_lf) != opp_c);

                    Fire new_F{};
                    new_F.d = F_d+1;
                    new_F.c = opp_c;
                    new_F.le = opp_le;
                    new_F.lf = opp_lf;

                    queue.push(new_F);
                }
            }
        }

        /* Label border facets */
        for (const auto& c : mesh_.cells) {
            for (GEO::index_t lf = 0; lf < 6; ++lf) {
                if (mesh_.cells.adjacent(c, lf) == GEO::NO_CELL)
                    mesh_cf_tagged_[8*c+lf] = 0;
            }
        }

        /* Decompose */
        const GEO::index_t blocks_nb = decompose_into_blocks();

        return blocks_nb;
    }

    void HexMotorCycleComplex::label_blocks(
        GEO::Attribute<GEO::index_t>& mesh_c_block
        ) const {
        assert(mesh_c_block.is_bound());
        assert(mesh_c_block.size() == mesh_.cells.nb());
        if (blocks_.empty())
            throw std::logic_error("Need to call compute() first!");

        for (GEO::index_t i = 0, i_end = blocks_.size(); i < i_end; ++i) {
            const auto& block = blocks_[i];
            for (const auto& bcs = block.block_cells();
                const auto& bc : bcs)
                mesh_c_block[bc.c] = i;
        }
    }

    void HexMotorCycleComplex::create_coarse_mesh(
        GEO::Mesh& mesh_out,
        std::vector<GEO::index_t>* old_cf_to_new_cf
        ) const {
        if (blocks_.empty())
            throw std::logic_error("Need to call compute() first!");

        mesh_out.clear(false);
        mesh_out.copy(mesh_, false, GEO::MESH_VERTICES);

        GEO::index_t new_c = mesh_out.cells.create_hexes(blocks_.size());
        for (const auto& block : blocks_) {
            for (GEO::index_t lv = 0; lv < 8; ++lv)
                mesh_out.cells.set_vertex(new_c, lv, block.cell_corner_vertex(lv));
            ++new_c;
        }
        mesh_out.cells.connect();

        if (old_cf_to_new_cf != nullptr) {
            old_cf_to_new_cf->assign(8*mesh_.cells.nb(), GEO::NO_INDEX);
            assert(mesh_out.cells.nb() == blocks_.size());
            for (const auto& c : mesh_out.cells) {
                for (const auto& BC : blocks_[c].block_cells()) {
                    for (GEO::index_t lf = 0; lf < 6; ++lf) {
                        if (const auto old_cf = 8*BC.c + BC.lfs[lf];
                            mesh_cf_tagged_[old_cf] != GEO::NO_INDEX
                            )
                            (*old_cf_to_new_cf)[old_cf] = 8*c+lf;
                    }
                }
            }
        }
    }

    void HexMotorCycleComplex::find_all_singular_and_border_edges(
        ) {
        mesh_ce_singular_.assign(12*mesh_.cells.nb(), false);
        mesh_ce_border_.assign(12*mesh_.cells.nb(), false);

        std::unordered_map<std::pair<GEO::index_t, GEO::index_t>, std::pair<GEO::index_t, bool>, PairHash> cell_edges; /*
            edge (ev0, ev1), ev0 < ev1 -> (incident cells nb, is on boundary?) */
        for (const auto& c : mesh_.cells) {
            for (GEO::index_t le = 0; le < 12; ++le) {
                bool is_on_boundary = (mesh_.cells.adjacent(c, mesh_.cells.edge_adjacent_facet(c, le, 0)) == GEO::NO_FACET) ||
                                      (mesh_.cells.adjacent(c, mesh_.cells.edge_adjacent_facet(c, le, 1)) == GEO::NO_FACET);
                const auto& ev0 = mesh_.cells.edge_vertex(c, le, 0);
                const auto& ev1 = mesh_.cells.edge_vertex(c, le, 1);
                const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(ev0 ,ev1);
                if (auto it = cell_edges.find(edge);
                    it == cell_edges.end())
                    cell_edges.emplace(edge, std::pair(1, is_on_boundary));
                else {
                    ++it->second.first;
                    if (is_on_boundary)
                        it->second.second = true;
                }
            }
        }

        /* Label le */
        for (const auto& c : mesh_.cells) {
            for (GEO::index_t le = 0; le < 12; ++le) {
                const auto& ev0 = mesh_.cells.edge_vertex(c, le, 0);
                const auto& ev1 = mesh_.cells.edge_vertex(c, le, 1);
                const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(ev0 ,ev1);
                assert(cell_edges.contains(edge));

                if (const auto& [nb, is_on_boundary] = cell_edges.at(edge);
                    is_on_boundary
                    ) {
                    mesh_ce_border_[12*c+le] = true;
                    if (nb != 2)
                        mesh_ce_singular_[12*c+le] = true;
                    }
                else {
                    if (nb != 4)
                        mesh_ce_singular_[12*c+le] = true;
                }
            }
        }
    }

    void HexMotorCycleComplex::ignite(
        std::priority_queue<Fire>& queue
        ) const {
        assert(mesh_ce_singular_.size() == 12*mesh_.cells.nb());

        while (!queue.empty())
            queue.pop();

        std::vector<bool> processed_edges(12*mesh_.cells.nb(), false);
        for (const auto& c : mesh_.cells) {
            for (GEO::index_t le = 0; le < 12; ++le) {
                if (processed_edges[12*c+le] || !mesh_ce_singular_[12*c+le]) // for all singular edge
                    continue;

                /* Find all incident interior facets */
                std::vector<std::tuple<GEO::index_t, GEO::index_t, GEO::index_t>> ordered_c_le_lf;
                get_edge_incident_cells(mesh_, c, le, ordered_c_le_lf);
                for (const auto& [adj_c, adj_le, adj_lf] : ordered_c_le_lf) {
                    processed_edges[12*adj_c+adj_le] = true;

                    if (mesh_.cells.adjacent(adj_c, adj_lf) == GEO::NO_CELL)
                        continue;

                    Fire F{};
                    F.d = 0;
                    F.c = adj_c;
                    F.le = adj_le;
                    F.lf = adj_lf;

                    queue.push(F);
                }
            }
        }
    }

    GEO::index_t HexMotorCycleComplex::decompose_into_blocks(
        ) {
        blocks_.clear();

        std::vector<bool> prcessed_cells(mesh_.cells.nb(), false);
        for (const auto& start_c : mesh_.cells) {
            if (prcessed_cells[start_c]) // labelled
                continue;

            /* Build motorcycle block */
            HexMotorCycleBlock MC_block(mesh_, mesh_cf_tagged_);
            MC_block.flood_fill_cells(start_c);

            for (const auto& b_cell : MC_block.block_cells())
                prcessed_cells[b_cell.c] = true;

            blocks_.push_back(MC_block);
        }

        return blocks_.size();
    }
}