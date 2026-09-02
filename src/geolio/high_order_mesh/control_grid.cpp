//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "control_grid.h"
#include "node_positions.h"
#include "basis_functions.h"
#include <cassert>

namespace geolio
{
    ControlGrid::ControlGrid(
        const GEO::Mesh& mesh,
        const GEO::index_t order
        ) : mesh_(mesh), ORDER_(order)
    {
        assert(ORDER_ > 0);

        initialize_node_positions_1D();
    }

    void ControlGrid::set_nodes_type(
        const NodesType nodes_type
        ) {
        if (nodes_type != nodes_type_)
            initialize_node_positions_1D();
    }

    void ControlGrid::initialize_node_positions_1D(
        ) {
        switch (nodes_type_) {
            case NodesType::EQUALLY_SPACED_NODES:
                compute_equally_spaced_nodes(ORDER_, node_positions_1D_);
                break;
            case NodesType::CHEBYSHEV_GAUSS:
                compute_Chebyshev_Gauss_nodes(ORDER_, node_positions_1D_);
                break;
            case NodesType::CHEBYSHEV_GAUSS_LOBATTO:
                compute_Chebyshev_Gauss_Lobatto_nodes(ORDER_, node_positions_1D_);
                break;
            case NodesType::LEGENDRE_GAUSS_LOBATTO:
                compute_Legendre_Gauss_Lobatto_nodes(ORDER_, node_positions_1D_);
                break;
            default:
                assert(0);
        }
    }

    GEO::vec3 VolumeControlGrid::compute_cell_uvw_position(
        const GEO::index_t c,
        const GEO::vec3& uvw
        ) const {
        assert(c < mesh_.cells.nb());
        assert(uvw.x >= 0 && uvw.x <= 1);
        assert(uvw.y >= 0 && uvw.y <= 1);
        assert(uvw.z >= 0 && uvw.z <= 1);

        GEO::vec3 p(0, 0, 0);

        std::vector<double> Bu(ORDER_+1);
        std::vector<double> Bv(ORDER_+1);
        std::vector<double> Bw(ORDER_+1);
        Lagrange_basis_1D(uvw.x, node_positions_1D_, Bu);
        Lagrange_basis_1D(uvw.y, node_positions_1D_, Bv);
        Lagrange_basis_1D(uvw.z, node_positions_1D_, Bw);

        for (GEO::index_t i = 0; i <= ORDER_; ++i) {
            for (GEO::index_t j = 0; j <= ORDER_; ++j) {
                const double basis_uv = Bu[i] * Bv[j];
                for (GEO::index_t k = 0; k <= ORDER_; ++k) {
                    const double lag_basis = basis_uv * Bw[k];
                    p += lag_basis * get_cv_position(cell_cv(c, i, j, k));
                }
            }
        }

        return p;
    }

    GEO::vec3 VolumeControlGrid::compute_cell_facet_uv_normal(
        const GEO::index_t c,
        const GEO::index_t lf,
        const GEO::vec2& uv
        ) const {
        assert(c < mesh_.cells.nb());
        assert(lf < mesh_.cells.nb_facets(c));
        assert(uv.x >= 0 && uv.x <= 1);
        assert(uv.y >= 0 && uv.y <= 1);

        std::vector<double> Bu(ORDER_+1);
        std::vector<double> Bv(ORDER_+1);
        std::vector<double> dBu(ORDER_+1);
        std::vector<double> dBv(ORDER_+1);
        Lagrange_basis_1D(uv.x, node_positions_1D_, Bu);
        Lagrange_basis_1D(uv.y, node_positions_1D_, Bv);
        Lagrange_basis_deriv_1D(uv.x, node_positions_1D_, dBu);
        Lagrange_basis_deriv_1D(uv.y, node_positions_1D_, dBv);

        GEO::vec3 Tu(0, 0, 0), Tv(0, 0, 0);
        for (GEO::index_t i = 0; i < CONTROL_POINTS_NB_PER_EDGE; ++i) {
            for (GEO::index_t j = 0; j < CONTROL_POINTS_NB_PER_EDGE; ++j) {
                const auto& p = get_cv_position(cell_facet_cv(c, lf, i, j));
                Tu += p * dBu[i] * Bv[j];
                Tv += p * Bu[i] * dBv[j];
            }
        }
        return -GEO::cross(Tu, Tv); /* The orientation of the vertices of the cell facet is towards the interior of the
            cell, so the normal direction needs to be reversed. */
    }

    void VolumeControlGrid::compute_cell_uvw_dudvdw(
        const GEO::index_t c,
        const GEO::vec3& uvw,
        GEO::vec3& du,
        GEO::vec3& dv,
        GEO::vec3& dw,
        std::vector<double>& Bu,
        std::vector<double>& Bv,
        std::vector<double>& Bw,
        std::vector<double>& dBu,
        std::vector<double>& dBv,
        std::vector<double>& dBw
        ) const {
        assert(c < mesh_.cells.nb());
        assert(uvw.x >= 0 && uvw.x <= 1);
        assert(uvw.y >= 0 && uvw.y <= 1);
        assert(uvw.z >= 0 && uvw.z <= 1);

        du.x = 0; du.y = 0; du.z = 0;
        dv.x = 0; dv.y = 0; dv.z = 0;
        dw.x = 0; dw.y = 0; dw.z = 0;

        Bu.resize(ORDER_+1);
        Bv.resize(ORDER_+1);
        Bw.resize(ORDER_+1);
        dBu.resize(ORDER_+1);
        dBv.resize(ORDER_+1);
        dBw.resize(ORDER_+1);
        Lagrange_basis_1D(uvw.x, node_positions_1D_, Bu);
        Lagrange_basis_1D(uvw.y, node_positions_1D_, Bv);
        Lagrange_basis_1D(uvw.z, node_positions_1D_, Bw);
        Lagrange_basis_deriv_1D(uvw.x, node_positions_1D_, dBu);
        Lagrange_basis_deriv_1D(uvw.y, node_positions_1D_, dBv);
        Lagrange_basis_deriv_1D(uvw.z, node_positions_1D_, dBw);

        for (GEO::index_t k = 0; k < CONTROL_POINTS_NB_PER_EDGE; ++k) {
            for (GEO::index_t j = 0; j < CONTROL_POINTS_NB_PER_EDGE; ++j) {
                const double basis_vw = Bv[j] * Bw[k];
                const double basis_dvw= dBv[j] * Bw[k];
                const double basis_vdw= Bv[j] * dBw[k];
                for (GEO::index_t i = 0; i < CONTROL_POINTS_NB_PER_EDGE; ++i) {
                    const double lag_basis_duvw = dBu[i] * basis_vw;
                    const double lag_basis_udvw = Bu[i] * basis_dvw;
                    const double lag_basis_uvdw = Bu[i] * basis_vdw;
                    du += lag_basis_duvw * get_cv_position(cell_cv(c, i, j, k));
                    dv += lag_basis_udvw * get_cv_position(cell_cv(c, i, j, k));
                    dw += lag_basis_uvdw * get_cv_position(cell_cv(c, i, j, k));
                }
            }
        }
    }
}