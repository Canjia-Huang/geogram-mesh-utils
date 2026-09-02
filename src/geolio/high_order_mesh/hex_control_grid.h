//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_HEX_CONTROL_GRID_H
#define GEOLIO_HEX_CONTROL_GRID_H
#include "volume_control_grid.h"
#include <Eigen/Dense>

namespace geolio
{
    class HexControlGrid : public VolumeControlGrid {
    public:
        /**
         * @brief Construct a hexahedral high-order control grid.
         * @param[in] mesh Input hexahedral mesh used as the reference topology/geometry.
         * @param[in] order Polynomial order of the tensor-product hexahedral mapping.
         */
        HexControlGrid(const GEO::Mesh& mesh, GEO::index_t order);

        /**
         * Evaluate the Jacobian matrix of the cell mapping at a parameter point.
         *
         * The Jacobian is a 3x3 matrix containing the partial derivatives of the physical
         * position with respect to the three parametric directions (u, v, w).
         *
         * @param[in] c Cell index, 0,1,...,hex_mesh.cells.nb()-1.
         * @param[in] uvw Parameter point in the cell parameter domain [0, 1]^3.
         * @param[out] J Output 3x3 Jacobian matrix at the given parameter point.
         *               Entry J(i,j) represents \f$\partial x_i / \partial u_j\f$ where:
         *               - columns 0, 1, 2 correspond to u, v, w derivatives
         *               - rows 0, 1, 2 correspond to x, y, z physical coordinates
         */
        void compute_cell_uvw_Jacobian(
                GEO::index_t c,
                const GEO::vec3& uvw,
                Eigen::Matrix3d& J) const;

        enum class MeasureType {
            JACOBIAN,             // Signed Jacobian determinant; non-positive values indicate inversion or degeneration.
            SCALED_JACOBIAN,      // Skew measure / normalized Jacobian; 1.0 is ideal, and non-positive values indicate collapse or inversion.
            INVERSE_MEAN_RATIO,   // Shape-quality metric combining angle and aspect-ratio distortion; 1.0 is best and 0 indicates degeneration.
            MIPS                  // Minimizes shear and anisotropic stretching; 1.0 is best and the value grows toward infinity near degeneration.
        };

        /**
         * Evaluate a geometric quality metric at a point in the cell parameter domain.
         *
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[in] uvw parameter point in the cell parameter domain [0, 1]^3
         * @param[in] quality_type quality metric to evaluate:
         *                       - `QualityType::JACOBIAN`: signed Jacobian determinant, [-inf, inf]
         *                       - `QualityType::SCALED_JACOBIAN`: normalized Jacobian / skew measure, [-1, 1], best: 1
         *                       - `QualityType::INVERSE_MEAN_RATIO`: shape quality measure based on Jacobian and metric lengths, [0, 1], best: 1
         *                       - `QualityType::MIPS`: penalizes shear and anisotropic stretching, [1, inf], best: 1
         * @return the requested quality value at the given parameter point
         */
        [[nodiscard]] double compute_cell_uvw_measure(
                GEO::index_t c,
                const GEO::vec3& uvw,
                MeasureType quality_type) const;

        /**
         * Evaluate the gradient of the Jacobian determinant at a cell parameter point.
         *
         * The output stores \f$\partial\det(J)/\partial x\f$ with respect to all local control-point
         * coordinates of cell \p c, flattened in xyz order.
         *
         * @param[in] c Cell index, 0,1,...,hex_mesh.cells.nb()-1.
         * @param[in] uvw Parameter point in the cell parameter domain [0, 1]^3.
         * @param[out] gradient Output buffer of size `3 * control_points_nb_per_cell()`.
         *                      For local control point `N`, entries are:
         *                      - `gradient[3*N+0] = d(detJ)/dP_N.x`
         *                      - `gradient[3*N+1] = d(detJ)/dP_N.y`
         *                      - `gradient[3*N+2] = d(detJ)/dP_N.z`
         */
        void compute_cell_uvw_detJ_gradient(
            GEO::index_t c,
            const GEO::vec3& uvw,
            std::vector<double>& gradient) const;

        /**
         * @brief Compute the reference volume of every cell in the mesh.
         *
         * @param[out] volumes Output array that receives one reference volume per
         *            cell, in the same order as `hex_mesh_.cells`.
         * @note The caller is responsible for providing storage for all cells.
         */
        void compute_cells_volume(std::vector<double>& volumes) const;

        /**
         * Assemble physical control-point coordinates of one cell into a dense matrix.
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[out] P matrix of control-point positions for cell \\p c
         */
        void compute_cell_vertices_position_matrix(
            GEO::index_t c,
            Eigen::MatrixXd& P);

        /**
         * Assemble basis gradients at a parameter point for all local control points.
         * @param[in] uvw parameter point in [0,1]^3
         * @param[out] Bg gradient matrix of tensor-product basis values
         * @pre Bg.size == CONTROL_POINTS_NB_PER_CELL * 3
         */
        void compute_basis_gradient_matrix(
            const GEO::vec3& uvw,
            Eigen::MatrixXd& Bg) const;

    protected:
        /**
         * @brief Initialize local indexing/layout rules for hexahedral control nodes.
         */
        void initialize_nodes_arrangement() override;

        /**
         * @brief Build global control-node coordinates and cell-to-control-node connectivity.
         */
        void initialize_control_nodes() override;
    };
}

#endif //GEOLIO_HEX_CONTROL_GRID_H
