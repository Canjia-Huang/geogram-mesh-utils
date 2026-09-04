//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/2.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_CONTROL_GRID_H
#define GEOLIO_CONTROL_GRID_H
#include <cassert>
#include <geogram/mesh/mesh.h>

namespace geolio
{
    class ControlGrid {
    public:
        /**
         * @brief Construct a control grid from a reference mesh and polynomial order.
         * @param[in] mesh Input mesh used as topology/geometry reference.
         * @param[in] order Polynomial order of the high-order representation.
         */
        ControlGrid(const GEO::Mesh& mesh, GEO::index_t order);

        /**
         * @brief Virtual destructor.
         */
        virtual ~ControlGrid() = default;

        /**
         * @brief Access the reference mesh.
         * @return Const reference to the underlying mesh.
         */
        const auto& mesh() const { return mesh_; }

        enum class BasisFunctionType {
            BEZIER, // not support yet
            LAGRANGE
        };

        /**
         * @brief Get the basis-function family used by this control grid.
         * @return Current basis-function type.
         */
        [[nodiscard]] const auto& basis_function_type() const { return basis_function_type_; }

        enum class NodesType {
            EQUALLY_SPACED_NODES,
            CHEBYSHEV_GAUSS,
            CHEBYSHEV_GAUSS_LOBATTO,
            LEGENDRE_GAUSS_LOBATTO
        };

        /**
         * @brief Get the 1D node distribution type.
         * @return Current control-node distribution policy.
         */
        [[nodiscard]] const auto& nodes_type() const { return nodes_type_; }

        /**
         * @brief Set the 1D node distribution type and refresh dependent data.
         * @param[in] nodes_type New control-node distribution policy.
         */
        void set_nodes_type(NodesType nodes_type);

        /**
         * Get 1D parametric coordinates of control nodes.
         * @return const reference to node positions in [0,1]
         */
        [[nodiscard]] const auto& node_positions_1D() const { return node_positions_1D_; }

        /**
         * @brief Get the polynomial order.
         * @return Polynomial order used by this control grid.
         */
        [[nodiscard]] auto order() const { return ORDER_; }

        /**
         * @brief Get the total number of control nodes.
         * @return Number of vertices stored in the control-node mesh.
         */
        [[nodiscard]] auto control_nodes_nb() const { return control_nodes_.vertices.nb(); }

        /**
         * @brief Access a mutable control node by global index.
         * @param[in] v Control-node index.
         * @return Mutable reference to the control-node position.
         */
        auto& control_node(const GEO::index_t v) {
            assert(v < control_nodes_nb());
            return control_nodes_.vertices.point(v);
        }

        /**
         * @brief Access a const control node by global index.
         * @param[in] v Control-node index.
         * @return Const reference to the control-node position.
         */
        [[nodiscard]] const auto& control_node(const GEO::index_t v) const {
            assert(v < control_nodes_nb());
            return control_nodes_.vertices.point(v);
        }

        /**
         * @brief Access the mutable contiguous control-node coordinate array.
         * @return Mutable view/proxy over all control-node coordinates.
         */
        auto control_nodes() {
            return control_nodes_.vertices.points();
        }

        /**
         * @brief Access the const contiguous control-node coordinate array.
         * @return Const view/proxy over all control-node coordinates.
         */
        [[nodiscard]] auto control_nodes() const {
            return control_nodes_.vertices.points();
        }

        /**
         * @brief Get a raw mutable pointer to one control-node coordinate tuple.
         * @param[in] v Control-node index.
         * @return Pointer to the first component of the indexed node position.
         */
        auto* control_node_ptr(const GEO::index_t v) {
            return control_nodes_.vertices.point_ptr(v);
        }

    protected:
        const GEO::Mesh& mesh_;

        BasisFunctionType basis_function_type_ = BasisFunctionType::LAGRANGE;

        /**
         * @brief Initialize the 1D parametric node positions according to `nodes_type_`.
         */
        void initialize_node_positions_1D();
        NodesType nodes_type_ = NodesType::EQUALLY_SPACED_NODES;
        std::vector<double> node_positions_1D_; // 1D distribution of control nodes

        const GEO::index_t ORDER_;

        /**
         * @brief Build control-node connectivity/geometry for the derived grid type.
         */
        virtual void initialize_control_nodes() = 0;
        GEO::Mesh control_nodes_;
        std::vector<GEO::index_t> element_control_nodes_;
    };
}

#endif //GEOLIO_CONTROL_GRID_H
