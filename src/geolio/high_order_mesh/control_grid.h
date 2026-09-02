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
        ControlGrid(const GEO::Mesh& mesh, GEO::index_t order);

        virtual ~ControlGrid() = default;

        enum class NodesType {
            EQUALLY_SPACED_NODES,
            CHEBYSHEV_GAUSS,
            CHEBYSHEV_GAUSS_LOBATTO,
            LEGENDRE_GAUSS_LOBATTO
        };

        [[nodiscard]] const auto& nodes_type() const { return nodes_type_; }

        void set_nodes_type(NodesType nodes_type);

        // [[nodiscard]] const auto& control_nodes() const { return control_nodes_; }

        [[nodiscard]] const auto& control_nodes_nb() const { return control_nodes_nb_; }

        auto& control_node(const GEO::index_t v) {
            assert(v < control_nodes_nb_);
            return GEO::Memory::pointer_as_reference<GEO::vec3>(&control_nodes_[CONTROL_NODES_DIM_*v]);
        }

        [[nodiscard]] const auto& control_node(const GEO::index_t v) const {
            assert(v < control_nodes_nb_);
            return GEO::Memory::pointer_as_reference<GEO::vec3>(&control_nodes_[CONTROL_NODES_DIM_*v]);
        }

        auto control_nodes() {
            return GEO::transform_range_ref(
                GEO::index_range(0, control_nodes_nb_),
                [this](GEO::index_t v)->const GEO::vec3& {
                    return GEO::Memory::pointer_as_reference<GEO::vec3>(&control_nodes_[CONTROL_NODES_DIM_*v]);
                }
            );
        }

        [[nodiscard]] auto control_nodes() const {
            return GEO::transform_range_ref(
                GEO::index_range(0, control_nodes_nb_),
                [this](GEO::index_t v)->const GEO::vec3& {
                    return GEO::Memory::pointer_as_reference<GEO::vec3>(&control_nodes_[CONTROL_NODES_DIM_*v]);
                }
            );
        }

        auto* control_node_ptr(const GEO::index_t v) {
            return &control_nodes_[CONTROL_NODES_DIM_*v];
        }

    protected:
        const GEO::Mesh& mesh_;

        void initialize_node_positions_1D();
        NodesType nodes_type_ = NodesType::EQUALLY_SPACED_NODES;
        std::vector<double> node_positions_1D_; // 1D distribution of control nodes

        virtual void initialize_nodes_arrangement() = 0;
        const GEO::index_t ORDER_;

        virtual void initialize_control_nodes() = 0;
        const GEO::index_t CONTROL_NODES_DIM_ = 3;
        GEO::index_t control_nodes_nb_ = 0;
        std::vector<double> control_nodes_;
        std::vector<GEO::index_t> element_control_nodes_;
    };

    class SurfaceControlGrid : public ControlGrid {
    public:
        SurfaceControlGrid(const GEO::Mesh& mesh, const GEO::index_t order) : ControlGrid(mesh, order) {}

    protected:
        GEO::index_t CONTROL_POINTS_NB_PER_EDGE = GEO::NO_INDEX;
        GEO::index_t CONTROL_POINTS_NB_PER_FACET = GEO::NO_INDEX;
        GEO::index_t INTERNAL_CONTROL_POINTS_NB_PER_EDGE = GEO::NO_INDEX;
        GEO::index_t INTERNAL_CONTROL_POINTS_NB_PER_FACET = GEO::NO_INDEX;
        std::vector<GEO::index_t> VERTEX_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<GEO::index_t> EDGE_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<int>          EDGE_CONTROL_POINTS_NEXT_IDX_STEP_{};
        std::vector<GEO::index_t> EDGE_INTERNAL_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<int>          EDGE_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP_{};
        std::vector<GEO::index_t> FACET_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<int>          FACET_CONTROL_POINTS_NEXT_IDX_STEP0_{};
        std::vector<int>          FACET_CONTROL_POINTS_NEXT_IDX_STEP1_{};
        std::vector<GEO::index_t> FACET_INTERNAL_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<int>          FACET_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP0_{};
        std::vector<int>          FACET_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP1_{};
    };

    class VolumeControlGrid : public ControlGrid {
    public:
        VolumeControlGrid(const GEO::Mesh& mesh, const GEO::index_t order) : ControlGrid(mesh, order) {}

        /**
         * Get a local control point index by local vertex index.
         * @param[in] lv local vertex index in the cell, 0,1,...,mesh_.cells.nb_vertices
         * @return local control point index, 0,1,...,CONTROL_POINTS_NB_PER_CELL
         */
        [[nodiscard]] GEO::index_t vertex_lcv(const GEO::index_t lv) const {
            assert(lv < mesh_.cells.nb_vertices(0));
            return VERTEX_CONTROL_POINTS_BEGIN_IDX_[lv];
        }

        /**
         * Get a local control point index by local edge index and local vertex index in the edge.
         * @param[in] le local edge index, 0,1,...,mesh_.cells.nb_edges
         * @param[in] lv local vertex index in the edge (ev0 -> ev1), 0,1,...,order
         * @return local control point index, 0,1,...,CONTROL_POINTS_NB_PER_CELL
         */
        [[nodiscard]] GEO::index_t edge_lcv(const GEO::index_t le, const GEO::index_t lv) const {
            assert(le < mesh_.cells.nb_edges(0));
            assert(lv < CONTROL_POINTS_NB_PER_EDGE);
            return EDGE_CONTROL_POINTS_BEGIN_IDX_[le] + lv*EDGE_CONTROL_POINTS_NEXT_IDX_STEP_[le];
        }

        /**
         * Get a local internal control point index by local edge index and local vertex index in the edge.
         * @param[in] le local edge index, 0,1,...,mesh_.cells.nb_edges
         * @param[in] lv local vertex index in the edge (ev0 -> ev1), 0,1,...,order-2
         * @return local control point index, 0,1,...,CONTROL_POINTS_NB_PER_CELL
         */
        [[nodiscard]] GEO::index_t edge_inner_lcv(const GEO::index_t le, const GEO::index_t lv) const {
            assert(le < mesh_.cells.nb_edges(0));
            assert(lv < INTERNAL_CONTROL_POINTS_NB_PER_EDGE);
            return EDGE_INTERNAL_CONTROL_POINTS_BEGIN_IDX_[le] + lv*EDGE_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP_[le];
        }

        /**
         * Get a local control point index by local facet index and two local vertex index in the facet.
         * @param[in] lf local facet index, 0,1,...,mesh_.cells.nb_facets
         * @param[in] lv0 local vertex index in the facet (fv0 -> fv1), 0,1,...,order
         * @param[in] lv1 local vertex index in the facet (fv0 -> fv3), 0,1,...,order
         * @return local control point index, 0,1,...,CONTROL_POINTS_NB_PER_CELL
         */
        [[nodiscard]] GEO::index_t facet_lcv(const GEO::index_t lf, const GEO::index_t lv0, const GEO::index_t lv1) const {
            assert(lf < mesh_.cells.nb_facets(0));
            assert(lv0 < CONTROL_POINTS_NB_PER_EDGE);
            assert(lv1 < CONTROL_POINTS_NB_PER_EDGE);
            return FACET_CONTROL_POINTS_BEGIN_IDX_[lf] + lv0*FACET_CONTROL_POINTS_NEXT_IDX_STEP0_[lf] + lv1*FACET_CONTROL_POINTS_NEXT_IDX_STEP1_[lf];
        }

        /**
         * Get a local internal control point index by local facet index and two local vertex index in the facet.
         * @param[in] lf local facet index, 0,1,...,mesh_.cells.nb_facets
         * @param[in] lv0 local vertex index in the facet (fv0 -> fv1), 0,1,...,order-2
         * @param[in] lv1 local vertex index in the facet (fv0 -> fv3), 0,1,...,order-2
         * @return local control point index, 0,1,...,CONTROL_POINTS_NB_PER_CELL
         */
        [[nodiscard]] GEO::index_t facet_inner_lcv(const GEO::index_t lf, const GEO::index_t lv0, const GEO::index_t lv1) const {
            assert(lf < mesh_.cells.nb_facets(0));
            assert(lv0 < INTERNAL_CONTROL_POINTS_NB_PER_EDGE);
            assert(lv1 < INTERNAL_CONTROL_POINTS_NB_PER_EDGE);
            return FACET_INTERNAL_CONTROL_POINTS_BEGIN_IDX_[lf] + lv0*FACET_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP0_[lf] + lv1*FACET_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP1_[lf];
        }

        /**
         * Get a local control point index by three local vertex indexes in the cell
         * @param[in] lv0 local vertex index in the facet (cv0 -> cv1), 0,1,...,order
         * @param[in] lv1 local vertex index in the facet (cv0 -> cv2), 0,1,...,order
         * @param[in] lv2 local vertex index in the facet (cv0 -> cv4), 0,1,...,order
         * @return local control point index, 0,1,...,CONTROL_POINTS_NB_PER_CELL
         */
        [[nodiscard]] GEO::index_t cell_lcv(const GEO::index_t lv0, const GEO::index_t lv1, const GEO::index_t lv2) const {
            assert(lv0 < CONTROL_POINTS_NB_PER_EDGE);
            assert(lv1 < CONTROL_POINTS_NB_PER_EDGE);
            assert(lv2 < CONTROL_POINTS_NB_PER_EDGE);
            return CELL_CONTROL_POINTS_BEGIN_IDX_ + lv0*CELL_CONTROL_POINTS_NEXT_IDX_STEP0_ + lv1*CELL_CONTROL_POINTS_NEXT_IDX_STEP1_ + lv2*CELL_CONTROL_POINTS_NEXT_IDX_STEP2_;
        }

        /**
         * Get a local internal control point index by three local vertex indexes in the cell
         * @param[in] lv0 local vertex index in the facet (cv0 -> cv1), 0,1,...,order-2
         * @param[in] lv1 local vertex index in the facet (cv0 -> cv2), 0,1,...,order-2
         * @param[in] lv2 local vertex index in the facet (cv0 -> cv4), 0,1,...,order-2
         * @return local control point index, 0,1,...,CONTROL_POINTS_NB_PER_CELL
         */
        [[nodiscard]] GEO::index_t cell_inner_lcv(const GEO::index_t lv0, const GEO::index_t lv1, const GEO::index_t lv2) const {
            assert(lv0 < INTERNAL_CONTROL_POINTS_NB_PER_EDGE);
            assert(lv1 < INTERNAL_CONTROL_POINTS_NB_PER_EDGE);
            assert(lv2 < INTERNAL_CONTROL_POINTS_NB_PER_EDGE);
            return CELL_INTERNAL_CONTROL_POINTS_BEGIN_IDX_ + lv0*CELL_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP0_ + lv1*CELL_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP1_ + lv2*CELL_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP2_;
        }

        /**
         * Get a control point index by local vertex index in the cell.
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[in] lv local vertex index in the cell, 0,1,...,7
         * @return control point index
         */
        [[nodiscard]] GEO::index_t cell_vertex_cv(const GEO::index_t c, const GEO::index_t lv) const {
            assert(c < mesh_.cells.nb());
            return element_control_nodes_[c*CONTROL_POINTS_NB_PER_CELL + vertex_lcv(lv)];
        }

        /**
         * Get a control point index by local edge index and local vertex index in the edge.
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[in] le local edge index, 0,1,...,11
         * @param[in] lv local vertex index in the edge (ev0 -> ev1), 0,1,...,order
         * @return control point index
         */
        [[nodiscard]] GEO::index_t cell_edge_cv(const GEO::index_t c, const GEO::index_t le, const GEO::index_t lv) const {
            assert(c < mesh_.cells.nb());
            return element_control_nodes_[c*CONTROL_POINTS_NB_PER_CELL + edge_lcv(le, lv)];
        }

        /**
         * Get an internal control point index by local edge index and local vertex index in the edge.
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[in] le local edge index, 0,1,...,11
         * @param[in] lv local vertex index in the edge (ev0 -> ev1), 0,1,...,order-2
         * @return control point index
         */
        [[nodiscard]] GEO::index_t cell_edge_inner_cv(const GEO::index_t c, const GEO::index_t le, const GEO::index_t lv) const {
            assert(c < mesh_.cells.nb());
            return element_control_nodes_[c*CONTROL_POINTS_NB_PER_CELL + edge_inner_lcv(le, lv)];
        }

        /**
         * Get a control point index by local facet index and two local vertex index in the facet.
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[in] lf local facet index, 0,1,...,5
         * @param[in] lv0 local vertex index in the facet (fv0 -> fv1), 0,1,...,order
         * @param[in] lv1 local vertex index in the facet (fv0 -> fv3), 0,1,...,order
         * @return control point index
         */
        [[nodiscard]] GEO::index_t cell_facet_cv(const GEO::index_t c, const GEO::index_t lf, const GEO::index_t lv0, const GEO::index_t lv1) const {
            assert(c < mesh_.cells.nb());
            return element_control_nodes_[c*CONTROL_POINTS_NB_PER_CELL + facet_lcv(lf, lv0, lv1)];
        }

        /**
         * Get an internal control point index by local facet index and two local vertex indexes in the facet.
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[in] lf local facet index, 0,1,...,5
         * @param[in] lv0 local vertex index in the facet (fv0 -> fv1), 0,1,...,order-2
         * @param[in] lv1 local vertex index in the facet (fv0 -> fv3), 0,1,...,order-2
         * @return control point index
         */
        [[nodiscard]] GEO::index_t cell_facet_inner_cv(const GEO::index_t c, const GEO::index_t lf, const GEO::index_t lv0, const GEO::index_t lv1) const {
            assert(c < mesh_.cells.nb());
            return element_control_nodes_[c*CONTROL_POINTS_NB_PER_CELL + facet_inner_lcv(lf, lv0, lv1)];
        }

        /**
         * Get a control point index by flattened local control-point index in one cell.
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[in] lv flattened local control-point index, 0,1,...,CONTROL_POINTS_NB_PER_CELL-1
         * @return control point index
         */
        [[nodiscard]] GEO::index_t cell_cv(const GEO::index_t c, const GEO::index_t lv) const {
            assert(c < mesh_.cells.nb());
            assert(lv < CONTROL_POINTS_NB_PER_CELL);
            return element_control_nodes_[c*CONTROL_POINTS_NB_PER_CELL + lv];
        }

        /**
         * Get a control point index by three local vertex indexes in the cell
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[in] lv0 local vertex index in the facet (cv0 -> cv1), 0,1,...,order
         * @param[in] lv1 local vertex index in the facet (cv0 -> cv2), 0,1,...,order
         * @param[in] lv2 local vertex index in the facet (cv0 -> cv4), 0,1,...,order
         * @return control point index
         */
        [[nodiscard]] GEO::index_t cell_cv(const GEO::index_t c, const GEO::index_t lv0, const GEO::index_t lv1, const GEO::index_t lv2) const {
            assert(c < mesh_.cells.nb());
            return element_control_nodes_[c*CONTROL_POINTS_NB_PER_CELL + cell_lcv(lv0, lv1, lv2)];
        }

        /**
         * Get an internal control point index by three local vertex indexes in the cell
         * @param[in] c cell index, 0,1,...,hex_mesh.cells.nb()-1
         * @param[in] lv0 local vertex index in the facet (cv0 -> cv1), 0,1,...,order-2
         * @param[in] lv1 local vertex index in the facet (cv0 -> cv2), 0,1,...,order-2
         * @param[in] lv2 local vertex index in the facet (cv0 -> cv4), 0,1,...,order-2
         * @return control point index
         */
        [[nodiscard]] GEO::index_t cell_inner_cv(const GEO::index_t c, const GEO::index_t lv0, const GEO::index_t lv1, const GEO::index_t lv2) const {
            assert(c < mesh_.cells.nb());
            return element_control_nodes_[c*CONTROL_POINTS_NB_PER_CELL + cell_inner_lcv(lv0, lv1, lv2)];
        }

        /**
         * Get the ref of the control point in the grid.
         * @param[in] cv global control point index
         * @return
         */
        [[nodiscard]] auto& get_cv_position(const GEO::index_t cv) {
            assert(cv < control_nodes_.size());
            return control_node(cv);
        }

        /**
         * Get the const ref of the control point in the grid.
         * @param[in] cv global control point index
         * @return
         */
        [[nodiscard]] const auto& get_cv_position(const GEO::index_t cv) const {
            assert(cv < control_nodes_.size());
            return control_node(cv);
        }

        /**
         * @brief Map a parametric coordinate inside a cell to physical space.
         *
         * Evaluates the high-order cell mapping at the given parametric coordinate
         * `uvw` ∈ [0,1]^3 and returns the corresponding physical-space point. The
         * mapping is built from the cell's control-point positions stored in the
         * internal control grid `grid_`.
         *
         * @param[in] c Index of the hexahedral cell (0..hex_mesh_.cells.nb()-1).
         * @param[in] uvw Parametric coordinate in the cell-local domain [0,1]^3.
         *
         * @return Physical-space position corresponding to the input parametric point.
         *
         * @note Preconditions: `c` must be a valid cell index and the components of
         *       `uvw` are expected to be in [0,1] for meaningful results. The
         *       function uses the control-point coordinates currently stored in
         *       `grid_` to compute the mapped position.
         */
        [[nodiscard]] GEO::vec3 compute_cell_uvw_position(GEO::index_t c, const GEO::vec3& uvw) const;

        /**
         * @brief Map a parametric coordinate to physical space using provided control positions.
         *
         * Variant of `compute_cell_uvw_position` that evaluates the cell mapping
         * using an externally supplied flat array of control-node positions. This
         * is useful for evaluating hypothetical configurations (e.g., during
         * optimization) without modifying the internal control grid.
         *
         * @param[in] c Index of the hexahedral cell (0..hex_mesh_.cells.nb()-1).
         * @param[in] uvw Parametric coordinate in the cell-local domain [0,1]^3.
         * @param[in] cur_control_nodes_ptr Pointer to a flat array of control-node
         *            coordinates in the same layout used by the optimizer
         *            (`[x0,y0,z0,x1,y1,z1,...]` for all control vertices). The
         *            array must contain at least `3 * control_points_nb()` entries.
         *
         * @return Physical-space position corresponding to the input parametric point
         *         evaluated with the provided control-node positions.
         *
         * @note The function does not take ownership of `control_nodes_position` and
         *       treats it as read-only. Caller must ensure the buffer is valid.
         */
        [[nodiscard]] GEO::vec3 compute_cell_uvw_position(GEO::index_t c, const GEO::vec3& uvw, const double* cur_control_nodes_ptr) const;

        /**
         * Evaluate the (non-unit) physical normal of a high-order cell facet.
         *
         * The normal is computed from the cross product of two facet tangents,
         * obtained by differentiating the facet mapping with respect to its local
         * parametric coordinates.
         *
         * @param[in] c Cell index, 0,1,...,hex_mesh.cells.nb()-1.
         * @param[in] lf Local facet index in the cell, 0,1,...,5.
         * @param[in] uv Facet-local parameter point `(u, v)` in [0, 1]^2.
         *          - u: facet-local parameter along the first facet axis (from cell_facet_vertex 0 to 1), typically in [0,1]
         *          - v: facet-local parameter along the second facet axis (from cell_facet_vertex 0 to 3), typically in [0,1]
         * @return Outward facet normal vector at the given parameter point in physical space.
         * @pre `c < hex_mesh.cells.nb()`.
         * @pre `lf < 6`.
         * @pre `parameter_point.x` and `parameter_point.y` are in [0, 1].
         * @note The returned vector is not normalized; its magnitude equals the local area scaling.
         */
        [[nodiscard]] GEO::vec3 compute_cell_facet_uv_normal(GEO::index_t c, GEO::index_t lf, const GEO::vec2& uv) const;

        /**
         * Evaluate the first-order parametric derivatives of the cell mapping at a parameter point.
         *
         * This helper computes the physical-space tangent vectors with respect to the three parametric
         * directions, together with the corresponding 1D basis values and basis derivatives used in the
         * tensor-product evaluation.
         *
         * @param[in] c Cell index, 0,1,...,hex_mesh.cells.nb()-1.
         * @param[in] uvw Parameter point in the cell parameter domain [0, 1]^3.
         * @param[out] du Output tangent vector for partial derivative with respect to u.
         * @param[out] dv Output tangent vector for partial derivative with respect to v.
         * @param[out] dw Output tangent vector for partial derivative with respect to w.
         * @param[out] Bu Output buffer that receives the 1D basis values in the u direction.
         * @param[out] Bv Output buffer that receives the 1D basis values in the v direction.
         * @param[out] Bw Output buffer that receives the 1D basis values in the w direction.
         * @param[out] dBu Output buffer that receives the 1D basis derivatives in the u direction.
         * @param[out] dBv Output buffer that receives the 1D basis derivatives in the v direction.
         * @param[out] dBw Output buffer that receives the 1D basis derivatives in the w direction.
         */
        void compute_cell_uvw_dudvdw(GEO::index_t c, const GEO::vec3& uvw,
            GEO::vec3& du, GEO::vec3& dv, GEO::vec3& dw,
            std::vector<double>& Bu, std::vector<double>& Bv, std::vector<double>& Bw,
            std::vector<double>& dBu, std::vector<double>& dBv, std::vector<double>& dBw) const;

    protected:
        GEO::index_t CONTROL_POINTS_NB_PER_EDGE = GEO::NO_INDEX;
        GEO::index_t CONTROL_POINTS_NB_PER_FACET = GEO::NO_INDEX;
        GEO::index_t CONTROL_POINTS_NB_PER_CELL = GEO::NO_INDEX;
        GEO::index_t INTERNAL_CONTROL_POINTS_NB_PER_EDGE = GEO::NO_INDEX;
        GEO::index_t INTERNAL_CONTROL_POINTS_NB_PER_FACET = GEO::NO_INDEX;
        GEO::index_t INTERNAL_CONTROL_POINTS_NB_PER_CELL = GEO::NO_INDEX;
        std::vector<GEO::index_t> VERTEX_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<GEO::index_t> EDGE_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<int>          EDGE_CONTROL_POINTS_NEXT_IDX_STEP_{};
        std::vector<GEO::index_t> EDGE_INTERNAL_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<int>          EDGE_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP_{};
        std::vector<GEO::index_t> FACET_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<int>          FACET_CONTROL_POINTS_NEXT_IDX_STEP0_{};
        std::vector<int>          FACET_CONTROL_POINTS_NEXT_IDX_STEP1_{};
        std::vector<GEO::index_t> FACET_INTERNAL_CONTROL_POINTS_BEGIN_IDX_{};
        std::vector<int>          FACET_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP0_{};
        std::vector<int>          FACET_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP1_{};
        GEO::index_t              CELL_CONTROL_POINTS_BEGIN_IDX_{};
        int                       CELL_CONTROL_POINTS_NEXT_IDX_STEP0_{};
        int                       CELL_CONTROL_POINTS_NEXT_IDX_STEP1_{};
        int                       CELL_CONTROL_POINTS_NEXT_IDX_STEP2_{};
        GEO::index_t              CELL_INTERNAL_CONTROL_POINTS_BEGIN_IDX_{};
        int                       CELL_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP0_{};
        int                       CELL_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP1_{};
        int                       CELL_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP2_{};
    };
}

#endif //GEOLIO_CONTROL_GRID_H
