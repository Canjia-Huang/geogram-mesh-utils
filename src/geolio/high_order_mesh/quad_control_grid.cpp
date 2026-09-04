//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/4.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "quad_control_grid.h"
#include <geolio/common/pair_hash.h>

namespace geolio
{
    template<GEO::index_t DIM>
    QuadControlGrid<DIM>::QuadControlGrid(
        const GEO::Mesh& mesh,
        GEO::index_t order
        ) : SurfaceControlGrid<DIM>(mesh, order)
    {
        assert([&]() {
            for (const auto& f : this->mesh_.facets) {
                if (this->mesh_.facets.nb_vertices(f) != 4)
                   return false;
            }
            return true;
        }());

        QuadControlGrid::initialize_nodes_arrangement();
        QuadControlGrid::initialize_control_nodes();
    }

    template<GEO::index_t DIM>
    void QuadControlGrid<DIM>::initialize_nodes_arrangement(
        ) {

        const GEO::index_t LAST_LAYER_BEGIN_IDX = (this->CONTROL_POINTS_NB_PER_EDGE_-1)*this->CONTROL_POINTS_NB_PER_EDGE_;

        /* == Vertex =============================================================================================== */
        this->ELEMENT_VERTEX_CONTROL_POINTS_BEGIN_IDX_ = {
            0,
            this->CONTROL_POINTS_NB_PER_EDGE_-1,
            this->CONTROL_POINTS_NB_PER_FACET_-1,
            LAST_LAYER_BEGIN_IDX
        };

        /* == Edge ================================================================================================= */
        this->ELEMENT_EDGE_CONTROL_POINTS_BEGIN_IDX_ = this->ELEMENT_VERTEX_CONTROL_POINTS_BEGIN_IDX_;
        this->ELEMENT_EDGE_CONTROL_POINTS_NEXT_IDX_STEP_ = {
            1,
            static_cast<int>(this->CONTROL_POINTS_NB_PER_EDGE_),
            -1,
            -static_cast<int>(this->CONTROL_POINTS_NB_PER_EDGE_),
        };
        this->ELEMENT_EDGE_INTERNAL_CONTROL_POINTS_BEGIN_IDX_ = {
            1,
            2*this->CONTROL_POINTS_NB_PER_EDGE_-1,
            this->CONTROL_POINTS_NB_PER_FACET_-2,
            (this->CONTROL_POINTS_NB_PER_EDGE_-2)*this->CONTROL_POINTS_NB_PER_EDGE_
        };
        this->ELEMENT_EDGE_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP_ = this->ELEMENT_EDGE_CONTROL_POINTS_NEXT_IDX_STEP_;

        /* == Facet ================================================================================================ */
        this->ELEMENT_CONTROL_POINTS_BEGIN_IDX_ = 0;
        this->ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP0_ = 1;
        this->ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP1_ = static_cast<int>(this->CONTROL_POINTS_NB_PER_EDGE_);
        this->ELEMENT_INTERNAL_CONTROL_POINTS_BEGIN_IDX_ = this->CONTROL_POINTS_NB_PER_EDGE_+1;
        this->ELEMENT_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP0_ = this->ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP0_;
        this->ELEMENT_INTERNAL_CONTROL_POINTS_NEXT_IDX_STEP1_ = this->ELEMENT_CONTROL_POINTS_NEXT_IDX_STEP1_;
    }

    template<GEO::index_t DIM>
    void QuadControlGrid<DIM>::initialize_control_nodes(
        ) {
        assert(this->node_positions_1D_.size() == this->order_+1);

        /* == Get all shared edges ================================================================================= */
        std::unordered_map<std::pair<GEO::index_t, GEO::index_t>, std::vector<GEO::index_t>, PairHash> quad_edges_control_points; /* (ev0, ev1), ev0 < ev1 -> control vertices from ev0 -> ev1 */
        {
            for (const auto& f : this->mesh_.facets) {
                for (GEO::index_t lv = 0; lv < 4; ++lv) {
                    const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(
                        this->mesh_.facets.vertex(f, lv),
                        this->mesh_.facets.vertex(f, (lv+1)%4));
                    quad_edges_control_points.emplace(edge, std::vector<GEO::index_t>(this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_, GEO::NO_VERTEX));
                }
            }
        }

        /* == Create grid elements ================================================================================= */
        this->control_nodes_.vertices.clear();
        GEO::index_t new_v = this->control_nodes_.vertices.create_vertices(
                            this->mesh_.vertices.nb() + // vertices
                            quad_edges_control_points.size() * this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_ + // edges
                            this->mesh_.facets.nb() * this->INTERNAL_CONTROL_POINTS_NB_PER_FACET_ // facets
                            );
        assert(new_v == 0);

        /* == For vertices == */
        for (const auto& v : this->mesh_.vertices)
            this->control_node(new_v++) = this->mesh_.vertices.template point<DIM>(v);

        /* == For edges == */
        for (auto& [edge, control_vertices] : quad_edges_control_points) {
            const auto& ep0 = this->mesh_.vertices.template point<DIM>(edge.first);
            const auto& ep1 = this->mesh_.vertices.template point<DIM>(edge.second);
            for (GEO::index_t i = 0, i_end = this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; i < i_end; ++i) {
                const double r = this->node_positions_1D_[i+1];
                this->control_node(new_v) = (1-r)*ep0 + r*ep1;
                control_vertices[i] = new_v;
                ++new_v;
            }
        }

        /* == For facets == */
        std::vector<std::vector<GEO::index_t>> quad_facets_control_points(this->mesh_.facets.nb()); /*
            [f] -> the idx of control points of this facet  */
        for (const auto& f : this->mesh_.facets) {
            auto& f_control_points = quad_facets_control_points[f];
            f_control_points.reserve(this->INTERNAL_CONTROL_POINTS_NB_PER_FACET_);

            assert(this->mesh_.facets.nb_vertices(f) == 4);
            const auto& f_p0 = this->mesh_.facets.template point<DIM>(f, 0);
            const auto& f_p1 = this->mesh_.facets.template point<DIM>(f, 1);
            const auto& f_p2 = this->mesh_.facets.template point<DIM>(f, 2);
            const auto& f_p3 = this->mesh_.facets.template point<DIM>(f, 3);

            for (GEO::index_t i = 0; i < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++i) {
                const double ri = this->node_positions_1D_[i+1];
                for (GEO::index_t j = 0; j < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++j) {
                    const double rj = this->node_positions_1D_[j+1];

                    this->control_node(new_v) = (1-ri)*(1-rj)*f_p0
                                              + ri*(1-rj)*f_p3
                                              + (1-ri)*rj*f_p1
                                              + ri*rj*f_p2;
                        f_control_points.push_back(new_v);
                        ++new_v;
                }
            }
        }

        assert(new_v == this->control_nodes_nb());

        /* == Create regular index ================================================================================= */
        this->element_control_nodes_.assign(this->CONTROL_POINTS_NB_PER_FACET_ * this->mesh_.facets.nb(), GEO::NO_VERTEX);

        for (const auto& f : this->mesh_.facets) {
            const GEO::index_t FACET_BEGIN_IDX = f*this->CONTROL_POINTS_NB_PER_FACET_;

            /* For vertices */
            for (GEO::index_t lv = 0; lv < 4; ++lv)
                this->element_control_nodes_[
                        FACET_BEGIN_IDX +
                        this->facet_vertex_lcv(lv)
                        ] = this->mesh_.facets.vertex(f, lv);

            /* For edges */
            for (GEO::index_t le = 0; le < 4; ++le) {
                const auto& ev0 = this->mesh_.facets.vertex(f, le);
                const auto& ev1 = this->mesh_.facets.vertex(f, (le+1)%4);
                const std::pair<GEO::index_t, GEO::index_t> edge = std::minmax(ev0, ev1);

                assert(quad_edges_control_points.contains(edge));
                const auto& edge_control_points = quad_edges_control_points.at(edge);
                assert(edge_control_points.size() == this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_);

                if (ev0 == edge.first) { // do not need to inverse
                    for (GEO::index_t lv = 0; lv < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv)
                        this->element_control_nodes_[
                            FACET_BEGIN_IDX +
                            this->facet_edge_inner_lcv(le, lv)
                            ] = edge_control_points[lv];
                }
                else { // need to inverse
                    for (GEO::index_t lv = 0; lv < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv)
                        this->element_control_nodes_[
                            FACET_BEGIN_IDX +
                            this->facet_edge_inner_lcv(le, lv)
                            ] = edge_control_points[this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_-1-lv];
                }
            }

            /* For facets */
            const auto& facet_control_points = quad_facets_control_points[f];
            assert(facet_control_points.size() == this->INTERNAL_CONTROL_POINTS_NB_PER_CELL_);

            for (GEO::index_t lv1 = 0; lv1 < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv1) {
                for (GEO::index_t lv0 = 0; lv0 < this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_; ++lv0) {
                    this->element_control_nodes_[
                        FACET_BEGIN_IDX +
                        this->facet_inner_lcv(lv0, lv1)
                        ] = facet_control_points[lv1*this->INTERNAL_CONTROL_POINTS_NB_PER_EDGE_ + lv0];
                }
            }
        }
    }

    template class QuadControlGrid<2>;
    template class QuadControlGrid<3>;
}