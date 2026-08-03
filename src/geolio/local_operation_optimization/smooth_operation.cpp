//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "smooth_operation.h"
#include <cassert>
#include <utility>
#include <vector>

#include "geolio/common/log.h"

namespace geolio
{
    template<GEO::index_t DIM>
    SmoothOperation<DIM>::SmoothOperation(
        MeshElementManager<DIM>& mesh_element_manager,
        const GEO::index_t geometric_constraint,
        const bool allow_smooth_fixed_edge_vertices
        ) : BaseOperation<DIM>(mesh_element_manager),
            GEOMETRIC_CONSTRAINT_(geometric_constraint),
            ALLOW_SMOOTH_FIXED_EDGE_VERTICES_(allow_smooth_fixed_edge_vertices)
    {
        /* Collect the adjacent vertices of a vertex. */
        mesh_v_adjacent_v.assign(this->mesh_.vertices.nb(), std::vector<GEO::index_t>());
        for (const auto& f : this->mesh_.facets) {
            if (!this->manager_.mesh_f_used[f])
                continue;

            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                const auto& v = this->mesh_.facets.vertex(f, lv);
                assert(this->manager_.mesh_v_used[v]);
                const auto& nv = this->mesh_.facets.vertex(f, (lv+1)%3);
                assert(this->manager_.mesh_v_used[nv]);

                mesh_v_adjacent_v[v].push_back(nv);
                if (this->mesh_.facets.adjacent(f, lv) == GEO::NO_FACET)
                    mesh_v_adjacent_v[nv].push_back(v);
            }
        }

        /* Prepare for smooth fixed edge vertices */
        if (ALLOW_SMOOTH_FIXED_EDGE_VERTICES_) {
            mesh_fixed_edge_v_adjacent_v.assign(this->mesh_.vertices.nb(), std::pair(GEO::NO_VERTEX, GEO::NO_VERTEX));

            for (const auto& f : this->mesh_.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (const auto& fc = this->mesh_.facets.corner(f, lv);
                        this->manager_.mesh_fc_fixed[fc]
                        ) {
                        const auto& ev0 = this->mesh_.facets.vertex(f, lv);
                        const auto& ev1 = this->mesh_.facets.vertex(f, (lv+1)%3);
                        {
                            if (auto& [adj_v0, adj_v1] = mesh_fixed_edge_v_adjacent_v[ev0];
                                adj_v0 == GEO::NO_VERTEX)
                                adj_v0 = ev1;
                            else if (adj_v1 == GEO::NO_VERTEX) {
                                if (adj_v0 != ev1)
                                    adj_v1 = ev1;
                            }
                        }
                        {
                            if (auto& [adj_v0, adj_v1] = mesh_fixed_edge_v_adjacent_v[ev1];
                                adj_v0 == GEO::NO_VERTEX)
                                adj_v0 = ev0;
                            else if (adj_v1 == GEO::NO_VERTEX) {
                                if (adj_v0 != ev0)
                                    adj_v1 = ev0;
                            }
                        }
                    }
                }
            }

            /* Fix fixed vertex */
            for (const auto& v : this->mesh_.vertices) {
                if (this->manager_.mesh_v_fixed[v])
                    mesh_fixed_edge_v_adjacent_v[v] = std::pair(GEO::NO_VERTEX, GEO::NO_VERTEX);
            }
        }
        else {
            mesh_v_on_fixed_edges_.assign(this->mesh_.vertices.nb(), false);
            for (const auto& f : this->mesh_.facets) {
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (const auto& fc = this->mesh_.facets.corner(f, lv);
                        this->manager_.mesh_fc_fixed[fc]
                        ) {
                        const auto& ev0 = this->mesh_.facets.vertex(f, lv);
                        const auto& ev1 = this->mesh_.facets.vertex(f, (lv+1)%3);
                        mesh_v_on_fixed_edges_[ev0] = true;
                        mesh_v_on_fixed_edges_[ev1] = true;
                    }
                }
            }
        }

        /* Predetermine which vertices are involved in smoothing. */
        mesh_v_perform_.assign(this->mesh_.vertices.nb(), true);
        for (const auto& v : this->mesh_.vertices) {
            if (!is_perform_valid(v))
                mesh_v_perform_[v] = false;
        }

        /* Pre-allocated */
        mesh_v_new_pos.resize(this->mesh_.vertices.nb());

        /* Init AABB tree (for 3D mesh). */
        if constexpr (DIM == 3) {
            if (GEOMETRIC_CONSTRAINT_ == PROJECT_TO_ORIGINAL_MESH) {
                original_mesh_.copy(this->mesh_);
                {
                    GEO::vector<GEO::index_t> facets_to_delete(original_mesh_.facets.nb(), 0);
                    for (const auto& f : original_mesh_.facets) {
                        if (!this->manager_.mesh_f_used[f])
                            facets_to_delete[f] = 1;
                    }
                    original_mesh_.facets.delete_elements(facets_to_delete, true);
                }
                original_mesh_facet_AABB_.initialize(original_mesh_);
            }
        }
    }

    template<GEO::index_t DIM>
    double SmoothOperation<DIM>::do_once(
        ) {
        /* Compute vertex normal */
        std::vector<GEO::vec3> mesh_v_normal; // normalized
        if constexpr (DIM == 3) {
            if (GEOMETRIC_CONSTRAINT_ == TANGENTIAL_SMOOTHING) {
                mesh_v_normal.assign(this->mesh_.vertices.nb(), GEO::vec3());
                for (const auto& f : this->mesh_.facets) {
                    if (!this->manager_.mesh_f_used[f])
                        continue;

                    const auto normal = GEO::normalize(
                        GEO::Geom::triangle_normal(
                            this->mesh_.facets.point(f, 0),
                            this->mesh_.facets.point(f, 1),
                            this->mesh_.facets.point(f, 2)));
                    for (GEO::index_t lv = 0; lv < 3; ++lv) {
                        const auto& v = this->mesh_.facets.vertex(f, lv);
                        mesh_v_normal[v] += normal;
                    }
                }

                for (const auto& v : this->mesh_.vertices) {
                    if (!mesh_v_perform_[v])
                        continue;
                    mesh_v_normal[v] = GEO::normalize(mesh_v_normal[v]);
                }
            }
        }

        /* Compute target position */
        for (const auto& v : this->mesh_.vertices) {
            if (!mesh_v_perform_[v])
                continue;

            auto& original_p = this->mesh_.vertices.template point<DIM>(v);

            /* Average */
            auto& target_p = mesh_v_new_pos[v];
            target_p = GEO::vecng<DIM, GEO::Numeric::float64>();
            for (const auto& nv : mesh_v_adjacent_v[v])
                target_p += this->mesh_.vertices.template point<DIM>(nv);
            assert(!mesh_v_adjacent_v[v].empty());
            target_p /= mesh_v_adjacent_v[v].size();

            /* == Constraints ====================================================================================== */

            /* Slide along fixed edge */
            if (ALLOW_SMOOTH_FIXED_EDGE_VERTICES_) {
                assert(mesh_fixed_edge_v_adjacent_v.size() == this->mesh_.vertices.nb());
                assert(!this->manager_.mesh_v_fixed[v]);
                const auto& [v0, v1] = mesh_fixed_edge_v_adjacent_v[v];
                if (v0 == GEO::NO_VERTEX || v1 == GEO::NO_VERTEX)
                    continue;
                const auto& p0 = this->mesh_.vertices.template point<DIM>(v0);
                const auto& p1 = this->mesh_.vertices.template point<DIM>(v1);
                // const auto p1p0 = GEO::normalize(p1-p0);
                // target_p = p0 + GEO::dot(target_p-p0, p1p0) * p1p0;
                target_p = 0.5*(p0+p1);
            }

            /* Project to original mesh */
            if constexpr (DIM == 3) {
                if (GEOMETRIC_CONSTRAINT_ == PROJECT_TO_ORIGINAL_MESH) {
                    GEO::vec3 nearest_pos;
                    double sq_dist;
                    original_mesh_facet_AABB_.nearest_facet(target_p, nearest_pos, sq_dist);

                    target_p = nearest_pos;
                }
                else if (GEOMETRIC_CONSTRAINT_ == TANGENTIAL_SMOOTHING) {
                    assert(v < mesh_v_normal.size());

                    auto dir = target_p - original_p;
                    dir = dir - GEO::dot(dir, mesh_v_normal[v]) * mesh_v_normal[v];
                    target_p = original_p + dir;
                }
            }

            /* Damping */
            target_p = damping_factor_ * original_p + (1-damping_factor_) * target_p;
        }

        /* Update */
        double max_displacement = -std::numeric_limits<double>::max();
        for (const auto& v : this->mesh_.vertices) {
            if (!mesh_v_perform_[v])
                continue;

            auto& original_p = this->mesh_.vertices.template point<DIM>(v);
            auto& target_p = mesh_v_new_pos[v];

            /* Record max displacement */
            max_displacement = std::max(
                max_displacement,
                GEO::distance(original_p, target_p));

            original_p = target_p;
        }

        return true;
    }

    template<GEO::index_t DIM>
    void SmoothOperation<DIM>::run_nb_times(
        const GEO::index_t iterations_nb
        ) {
        for (GEO::index_t iteration = 0; iteration < iterations_nb; ++iteration)
            do_once();
    }

    template<GEO::index_t DIM>
    void SmoothOperation<DIM>::run_until(
        const double displacement_threshold
        ) {
        while (do_once() > displacement_threshold) {}
    }

    template<GEO::index_t DIM>
    bool SmoothOperation<DIM>::is_perform_valid(
        const GEO::index_t v
        ) const {
        assert(v < this->mesh_.vertices.nb());


        if (!this->manager_.mesh_v_used[v]) // This vertex should not yet exist.
            return false;

        if (this->manager_.mesh_v_fixed[v]) // Cannot move fixed vertex.
            return false;

        if (!ALLOW_SMOOTH_FIXED_EDGE_VERTICES_) {
            assert(mesh_v_on_fixed_edges_.size() == this->mesh_.vertices.nb());
            if (mesh_v_on_fixed_edges_[v])
                return false;
        }

        return true;
    }

    template class SmoothOperation<2>;
    template class SmoothOperation<3>;
}
