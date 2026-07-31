//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_TRI_LOCAL_OPERATION_OPTIMIZATION_H
#define GEOLIO_TRI_LOCAL_OPERATION_OPTIMIZATION_H

#include <cassert>
#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_AABB.h>

#include "collapse_operation.h"
#include "mesh_element_manager.h"
#include "smooth_operation.h"
#include "split_operation.h"
#include "swap_operation.h"

namespace geolio
{
    class TriLocalOperationOptimization {
    public:
        explicit TriLocalOperationOptimization(GEO::Mesh& mesh);


        /**
         * @brief Runs the local optimization pipeline.
         *
         * @param[in] rounds_nb Number of optimization rounds to execute. Each
         * round runs split, collapse, swap and smooth passes in that order.
         * @param[in] target_edge_length Desired target edge length used to
         * guide split/collapse decisions. If negative, the optimizer computes
         * an automatic target using compute_average_mesh_edge_length().
         *
         * @details
         * The method performs the following steps per round:
         * - bind_attributes() to create temporary usage attributes
         * - split_edges(target_edge_length * 1.5) to refine long edges
         * - collapse_edges(target_edge_length * 0.5) to remove short edges
         * - swap_edges() to improve mesh valence
         * - smooth_vertices(1) to relax vertex positions
         * - clean_unused_elements() to remove released facets
         * After the requested rounds, unbind_attributes() is called to clean up.
         */
        void optimize(
            GEO::index_t rounds_nb = 5,
            double target_edge_length = -1);

        /**
         * @brief Marks a specific vertex as fixed.
         *
         * @param[in] v Vertex index to lock.
         *
         * @details
         * A fixed vertex is excluded from smoothing and from local operations
         * that would move it. The caller must ensure `v` is a valid vertex
         * index in `mesh_`.
         */
        void fix_vertex(
            const GEO::index_t v
            ) {
            assert(v < mesh_.vertices.nb());
            manager_.mesh_v_fixed[v] = true;
        }

        void fix_edge(
            const GEO::index_t f,
            const GEO::index_t lv
            ) {
            assert(f < mesh_.facets.nb());
            assert(lv < 3);
            fix_edge(mesh_.facets.corner(f, lv));
            if (const auto nf = mesh_.facets.adjacent(f, lv);
                nf != GEO::NO_FACET) {
                const auto v = mesh_.facets.vertex(f, lv);
                const auto nlv = mesh_.facets.find_vertex(nf, v);
                assert(nlv != GEO::NO_INDEX);
                fix_edge(mesh_.facets.corner(nf, (nlv+2)%3));
            }
        }

        void fix_edge(
            const GEO::index_t fc
            ) {
            assert(fc < mesh_.facet_corners.nb());
            manager_.mesh_fc_fixed[fc] = true;
        }

        void fix_boundary_elements();

        void fix_sharp_elements(double sharp_angle = 0.75*M_PI);

    protected:
        void label_boundary_vertices();

        void label_non_manifold_vertices();

        GEO::Mesh& mesh_;
        MeshElementManager manager_;
    };
}

#endif //GEOLIO_TRI_LOCAL_OPERATION_OPTIMIZATION_H
