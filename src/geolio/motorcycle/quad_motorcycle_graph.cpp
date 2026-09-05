//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/9/5.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "quad_motorcycle_graph.h"
#include <cassert>
#include <geolio/common/utils.h>

namespace geolio
{
    QuadMotorCycleGraph::QuadMotorCycleGraph(
        const GEO::Mesh& mesh
        ) : attribute_id_(geolio::generate_random_string(22)),
            mesh_(mesh)
    {
        assert([&]() {
            for (const auto& f : mesh_.facets) {
                if (mesh_.facets.nb_vertices(f) != 4)
                    return false;
            }
            return true;
        }()); // check all-quad mesh

        mesh_fc_tagged_.bind(mesh_.facet_corners.attributes(), attribute_id_+":tagged");
        mesh_fc_tagged_.fill(GEO::NO_INDEX);

        find_all_singular_and_border_vertices();
    }

    QuadMotorCycleGraph::~QuadMotorCycleGraph(
        ) {
        if (mesh_fc_tagged_.is_bound())
            mesh_fc_tagged_.destroy();
    }

    GEO::index_t QuadMotorCycleGraph::compute(
        const QuadMotorCycleGraphComplexType complex_type
        ) {
        std::priority_queue<Fire> queue;

        /* Ignite */
        ignite(queue);
    }

    void QuadMotorCycleGraph::find_all_singular_and_border_vertices(
        ) {
        mesh_v_singular_.assign(mesh_.vertices.nb(), false);
        mesh_v_border_.assign(mesh_.vertices.nb(), false);

        std::vector<GEO::index_t> v_incident_facets_nb(mesh_.vertices.nb(), 0);
        for (const auto& f : mesh_.facets) {
            for (GEO::index_t lv = 0; lv < 4; ++lv) {
                ++v_incident_facets_nb[mesh_.facets.vertex(f, lv)];

                if (mesh_.facets.adjacent(f, lv) == GEO::NO_FACET) {
                    mesh_v_border_[mesh_.facets.vertex(f, lv)] = true;
                    mesh_v_border_[mesh_.facets.vertex(f, (lv+1)%4)] = true;
                }
            }
        }

        /* Label singular vertex */
        for (const auto& v : mesh_.vertices) {
            if (mesh_v_border_[v]) {
                if (v_incident_facets_nb[v] != 2)
                    mesh_v_singular_[v] = true;
            }
            else {
                if (v_incident_facets_nb[v] != 4)
                    mesh_v_singular_[v] = true;
            }
        }
    }

    void QuadMotorCycleGraph::ignite(
        std::priority_queue<Fire>& queue
        ) const {
        assert(mesh_v_singular_.size() == mesh_.vertices.nb());
        assert(queue.empty());

        while (!queue.empty())
            queue.pop();

        for (const auto& v : mesh_.vertices) {
            if (!mesh_v_singular_[v]) // for all singular vertex
                continue;

            /* Find all incident interior edges */
        }
    }
}