//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "detect_mesh_defects.h"
#include <unordered_set>
#include <geolio/common/array_hash.h>
#include "mesh_operations.h"

namespace geolio
{
    GEO::index_t detect_duplicate_triangles(
        const GEO::Mesh& mesh,
        std::vector<GEO::index_t>& duplicate_triangles
        ) {
        std::vector<bool> mesh_f_duplicate(mesh.facets.nb(), false);
        std::unordered_set<std::array<GEO::index_t, 3>, Array3Hash<GEO::index_t>> duplicate_triangles_set;
        for (const auto& f : mesh.facets) {
            if (mesh.facets.nb_vertices(f) != 3)
                continue;

            std::array<GEO::index_t, 3> fvs = {
                mesh.facets.vertex(f, 0),
                mesh.facets.vertex(f, 1),
                mesh.facets.vertex(f, 2)
            };
            std::ranges::sort(fvs);

            if (!duplicate_triangles_set.insert(fvs).second)
                mesh_f_duplicate[f] = true;
        }

        /* Output */
        duplicate_triangles.clear();
        for (const auto& f : mesh.facets) {
            if (mesh_f_duplicate[f])
                duplicate_triangles.push_back(f);
        }

        return duplicate_triangles.size();
    }

    GEO::index_t detect_non_manifold_vertices(
        const GEO::Mesh& mesh,
        std::vector<GEO::index_t>& non_manifold_vertices,
        const bool pseudo_manifold
        ) {
        std::vector<GEO::index_t> mesh_v_adjacent_facets_nb(mesh.vertices.nb(), 0);
        for (const auto& f : mesh.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                const auto& v = mesh.facets.vertex(f, lv);
                ++mesh_v_adjacent_facets_nb[v];
            }
        }

        std::vector<std::pair<GEO::index_t, GEO::index_t>> ordered_f_and_lv; // pre-allocated
        for (const auto& f : mesh.facets) {
            for (GEO::index_t lv = 0; lv < 3; ++lv) {
                const auto& v = mesh.facets.vertex(f, lv);
                if (mesh_v_adjacent_facets_nb[v] == GEO::NO_INDEX) // already check
                    continue;

                const auto ON_BORDER = get_vertex_incident_facets(mesh, f, lv, ordered_f_and_lv);
                if (ordered_f_and_lv.size() != mesh_v_adjacent_facets_nb[v]) // non-manifold vertex
                    non_manifold_vertices.push_back(v);
                if (!pseudo_manifold && ON_BORDER)
                    non_manifold_vertices.push_back(v);

                mesh_v_adjacent_facets_nb[v] = GEO::NO_INDEX; // label
            }
        }

        return non_manifold_vertices.size();
    }
}
