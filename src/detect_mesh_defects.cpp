//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "geolio/detect_mesh_defects.h"
#include "geolio/array_hash.h"
#include <unordered_set>

namespace geolio
{
    GEO::index_t detect_duplicate_triangles(
        const GEO::Mesh& mesh,
        std::vector<GEO::index_t>& duplicate_triangles
        ) {
        std::vector<bool> mesh_f_duplicate(mesh.facets.nb(), false);
        std::unordered_map<std::array<GEO::index_t, 3>, GEO::index_t, Array3Hash<GEO::index_t>> duplicate_triangles_set;
        for (const auto& f : mesh.facets) {
            if (mesh.facets.nb_vertices(f) != 3)
                continue;

            std::array<GEO::index_t, 3> fvs = {
                mesh.facets.vertex(f, 0),
                mesh.facets.vertex(f, 1),
                mesh.facets.vertex(f, 2)
            };
            std::ranges::sort(fvs);

            if (auto it = duplicate_triangles_set.find(fvs);
                it != duplicate_triangles_set.end())
                duplicate_triangles_set.emplace(fvs, f);
            else {
                mesh_f_duplicate[it->second] = true;
                mesh_f_duplicate[f] = true;
            }
        }

        /* Output */
        duplicate_triangles.clear();
        for (const auto& f : mesh.facets) {
            if (mesh_f_duplicate[f])
                duplicate_triangles.push_back(f);
        }

        return duplicate_triangles.size();
    }
}