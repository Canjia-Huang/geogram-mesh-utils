//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "detect_mesh_defects.h"
#include <array>
#include <ranges>
#include <unordered_set>
#include <geolio/common/array_hash.h>
#include "mesh_operations.h"

namespace geolio
{
    /**
     * @brief Detects duplicate triangles in a mesh.
     * @details Scans every facet, skipping non-triangular faces, and builds a
     *          sorted key from each triangle's three vertex indices. Each key
     *          is inserted into an unordered set (keyed by `Array3Hash`); the
     *          first time a key is seen the triangle is kept, and any later
     *          triangle sharing the same key is flagged as a duplicate in a
     *          per-facet boolean array. The flagged indices are then collected
     *          into @p duplicate_triangles.
     * @param[in] mesh The input mesh to inspect.
     * @param[out] duplicate_triangles Output list that receives the indices of
     *        triangles identified as duplicates.
     * @return The number of duplicate triangles found.
     */
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

    /**
     * @brief Detects non-manifold vertices in a mesh.
     * @details First counts the number of facets incident to each vertex in
     *          `mesh_v_adjacent_facets_nb`. Then, for each facet vertex not yet
     *          processed, calls `get_vertex_incident_facets` to traverse the
     *          connected component of facets around that vertex. A vertex is
     *          classified as non-manifold when the traversal does not reach all
     *          of its incident facets, or when `pseudo_manifold` is false and
     *          the traversal stops at the mesh border. Processed vertices are
     *          labelled with `GEO::NO_INDEX` in the count array to avoid
     *          re-processing.
     * @param[in] mesh The input mesh to inspect.
     * @param[out] non_manifold_vertices Output list that receives the indices
     *            of vertices classified as non-manifold.
     * @param[in] pseudo_manifold Whether to also treat pseudo-manifold
     *            vertices as defects.
     * @return The number of non-manifold vertices found.
     */
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
