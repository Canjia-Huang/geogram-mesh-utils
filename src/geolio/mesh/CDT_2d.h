//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/27.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_CDT_2D_H
#define GEOLIO_CDT_2D_H

#include <cassert>
#include <geogram/delaunay/CDT_2d.h>
#include <geogram/mesh/mesh.h>

namespace geolio
{
    /**
     * @brief Appends a 2D constrained Delaunay triangulation to a mesh.
     * @details Creates new mesh vertices for every CDT vertex and copies their
     *          coordinates from the triangulation, embedding 2D points as
     *          (x, y, 0) when the mesh dimension is 3. It then creates one mesh
     *          triangle per CDT triangle and fills each corner using the CDT
     *          vertex indices remapped to the newly appended mesh vertices.
     * @param[in] CDT The source 2D constrained Delaunay triangulation.
     * @param[in, out] mesh The destination mesh.
     */
    inline void append_CDT2d_to_mesh(
        const GEO::CDT2d& CDT,
        GEO::Mesh& mesh
        ) {
        const GEO::index_t new_v = mesh.vertices.create_vertices(CDT.nv());
        if (mesh.vertices.dimension() == 2) {
            for (GEO::index_t v = 0, v_end = CDT.nv(); v < v_end; ++v)
                mesh.vertices.point<2>(new_v+v) = CDT.point(v);
        }
        else {
            assert(mesh.vertices.dimension() == 3);

            for (GEO::index_t v = 0, v_end = CDT.nv(); v < v_end; ++v) {
                const auto& p = CDT.point(v);
                mesh.vertices.point(new_v+v) = GEO::vec3(p.x, p.y, 0);
            }
        }

        const GEO::index_t new_f = mesh.facets.create_triangles(CDT.nT());
        for (GEO::index_t f = 0, f_end = CDT.nT(); f < f_end; ++f) {
            for (GEO::index_t lv = 0; lv < 3; ++lv)
                mesh.facets.set_vertex(new_f+f, lv, new_v+CDT.Tv(f, lv));
        }
    }
}

#endif //GEOLIO_CDT_2D_H
