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
     *
     * The function copies all CDT vertices into the mesh, then creates one
     * mesh triangle for each CDT triangle using the copied vertex indices.
     *
     * @param CDT The source 2D constrained Delaunay triangulation.
     * @param mesh The destination mesh, which must already be 2D.
     */
    inline void append_CDT2d_to_mesh(
        const GEO::CDT2d& CDT,
        GEO::Mesh& mesh
        ) {
        assert(mesh.vertices.dimension() == 2);

        const GEO::index_t new_v = mesh.vertices.create_vertices(CDT.nv());
        for (GEO::index_t v = 0, v_end = CDT.nv(); v < v_end; ++v)
            mesh.vertices.point<2>(new_v+v) = CDT.point(v);

        const GEO::index_t new_f = mesh.facets.create_triangles(CDT.nT());
        for (GEO::index_t f = 0, f_end = CDT.nT(); f < f_end; ++f) {
            for (GEO::index_t lv = 0; lv < 3; ++lv)
                mesh.facets.set_vertex(new_f+f, lv, new_v+CDT.Tv(f, lv));
        }
    }
}

#endif //GEOLIO_CDT_2D_H
