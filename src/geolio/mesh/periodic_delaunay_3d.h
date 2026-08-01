//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_PERIODIC_DELAUNAY_3D_H
#define GEOLIO_PERIODIC_DELAUNAY_3D_H

#include <geogram/delaunay/periodic_delaunay_3d.h>
#include <geogram/mesh/mesh.h>

namespace geolio
{
    /**
     * @brief Appends all vertices and tetrahedra from a periodic Delaunay
     *        structure to a mesh.
     * @details Creates new mesh vertices at the end of @p mesh and copies the
     *          coordinates of every Delaunay vertex in index order. It then
     *          creates one tetrahedron per Delaunay cell and fills each local
     *          corner with the newly appended mesh vertex corresponding to the
     *          cell's local vertex index. Existing mesh vertices and cells are
     *          preserved; only new elements are appended.
     * @param[in] delaunay Source periodic Delaunay triangulation to export.
     * @param[in, out] mesh Destination mesh receiving appended vertices and
     *        tetrahedra.
     */
    inline void append_PeriodicDelaunay3d_to_mesh(
        const GEO::PeriodicDelaunay3d& delaunay,
        GEO::Mesh& mesh
        ) {
        const auto DELAUNAY_VERTICES_NB = delaunay.nb_vertices();
        const GEO::index_t new_v = mesh.vertices.create_vertices(DELAUNAY_VERTICES_NB);
        for (GEO::index_t v = 0; v < DELAUNAY_VERTICES_NB; ++v)
            mesh.vertices.point(new_v+v) = delaunay.vertex(v);

        const auto DELAUNAY_CELLS_NB = delaunay.nb_cells();
        const GEO::index_t new_c = mesh.cells.create_tets(DELAUNAY_CELLS_NB);
        for (GEO::index_t c = 0; c < DELAUNAY_CELLS_NB; ++c) {
            for (GEO::index_t lv = 0; lv < 4; ++lv)
                mesh.cells.set_vertex(new_c+c, lv, new_v+delaunay.cell_vertex(c, lv));
        }
    }
}

#endif //GEOLIO_PERIODIC_DELAUNAY_3D_H
