//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_OFFSET_RESTRICTED_VORONOI_DIAGRAM_H
#define GEOLIO_OFFSET_RESTRICTED_VORONOI_DIAGRAM_H
#include <geogram/mesh/mesh.h>

namespace geolio
{
    class OffsetRestrictedVoronoiDiagram {
    public:
        explicit OffsetRestrictedVoronoiDiagram(const GEO::Mesh& mesh);

        void set_sites(const double* sites, GEO::index_t sites_nb);

        void compute(double d);

    private:
        const GEO::Mesh& mesh_;
        const double* sites_ = nullptr;
        GEO::index_t sites_nb_ = GEO::NO_INDEX;
    };
}

#endif //GEOLIO_OFFSET_RESTRICTED_VORONOI_DIAGRAM_H
