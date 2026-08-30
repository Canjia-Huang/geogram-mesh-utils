//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "offset_restricted_voronoi_diagram.h"
#include <cassert>
#include <geolio/common/log.h>

namespace geolio
{
    OffsetRestrictedVoronoiDiagram::OffsetRestrictedVoronoiDiagram(
        const GEO::Mesh& mesh
        ) : mesh_(mesh)
    {}

    void OffsetRestrictedVoronoiDiagram::set_sites(
        const double* sites,
        const GEO::index_t sites_nb
        ) {
        assert(sites != nullptr);
        sites_ = sites;
        sites_nb_ = sites_nb;
    }

    void OffsetRestrictedVoronoiDiagram::compute(
        const double d
        ) {
        LOG::TRACE("{}({})", __FUNCTION__, d);
        if (d < 0)
            throw std::invalid_argument("negative d");
        if (sites_ == nullptr)
            throw std::invalid_argument("sites_ == nullptr");
        if (sites_nb_ == GEO::NO_INDEX)
            throw std::invalid_argument("sites_nb_ == GEO::NO_INDEX");
    }
}