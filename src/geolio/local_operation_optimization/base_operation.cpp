//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/30.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#include "base_operation.h"
#include <geogram/mesh/mesh_io.h>
#include <geolio/common/log.h>

namespace geolio
{
    /**
     * @brief Constructs a BaseOperation bound to the given mesh element manager.
     * @details Stores references to the manager and the underlying mesh, generates a random
     *          attribute name via generate_random_string(), and binds a per-facet "processed"
     *          attribute under that name on the mesh facets, pre-filling it with false.
     * @param[in] mesh_element_manager The mesh element manager exposing the mesh and its
     *                                 usage/fixed element attributes.
     */
    BaseOperation::BaseOperation(
        MeshElementManager& mesh_element_manager
        ) : manager_(mesh_element_manager),
            mesh_(mesh_element_manager.mesh),
            attribute_name_(generate_random_string(22))
    {
        /* Bind attributes */
        mesh_f_processed_.bind(mesh_.facets.attributes(), attribute_name_+":processed");
        mesh_f_processed_.fill(false);
    }

    /**
     * @brief Destroys the BaseOperation.
     * @details Destroys the bound "processed" facet attribute if it is still bound,
     *          releasing the underlying mesh attribute storage.
     */
    BaseOperation::~BaseOperation(
        ) {
        /* Destroy attributes */
        if (mesh_f_processed_.is_bound())
            mesh_f_processed_.destroy();
    }

    /**
     * @brief Validates the consistency of the mesh element usage state.
     * @details Runs a set of consistency checks that are currently disabled at compile
     *          time (if constexpr(false)): used facets must only reference used vertices,
     *          the adjacent facets of a used facet must be used, and fixed facet corners
     *          must appear in matching pairs. In the current build it always returns true.
     * @return true if the mesh passes all checks (always true in the current build).
     */
    bool BaseOperation::post_check(
        ) {
        if constexpr (false) { // Check whether all used facets use the used vertices.
            for (const auto& f : mesh_.facets) {
                if (!manager_.mesh_f_used[f])
                    continue;
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (const auto& v = mesh_.facets.vertex(f, lv);
                        !manager_.mesh_v_used[v])
                        return false;
                }
            }
        }
        if constexpr (false) { // Adjacent facets of a used facet should also be detected as used.
            for (const auto& f : mesh_.facets) {
                if (!manager_.mesh_f_used[f])
                    continue;
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (const auto& nf = mesh_.facets.adjacent(f, lv);
                        nf != GEO::NO_FACET)
                        assert(manager_.mesh_f_used[nf]);
                }
            }
        }
        if constexpr (false) { // Fixed mesh facet corner should appear in pairs.
            for (const auto& f : mesh_.facets) {
                if (!manager_.mesh_f_used[f])
                    continue;
                for (GEO::index_t lv = 0; lv < 3; ++lv) {
                    if (const auto& nf = mesh_.facets.adjacent(f, lv);
                        nf != GEO::NO_FACET) {
                        if (manager_.mesh_fc_fixed[mesh_.facets.corner(f, lv)]) {
                            const auto& v = mesh_.facets.vertex(f, lv);
                            const auto nlv = mesh_.facets.find_vertex(nf, v);
                            assert(nlv != GEO::NO_INDEX);
                            assert(manager_.mesh_fc_fixed[mesh_.facets.corner(nf, (nlv+2)%3)]);
                        }
                    }
                }
            }
        }

        return true;
    }
}
