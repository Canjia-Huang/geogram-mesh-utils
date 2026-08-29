//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/29.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_APPLICATION_REGISTRY_H
#define GEOLIO_APPLICATION_REGISTRY_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "base_application.h"
#include "geolio/geobox/object/base_object.h"

namespace geolio::geobox
{
    /** Objects container shared by all sub-applications. */
    using ApplicationObjects = std::vector<std::shared_ptr<BaseObject>>;

    /**
     * Registry of plugin-style sub-applications (BaseApplication subclasses).
     * Mirrors the MeshIOHandler factory pattern from geolio/io/io.h: concrete
     * applications register a creator under a stable id at startup, and
     * GeoBoxApplication instantiates every registered application afterwards.
     * Unlike the meshio case, sub-applications need constructor arguments
     * (the display name and the objects container owned by GeoBoxApplication),
     * so the creator signature carries both instead of being a Factory0.
     */
    class BaseApplicationRegistry {
    public:
        /**
         * @brief Creator function type for a registered sub-application.
         * @details All registered concrete types must provide a constructor
         *          (std::string application_name, const ApplicationObjects&),
         *          which is the current convention (see
         *          LocalOperationOptimizationApplication).
         */
        using Creator = std::unique_ptr<BaseApplication>(*)(
            const std::string& display_name, const ApplicationObjects& objects);

        /**
         * @brief One registered sub-application: stable id (the lookup key),
         *        display name (the menu / window title) and its creator.
         */
        struct Entry {
            std::string id;
            std::string display_name;
            Creator creator;
        };

        /**
         * @brief Helper class to register a creator at function-local static
         *        initialization, mirroring GEO::Factory::RegisterCreator.
         * @tparam ConcreteType the sub-application class to register.
         */
        template <class ConcreteType>
        struct RegisterCreator {
            /**
             * @brief Registers @p ConcreteType under @p id.
             * @param[in] id stable identifier of the application.
             * @param[in] display_name user-facing name shown in the menu and
             *            as the window title.
             */
            RegisterCreator(const std::string& id, const std::string& display_name) {
                BaseApplicationRegistry::instance().register_creator(
                    id, display_name, &BaseApplicationRegistry::create<ConcreteType>);
            }
        };

        /**
         * @brief Gets the registry unique instance.
         */
        static BaseApplicationRegistry& instance() {
            static BaseApplicationRegistry registry;
            return registry;
        }

        /**
         * @brief Generic creator: instantiates @p ConcreteType with the
         *        display name and the objects container.
         * @tparam ConcreteType the sub-application class to instantiate.
         */
        template <class ConcreteType>
        static std::unique_ptr<BaseApplication> create(
            const std::string& display_name, const ApplicationObjects& objects) {
            return std::make_unique<ConcreteType>(display_name, objects);
        }

        /**
         * @brief Registers a creator under a stable id.
         * @param[in] id stable identifier of the application.
         * @param[in] display_name user-facing name shown in the menu and as
         *            the window title.
         * @param[in] creator the creation function.
         */
        void register_creator(std::string id, std::string display_name, Creator creator) {
            entries_.push_back({std::move(id), std::move(display_name), creator});
        }

        /**
         * @brief Finds a registered entry by its stable id.
         * @param[in] id stable identifier of the application.
         * @retval pointer to the entry if found.
         * @retval nullptr otherwise.
         */
        [[nodiscard]] const Entry* find(const std::string& id) const {
            for (const auto& entry : entries_) {
                if (entry.id == id)
                    return &entry;
            }
            return nullptr;
        }

        /**
         * @brief Lists all registered entries in registration order (which is
         *        the "Application" menu order).
         */
        [[nodiscard]] const std::vector<Entry>& entries() const {
            return entries_;
        }

    private:
        std::vector<Entry> entries_;
    };
}

#define GEOLIO_CONCAT_IMPL(a, b) a##b
#define GEOLIO_CONCAT(a, b) GEOLIO_CONCAT_IMPL(a, b)

/**
 * @brief Helper macro to register a BaseApplication subclass.
 * @details Declares a function-local static RegisterCreator object that, on
 *          the first call of the enclosing function, registers the creator of
 *          @p type under the stable id @p id (displayed as @p display_name).
 *          Mirrors geo_register_MeshIOHandler_creator() from geolio/io/io.h.
 * @param[in] type the ConcreteType subclass of BaseApplication.
 * @param[in] id stable identifier of the application (lookup key).
 * @param[in] display_name user-facing name shown in the menu and as the
 *            window title.
 */
#define geolio_register_BaseApplication_creator(type, id, display_name) \
    static geolio::geobox::BaseApplicationRegistry::RegisterCreator<type> \
        GEOLIO_CONCAT(geolio_register_application_creator_, __LINE__)(id, display_name)

#endif //GEOLIO_APPLICATION_REGISTRY_H
