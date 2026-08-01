//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_TEST_ENVIRONMENT_SPDLOG_H
#define GEOLIO_TEST_ENVIRONMENT_SPDLOG_H

#include <gtest/gtest.h>
#include <memory>
#include <vector>
#include <geolio/common/log.h>
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

class SpdlogTestEnvironment final : public testing::Environment {
public:
    void SetUp() override {
        const auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        const auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("test_spdlog.log", true);
        std::vector<spdlog::sink_ptr> sinks{console_sink, file_sink};
        const auto logger = std::make_shared<spdlog::logger>("GEOLIO_TEST", sinks.begin(), sinks.end());
        logger->flush_on(spdlog::level::info);
        logger->set_level(spdlog::level::trace);
        spdlog::set_default_logger(logger);
    }
};

#endif //GEOLIO_TEST_ENVIRONMENT_SPDLOG_H
