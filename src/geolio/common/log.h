//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/7/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LOG_H
#define GEOLIO_LOG_H

#include <spdlog/spdlog.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/sinks/sink.h>

#ifdef ERROR
#undef ERROR
#endif

namespace geolio::LOG {
    namespace detail {
        /**
         * @brief Captures the file name, function name and line number of a
         *        log call site, captured at compile time via compiler builtins.
         */
        class SourceLocation {
        public:
            /**
             * @brief Constructs a SourceLocation from the given call-site info.
             * @details Defaults to the compiler's builtin FILE/FUNCTION/LINE
             *          macros, so a log statement can be located automatically.
             * @param[in] fileName The source file name.
             * @param[in] funcName The enclosing function name.
             * @param[in] lineNum The line number in the source file.
             */
            constexpr SourceLocation(const char *fileName = __builtin_FILE(),
                                     const char *funcName = __builtin_FUNCTION(),
                                     std::uint32_t lineNum = __builtin_LINE()) noexcept
                    : _fileName(fileName), _funcName(funcName), _lineNum(lineNum) {
            }

            /**
             * @brief Returns the captured source file name.
             * @return The source file name.
             */
            [[nodiscard]] constexpr const char *FileName() const noexcept {
                return _fileName;
            }

            /**
             * @brief Returns the captured enclosing function name.
             * @return The function name.
             */
            [[nodiscard]] constexpr const char *FuncName() const noexcept {
                return _funcName;
            }

            /**
             * @brief Returns the captured source line number.
             * @return The line number.
             */
            [[nodiscard]] constexpr std::uint32_t LineNum() const noexcept {
                return _lineNum;
            }

        private:
            const char *_fileName;
            const char *_funcName;
            const std::uint32_t _lineNum;
        };

        /**
         * @brief Converts a SourceLocation into an spdlog source_loc.
         * @details Extracts the file name and line number from the location and
         *          passes them together with the function name to the spdlog
         *          source_loc constructor.
         * @param[in] location The compile-time captured call-site location.
         * @return The corresponding spdlog::source_loc.
         */
        constexpr auto GetLogSourceLocation(const SourceLocation &location) {
            return spdlog::source_loc{location.FileName(), static_cast<int>(location.LineNum()),
                                      location.FuncName()};
        }

    } // namespace detail

    /**
     * @brief Logs a trace-level message, including the call-site location.
     * @details Wraps spdlog's trace logging so that each log statement
     *          automatically carries its compile-time source location.
     */
    template<typename... Args>
    struct TRACE {
        /**
         * @brief Emits a trace-level log message.
         * @details Forwards the formatted message and arguments to spdlog with
         *          the given (or default-captured) source location.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         * @param[in] location The call-site location, auto-captured if omitted.
         */
        explicit constexpr TRACE(fmt::format_string<Args...> fmt, Args &&...args, const detail::SourceLocation &location = {}) {
            spdlog::log(GetLogSourceLocation(location), spdlog::level::trace, fmt, std::forward<Args>(args)...);
        }
    };

    /**
     * @brief CTAD deduction guide for TRACE.
     * @details Deduces Args from the format string and the forwarded arguments.
     */
    template<typename... Args>
    TRACE(fmt::format_string<Args...> fmt, Args &&...args) -> TRACE<Args...>;

    /**
     * @brief Logs a debug-level message, including the call-site location.
     * @details Wraps spdlog's debug logging so that each log statement
     *          automatically carries its compile-time source location.
     */
    template<typename... Args>
    struct DEBUG {
        /**
         * @brief Emits a debug-level log message.
         * @details Forwards the formatted message and arguments to spdlog with
         *          the given (or default-captured) source location.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         * @param[in] location The call-site location, auto-captured if omitted.
         */
        explicit constexpr DEBUG(fmt::format_string<Args...> fmt, Args &&...args, const detail::SourceLocation &location = {}) {
            spdlog::log(GetLogSourceLocation(location), spdlog::level::debug, fmt, std::forward<Args>(args)...);
        }
    };

    /**
     * @brief CTAD deduction guide for DEBUG.
     * @details Deduces Args from the format string and the forwarded arguments.
     */
    template<typename... Args>
    DEBUG(fmt::format_string<Args...> fmt, Args &&...args) -> DEBUG<Args...>;

    /**
     * @brief Logs an info-level message, including the call-site location.
     * @details Wraps spdlog's info logging so that each log statement
     *          automatically carries its compile-time source location.
     */
    template<typename... Args>
    struct INFO {
        /**
         * @brief Emits an info-level log message.
         * @details Forwards the formatted message and arguments to spdlog with
         *          the given (or default-captured) source location.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         * @param[in] location The call-site location, auto-captured if omitted.
         */
        explicit constexpr INFO(fmt::format_string<Args...> fmt, Args &&...args, const detail::SourceLocation &location = {}) {
            spdlog::log(GetLogSourceLocation(location), spdlog::level::info, fmt, std::forward<Args>(args)...);
        }
    };

    /**
     * @brief CTAD deduction guide for INFO.
     * @details Deduces Args from the format string and the forwarded arguments.
     */
    template<typename... Args>
    INFO(fmt::format_string<Args...> fmt, Args &&...args) -> INFO<Args...>;

    /**
     * @brief Logs a warning-level message, including the call-site location.
     * @details Wraps spdlog's warning logging so that each log statement
     *          automatically carries its compile-time source location.
     */
    template<typename... Args>
    struct WARN {
        /**
         * @brief Emits a warning-level log message.
         * @details Forwards the formatted message and arguments to spdlog with
         *          the given (or default-captured) source location.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         * @param[in] location The call-site location, auto-captured if omitted.
         */
        explicit constexpr WARN(fmt::format_string<Args...> fmt, Args &&...args, const detail::SourceLocation &location = {}) {
            spdlog::log(GetLogSourceLocation(location), spdlog::level::warn, fmt, std::forward<Args>(args)...);
        }
    };

    /**
     * @brief CTAD deduction guide for WARN.
     * @details Deduces Args from the format string and the forwarded arguments.
     */
    template<typename... Args>
    WARN(fmt::format_string<Args...> fmt, Args &&...args) -> WARN<Args...>;

    /**
     * @brief Logs an error-level message, including the call-site location.
     * @details Wraps spdlog's error logging so that each log statement
     *          automatically carries its compile-time source location.
     */
    template<typename... Args>
    struct ERROR {
        /**
         * @brief Emits an error-level log message.
         * @details Forwards the formatted message and arguments to spdlog with
         *          the given (or default-captured) source location.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         * @param[in] location The call-site location, auto-captured if omitted.
         */
        explicit constexpr ERROR(fmt::format_string<Args...> fmt, Args &&...args, const detail::SourceLocation &location = {}) {
            spdlog::log(GetLogSourceLocation(location), spdlog::level::err, fmt, std::forward<Args>(args)...);
        }
    };

    /**
     * @brief CTAD deduction guide for ERROR.
     * @details Deduces Args from the format string and the forwarded arguments.
     */
    template<typename... Args>
    ERROR(fmt::format_string<Args...> fmt, Args &&...args) -> ERROR<Args...>;

    /**
     * @brief Logs a critical-level message, including the call-site location.
     * @details Wraps spdlog's critical logging so that each log statement
     *          automatically carries its compile-time source location.
     */
    template<typename... Args>
    struct CRITICAL {
        /**
         * @brief Emits a critical-level log message.
         * @details Forwards the formatted message and arguments to spdlog with
         *          the given (or default-captured) source location.
         * @param[in] fmt The format string describing the message.
         * @param[in] args The arguments to be formatted into the message.
         * @param[in] location The call-site location, auto-captured if omitted.
         */
        explicit constexpr CRITICAL(fmt::format_string<Args...> fmt, Args &&...args, const detail::SourceLocation &location = {}) {
            spdlog::log(GetLogSourceLocation(location), spdlog::level::critical, fmt, std::forward<Args>(args)...);
        }
    };

    /**
     * @brief CTAD deduction guide for CRITICAL.
     * @details Deduces Args from the format string and the forwarded arguments.
     */
    template<typename... Args>
    CRITICAL(fmt::format_string<Args...> fmt, Args &&...args) -> CRITICAL<Args...>;
} // namespace LOG

#endif //GEOLIO_LOG_H
