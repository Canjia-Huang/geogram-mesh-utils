//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//

/**
 * This file is based on <geogram/basic/line_stream.cpp>
 */

#include "line_stream.h"
#include <geogram/basic/logger.h>

#include <cctype>
#include <cstdio>
#include <cstring>
#include <sstream>

namespace geolio
{
    LineInput::LineInput(const std::string& filename) :
        file_name_(filename),
        line_num_(0)
    {
        F_ = fopen(filename.c_str(), "r");
        ok_ = (F_ != nullptr);
    }

    LineInput::~LineInput() {
        if(F_ != nullptr) {
            fclose(F_);
            F_ = nullptr;
        }
    }

    bool LineInput::read_line(std::string& out) {
        out.clear();
        // Reads the line in chunks, so that lines of arbitrary length
        // are read completely (a fixed size buffer would truncate them).
        char buf[4096];
        while(fgets(buf, sizeof(buf), F_) != nullptr) {
            out += buf;
            if(!out.empty() && out.back() == '\n') {
                return true;
            }
            if(feof(F_)) {
                return true;
            }
        }
        return !out.empty();
    }

    bool LineInput::get_line() {
        if(F_ == nullptr) {
            return false;
        }
        line_.clear();
        // Skip the empty lines
        while(true) {
            if(!read_line(line_)) {
                return false;
            }
            ++line_num_;
            const unsigned char first =
                static_cast<unsigned char>(line_[0]);
            if(isprint(first) || first == '\t') {
                break;
            }
        }
        // If the line ends with a backslash, append
        // the next line to the current line.
        bool check_multiline = true;
        while(check_multiline) {
            const bool ends_with_eol =
                !line_.empty() &&
                (line_.back() == '\n' || line_.back() == '\r');
            size_t last = line_.size();
            while(last > 0 &&
                  (line_[last - 1] == '\n' || line_[last - 1] == '\r')) {
                --last;
            }
            if(ends_with_eol && last > 0 && line_[last - 1] == '\\') {
                // Replace the backslash with a space and drop the
                // end-of-line sequence, then read the next line and
                // append it to the current line.
                line_[last - 1] = ' ';
                line_.erase(last);
                std::string continuation;
                if(!read_line(continuation)) {
                    return false;
                }
                ++line_num_;
                line_ += continuation;
            } else {
                check_multiline = false;
            }
        }
        return true;
    }

#if 1
#ifdef GEO_OS_WINDOWS
#define safe_strtok strtok_s
#else
#define safe_strtok strtok_r
#endif

    void LineInput::get_fields(const char* separators) {
        field_.resize(0);
        char* context = nullptr;
        char* tok = safe_strtok(line_.data(), separators, &context);
        while(tok != nullptr) {
            field_.push_back(tok);
            tok = safe_strtok(nullptr, separators, &context);
        }
    }

#else
    void LineInput::get_fields(const char* separators) {
        field_.resize(0);
        char* tok = strtok(line_.data(), separators);
        while(tok != nullptr) {
            field_.push_back(tok);
            tok = strtok(nullptr, separators);
        }
    }

#endif

    void LineInput::conversion_error(const GEO::index_t index, const char* type) const {
        std::ostringstream out;
        out << "Line " << line_num_
            << ": field #" << index
            << " is not a valid " << type << " value: " << field(index);
        throw std::logic_error(out.str());
    }
}
