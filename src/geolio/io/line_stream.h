//
// Created by huangcanjia <huangcanjia0214@gmail.com> on 2026/8/26.
// Copyright (c) 2026 Graphics@XMU (https://graphics.xmu.edu.cn). All rights reserved.
//
#ifndef GEOLIO_LINE_STREAM_H
#define GEOLIO_LINE_STREAM_H

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <geogram/basic/numeric.h>
#include <geogram/basic/string.h>
#include <cassert>

/**
 * This file is based on <geogram/basic/line_stream.h>
 *
 * \note Modification vs. the original geogram implementation:
 * - The original class stored the current line in a fixed size buffer
 *   (char line_[MAX_LINE_LEN], 65535 bytes), so lines longer than that
 *   were truncated and the leftover part of the line was returned as a
 *   bogus extra line by the following get_line() call. In this version
 *   the line buffer is a dynamically growing std::string (see read_line()),
 *   so lines of arbitrary length are read completely.
 * - get_bracket_keyword() was added: it consumes a keyword(...)
 *   construct at the start of the current line, e.g.
 *   get_bracket_keyword("(") on "Elements(223852, ...)" returns the
 *   whitespace-trimmed keyword "Elements" and keeps the content inside
 *   the parentheses ("223852, ...") as the new line. It throws
 *   std::logic_error when the token is empty, and returns an empty
 *   string when the token is not found in the line.
 * - field_as_appro_double() was added: it converts a field written as a
 *   rational number "p/q" with arbitrarily large integers (as produced
 *   by Ansys AEDT files) into an approximate double, e.g.
 *   "-9910370759100826714601236637/3271414488397938850655109120".
 */
namespace geolio
{

    /**
     * \brief Reads an ASCII file line per line
     * \details LineInput reads an ASCII file line by line and splits
     * the line into a list of white space separated fields that can be
     * accessed individually or converted to numeric values.
     *
     * Functions field_as_int() and field_as_double() throw exceptions when
     * they cannot convert a field to a integer or floating point value, so
     * it is safe to wrap the LineNumber usage in a try / catch block as
     * follows:
     *
     * \code
     * try {
     *     LineInput in(filename);
     *     while( !in.eof() && in.get_line() ) {
     *         in.get_fields();
     *         double d = in.field_as_double(2);
     *     }
     * }
     * catch(const std::logic_error& ex) {
     *     std::cerr << "Got an error: " << ex.what() << std::endl;
     * }
     * \endcode
     */
    class LineInput {
    public:
    /**
     * \brief Creates a new line reader from a file
     * \details This open the file \p filename for reading and prepares to
     * read it line by line. If the file could not be opened, OK() will
     * return false;
     * \param[in] filename the name of the file to read
     */
    LineInput(const std::string& filename);

    /**
     * \brief Destroys the line reader
     * \details This closes the current input file.
     */
    ~LineInput();

    /**
     * \brief Checks if the line reader is ready to read.
     */
    [[nodiscard]] bool OK() const {
        return ok_;
    }

    /**
     * \brief Checks if line reader has reached the end of the input stream
     * \retval true if the stream is at end
     * \retval false otherwise
     */
    [[nodiscard]] bool eof() const {
        return feof(F_) ? true : false;
    }

    /**
     * \brief Reads a new line
     * \details Reads a new line from the input stream. Function
     * get_fields() must be called if you need to access to individual
     * fields in the line with field() and its typed variants.
     * \retval true if a line could be read
     * \retval false otherwise.
     */
    bool get_line();

    /**
     * \brief Gets the number of fields in the current line
     * \details Function get_fields() must be called once after get_line()
     * before calling this function, otherwise the result is undefined.
     * \return the number of fields in the current line
     */
    [[nodiscard]] GEO::index_t nb_fields() const {
        return static_cast<GEO::index_t>(field_.size());
    }

    /**
     * \brief Returns the current line number
     * \details If no line has been read so far, line_number() returns 0.
     */
    [[nodiscard]] size_t line_number() const {
        return line_num_;
    }

    /**
     * \brief Gets a line field as a modifiable string
     * \details The function returns the field at index \p i. Function
     * get_fields() must be called once after get_line() before calling
     * this function, otherwise the result is undefined.
     * \param[in] i the index of the field
     * \return the modifiable pointer to field string at index \p i
     */
    [[nodiscard]] char* field(const GEO::index_t i) {
        assert(i < nb_fields());
        return field_[i];
    }

    /**
     * \brief Gets a line field as a non-modifiable string
     * \details The function returns the field at index \p i. Function
     * get_fields() must be called once after get_line() before calling
     * this function, otherwise the result is undefined.
     * \param[in] i the index of the field
     * \return the const pointer to field string at index \p i
     */
    [[nodiscard]] const char* field(const GEO::index_t i) const {
        assert(i < nb_fields());
        return field_[i];
    }

    /**
     * \brief Gets a line field as an integer.
     * \details The function returns the field at index \p i converted to
     * an integer. Function get_fields() must be called once after
     * get_line() before calling this function, otherwise the result is
     * undefined.
     * \param[in] i the index of the field
     * \return the integer value of the field at index \p i
     * \exception std::logic_error if the field cannot be converted to an
     * integer value
     */
    [[nodiscard]] GEO::signed_index_t field_as_int(GEO::index_t i) const {
        GEO::signed_index_t result = 0;
        if(!GEO::String::from_string(field(i), result)) {
            conversion_error(i, "integer");
        }
        return result;
    }

    /**
     * \brief Gets a line field as an unsigned integer.
     * \details The function returns the field at index \p i converted to
     * an unsigned integer. Function get_fields() must be called once after
     * get_line() before calling this function, otherwise the result is
     * undefined.
     * \param[in] i the index of the field
     * \return the unsigned integer value of the field at index \p i
     * \exception std::logic_error if the field cannot be converted to an
     * unsigned integer value
     */
    [[nodiscard]] GEO::index_t field_as_uint(GEO::index_t i) const {
        GEO::index_t result = 0;
        if(!GEO::String::from_string(field(i), result)) {
            conversion_error(i, "unsigned integer");
        }
        return result;
    }

    /**
     * \brief Gets a line field as a double.
     * \details The function returns the field at index \p i converted to
     * a double. Function get_fields() must be called once after
     * get_line() before calling this function, otherwise the result is
     * undefined.
     * \param[in] i the index of the field
     * \return the floating point value of the field at index \p i
     * \exception std::logic_error if the field cannot be converted to a
     * floating point value
     */
    [[nodiscard]] double field_as_double(GEO::index_t i) const {
        double result = 0;
        if(!GEO::String::from_string(field(i), result)) {
            conversion_error(i, "floating point");
        }
        return result;
    }

    /**
     * \brief Gets a line field as an approximate double.
     * \details The function returns the field at index \p i converted to
     * a double. Function get_fields() must be called once after
     * get_line() before calling this function, otherwise the result is
     * undefined.
     * \details A field written as a rational number "p/q" with
     * arbitrarily large integers (e.g. "-9910370759100826714601236637/
     * 3271414488397938850655109120", as produced by Ansys AEDT files)
     * is converted to an approximate double. Fields that do not contain
     * a '/' are converted exactly like field_as_double().
     * \param[in] i the index of the field
     * \return the floating point value of the field at index \p i
     * \exception std::logic_error if the field cannot be converted to a
     * floating point value
     */
    [[nodiscard]] double field_as_appro_double(GEO::index_t i) const {
        const char* s = field(i);
        const char* slash = strchr(s, '/');
        if(slash == nullptr) {
            double result = 0;
            if(!GEO::String::from_string(s, result))
                conversion_error(i, "floating point");
            return result;
        }
        // Skips spaces, tabs and end-of-line characters.
        const auto skip_ws = [](const char* p) {
            while(*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') {
                ++p;
            }
            return p;
        };
        // Numerator: parse up to the '/', tolerating surrounding whitespace.
        errno = 0;
        char* end_num = nullptr;
        const long double num = strtold(s, &end_num);
        if(end_num == s || skip_ws(end_num) != slash || errno == ERANGE) {
            conversion_error(i, "floating point");
        }
        // Denominator: must be a non-empty, non-zero, fully consumed number.
        const char* den_start = skip_ws(slash + 1);
        errno = 0;
        char* end_den = nullptr;
        const long double den = strtold(den_start, &end_den);
        if(end_den == den_start || *skip_ws(end_den) != '\0' ||
           den == 0.0L || errno == ERANGE) {
            conversion_error(i, "floating point");
        }
        return static_cast<double>(num / den);
    }

    /**
     * \brief Compares a field with a string.
     * \details The function compares the field at index \p i with string
     * \p s and returns \c true if they are equal. Function get_fields()
     * must be called once after get_line() before calling this function,
     * otherwise the result is undefined.
     * \param[in] i the index of the field
     * \param[in] s the string to compare the field to
     * \retval true if field at index \p i equals string \p s
     * \retval false otherwise
     */
    bool field_matches(GEO::index_t i, const char* s) const {
        return strcmp(field(i), s) == 0;
    }

    /**
     * \brief Splits the current line into fields.
     * \details The function uses \p separators to split the
     * current line into individual fields that can be accessed
     * by field() and its typed variants.
     * \param[in] separators a string that contains all
     *  the characters considered as separators.
     * \see field()
     */
    void get_fields(const char* separators = " \t\r\n");

    /**
     * \brief Gets the keyword of a keyword(...) construct at the start
     * of the current line.
     * \details Extracts the keyword before the first occurrence of
     * \p token (typically '('), removes it from the current line
     * together with the token, and keeps the content between the token
     * and its matching closing delimiter (typically ')') as the new
     * current line. The returned keyword is trimmed of leading and
     * trailing whitespace.
     * \param[in] token the opening delimiter, e.g. "("
     * \return the trimmed keyword that was before \p token, or an empty
     * string if \p token is not present in the current line
     * \exception std::logic_error if \p token is empty
     * \note This function modifies the current line, so any field
     * pointer previously returned by field() is invalidated. It is
     * meant to be used on the raw line, before calling get_fields().
     * \code
     * // line_ == "Elements(223852, 110460, 4, 3)"
     * std::string kw = in.get_bracket_keyword("(");
     * // kw == "Elements" and line_ == "223852, 110460, 4, 3"
     * \endcode
     */
    [[nodiscard]] std::string get_bracket_keyword(const std::string& token);

    /**
     * \brief Gets the current line.
     * \details If get_fields() was called, then an end-of-string
     *  marker '\0' is present at the end of the first field.
     * \return a const pointer to the internal buffer that stores
     *  the current line
     */
    [[nodiscard]] const char* current_line() const {
        return line_.c_str();
    }

    private:
    /**
     * \brief Reads a full line of arbitrary length from the input file.
     * \details Reads characters until the end of the line or the end of the
     * file, growing \p out as needed, so lines longer than any fixed buffer
     * are read completely.
     * \param[out] out the string receiving the line
     * \retval true if a line could be read
     * \retval false if the end of the file was reached without reading
     * any character
     */
    bool read_line(std::string& out) const;

    /**
     * \brief Throws a conversion error.
     * \details This function is called by field_as_int() and
     * field_as_double() when the field \p index cannot be converted to
     * the desired type \p type.
     * \param[in] index index of the erroneous field.
     * \param[in] type the expected type.
     */
    GEO_NORETURN_DECL void conversion_error(
        GEO::index_t index, const char* type
    ) const GEO_NORETURN ;

    FILE* F_;
    std::string file_name_;
    size_t line_num_;
    std::string line_;
    std::vector<char*> field_;
    bool ok_;
    };
}

#endif //GEOLIO_LINE_STREAM_H
