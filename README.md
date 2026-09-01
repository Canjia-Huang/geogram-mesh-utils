# Geolio

![CI](https://github.com/Canjia-Huang/geolio/actions/workflows/build_and_test.yml/badge.svg)
![External Consumption](https://github.com/Canjia-Huang/geolio/actions/workflows/external_consumption.yml/badge.svg)

**Geolio** is a C++ library designed for performing various processing tasks in computer graphics (mainly mesh processing).

## Requirements / dependencies

- [**Geogram**](https://github.com/BrunoLevy/geogram) — geometry library, required at configure time (set `GEOGRAM_DIR=/path/to/geogram`).
- [**Eigen3**](https://eigen.tuxfamily.org) — header-only linear algebra library, required at configure time via `find_package(Eigen3)`.
- [**spdlog**](https://github.com/gabime/spdlog), [**CLI11**](https://github.com/CLIUtils/CLI11) and [**imoguizmo**](https://github.com/fknfilewalker/imoguizmo) — bundled as git submodules; used as header-only include paths. imoguizmo provides the ImGui/ImGuizmo integration used by the GeoBox application and relies on the imgui headers bundled with Geogram.

> Geolio must be cloned with `--recurse-submodules`, and a project that consumes geolio as a submodule must run `git submodule update --init --recursive`, so that the nested submodules above are present.

## Building the library

```bash
git clone --recurse-submodules https://github.com/Canjia-Huang/geolio.git
cd geolio
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
```

CMake options:

| Option       | Default | Description                          |
|--------------|---------|--------------------------------------|
| `BUILD_TESTS`| `ON`    | Build the test suite (gtest).        |

## Using Geolio as a git submodule

Geolio is designed to be embedded into other projects as a git submodule and consumed through `add_subdirectory`.

### 1. Add the submodule

```bash
git submodule add https://github.com/Canjia-Huang/geolio.git third_party/geolio
git submodule update --init --recursive
```

`--recursive` also pulls geolio's own nested submodules (spdlog, CLI11, imoguizmo).

### 2. Link it from your `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.30)
project(MyApp LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Centralize build outputs so the app and geolio's shared libraries/DLLs land in
# the same tree and are easy to find at runtime.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Embed geolio; exposes the target Geolio::geolio
add_subdirectory(third_party/geolio)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE Geolio::geolio)
```

When consumed as a submodule (`CMAKE_PROJECT_NAME != PROJECT_NAME`), geolio automatically disables `BUILD_TESTS`.

The output directories above put the built `Geolio.dll` next to your executable (in `build/bin/<config>` on Windows), so it is resolved at runtime without extra setup. Without them, on Windows the DLL lands in geolio's nested build subdirectory and must be added to `PATH`; Linux/macOS resolve the shared library automatically through RPATH.

### 3. Make Geogram and Eigen3 visible

The submodule build finds Geogram and Eigen3 through the same mechanisms as the standalone build. Set `GEOGRAM_DIR` (environment variable or `-DGEOGRAM_DIR=...`), and make sure Eigen3 is installed (see [Requirements](#requirements--dependencies)) if it is not already available:

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DGEOGRAM_DIR=/path/to/geogram
```

If the parent project already provides an `Eigen3::Eigen` target (e.g. through its own `find_package(Eigen3)`), geolio detects it and skips its own lookup.

## License

BSD 3-Clause — see [LICENSE](LICENSE).
