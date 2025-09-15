cmake_minimum_required(VERSION 3.16)
project(FS_GGVMap_generator CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# === Znajdź zewnętrzne biblioteki ===
find_package(CasADi REQUIRED)
find_package(nlohmann_json 3.0 REQUIRED)
find_package(Threads REQUIRED)

# === GTest (z systemu lub lokalnie) ===
enable_testing()

# Użycie GTest z systemu (jeśli zainstalowane przez apt)
find_package(GTest REQUIRED)
include(GoogleTest)

# === Dodaj bibliotekę interfejsową ===
add_library(ggd_params INTERFACE)
target_include_directories(ggd_params INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(ggd_params INTERFACE nlohmann_json::nlohmann_json casadi)

# === Testy ===
add_executable(test_params test/test_params.cpp)
target_link_libraries(test_params ggd_params GTest::gtest GTest::gtest_main Threads::Threads)

# Skopiuj JSON do katalogu roboczego (build/config/)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/example_config.json
               ${CMAKE_CURRENT_BINARY_DIR}/config/example_config.json
               COPYONLY)

# Dodaj test do CTest
gtest_discover_tests(test_params)
