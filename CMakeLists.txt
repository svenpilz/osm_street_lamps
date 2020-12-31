cmake_minimum_required(VERSION 3.19)
project(osm_street_lamps)

include(FetchContent)

find_package(Boost REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Protobuf REQUIRED)
find_package(OpenCV REQUIRED)

FetchContent_Declare(
    range_v3
    GIT_REPOSITORY https://github.com/ericniebler/range-v3
    GIT_TAG        0.11.0
)

FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG        v1.8.2
)

FetchContent_MakeAvailable(range_v3 spdlog)

FetchContent_Declare(
    OsmBinary
    GIT_REPOSITORY https://github.com/openstreetmap/OSM-binary
    GIT_TAG        v1.4.0
)

FetchContent_GetProperties(OsmBinary)
if(NOT osmbinary_POPULATED)
  FetchContent_Populate(OsmBinary)

  file(GLOB_RECURSE OSM_BINARY_PROTO_FILES "${osmbinary_SOURCE_DIR}/src/*.proto")
  protobuf_generate_cpp(OSMPBF_SRC_FILES OSMPBF_HEADER_FILES ${OSM_BINARY_PROTO_FILES})
  add_library(osmpbf STATIC ${OSMPBF_SRC_FILES})
  set_property(TARGET osmpbf PROPERTY CXX_STANDARD 20)
  target_link_libraries(osmpbf PUBLIC ${Protobuf_LIBRARIES})
  target_include_directories(osmpbf PUBLIC "${osmbinary_BINARY_DIR}/src")

  file(MAKE_DIRECTORY "${osmbinary_BINARY_DIR}/src/osmpbf")

  foreach(headerFile ${OSMPBF_HEADER_FILES})
      add_custom_command(TARGET osmpbf POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy ${headerFile} "${osmbinary_BINARY_DIR}/src/osmpbf")
  endforeach()
endif()

file(GLOB_RECURSE SRC "*.cpp")
add_executable(osm_street_lamps ${SRC})
target_link_libraries(osm_street_lamps PRIVATE Boost::headers ZLIB::ZLIB spdlog::spdlog range-v3 opencv_imgproc opencv_imgcodecs osmpbf)
set_property(TARGET osm_street_lamps PROPERTY CXX_STANDARD 20)

install(TARGETS osm_street_lamps)
