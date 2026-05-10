add_library(bayek_telemetry
  telemetry_packet.c
)

target_include_directories(bayek_telemetry
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(bayek_telemetry PUBLIC bayek_common)
