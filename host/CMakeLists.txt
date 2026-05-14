add_library(bayek_host_sitl
  sitl_conditions.c
  sitl_initial_conditions.c
)

target_include_directories(bayek_host_sitl
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(bayek_host_sitl PUBLIC bayek_common bayek_sim)
