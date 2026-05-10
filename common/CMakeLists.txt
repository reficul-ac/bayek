add_library(bayek_common
  math_utils.c
  control_utils.c
)

target_include_directories(bayek_common
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(bayek_common PUBLIC m)
