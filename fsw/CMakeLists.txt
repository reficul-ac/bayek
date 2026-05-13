add_library(bayek_fsw
  control.c
  fault.c
  fsw.c
  guidance.c
  nav.c
)

target_include_directories(bayek_fsw
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(bayek_fsw
  PUBLIC
    bayek_common
)
