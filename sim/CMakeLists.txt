add_library(bayek_sim
  sim_plant.c
)

target_include_directories(bayek_sim
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(bayek_sim PUBLIC bayek_common)
