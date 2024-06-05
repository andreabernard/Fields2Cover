add_executable(1_basic_types_tutorial 1_basic_types.cpp)
target_link_libraries(1_basic_types_tutorial 
  PUBLIC 
    ${PROJECT_NAME}::types 
    ${PROJECT_NAME}::utils
)

add_executable(2_objective_functions_tutorial 2_objective_functions.cpp)
target_link_libraries(2_objective_functions_tutorial 
  PUBLIC 
    ${PROJECT_NAME}::objectives
    ${PROJECT_NAME}::path_planning
    ${PROJECT_NAME}::utils
)

add_executable(3_headland_generator_tutorial 3_headland_generator.cpp)
target_link_libraries(3_headland_generator_tutorial 
  PUBLIC 
    ${PROJECT_NAME}::headland_generator
    ${PROJECT_NAME}::utils
)

add_executable(4_swath_generator_tutorial 4_swath_generator.cpp)
target_link_libraries(4_swath_generator_tutorial 
  PUBLIC 
    ${PROJECT_NAME}::objectives
    ${PROJECT_NAME}::headland_generator
    ${PROJECT_NAME}::swath_generator
    ${PROJECT_NAME}::utils
)

if(TARGET Fields2Cover)
  add_executable(5_route_planning_tutorial 5_route_planning.cpp)
  target_link_libraries(5_route_planning_tutorial PUBLIC ${PROJECT_NAME}::Fields2Cover)

  add_executable(6_path_planning_tutorial 6_path_planning.cpp)
  target_link_libraries(6_path_planning_tutorial PUBLIC ${PROJECT_NAME}::Fields2Cover)

  add_executable(7_decomposition_tutorial 7_decomposition.cpp)
  target_link_libraries(7_decomposition_tutorial PUBLIC ${PROJECT_NAME}::Fields2Cover)

  add_executable(8_complete_flow 8_complete_flow.cpp)
  target_compile_definitions(
    8_complete_flow
    PRIVATE
      DATA_PATH="${PROJECT_SOURCE_DIR}/data/"
  )
  target_link_libraries(8_complete_flow PUBLIC ${PROJECT_NAME}::Fields2Cover)

  add_executable(quick_start quick_start.cpp)
  target_compile_definitions(
    quick_start
    PRIVATE
      DATA_PATH="${PROJECT_SOURCE_DIR}/data/"
  )
  target_link_libraries(quick_start PUBLIC ${PROJECT_NAME}::Fields2Cover)
endif()
