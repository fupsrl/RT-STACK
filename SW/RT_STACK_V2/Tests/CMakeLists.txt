cmake_minimum_required(VERSION 3.20)
project(rt_stack_host_tests C)

enable_testing()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

function(rt_stack_strict_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /WX)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -pedantic)
  endif()
endfunction()

add_executable(trigger_decoder_tests
  trigger_decoder_test.c
  ../Core/Src/trigger_decoder.c
  ../Core/Src/trigger_recorder.c
)
target_include_directories(trigger_decoder_tests PRIVATE ../Core/Inc)
if(NOT MSVC)
  target_link_libraries(trigger_decoder_tests PRIVATE m)
endif()
rt_stack_strict_warnings(trigger_decoder_tests)
add_test(NAME trigger_decoder COMMAND trigger_decoder_tests)

add_executable(actuator_tests
  actuators/test_actuators.c
  ../Core/Src/spark.c
  ../Core/Src/injection.c
)
target_compile_definitions(actuator_tests PRIVATE
  ACTUATOR_HOST_TEST
  INJECTION_CURRENT_CALIBRATION_VALID=1
  ENGINE_INJECTOR_TEST_API_ENABLED=1
)
target_include_directories(actuator_tests PRIVATE
  ../Core/Inc
  ../Core/Src
  actuators
)
if(NOT MSVC)
  target_link_libraries(actuator_tests PRIVATE m)
endif()
rt_stack_strict_warnings(actuator_tests)
add_test(NAME actuators COMMAND actuator_tests)

add_executable(actuator_crank_only_tests
  actuators/test_actuators.c
  ../Core/Src/spark.c
  ../Core/Src/injection.c
)
target_compile_definitions(actuator_crank_only_tests PRIVATE
  ACTUATOR_HOST_TEST
  INJECTION_CURRENT_CALIBRATION_VALID=1
  ENGINE_INJECTOR_TEST_API_ENABLED=1
  IGNITION_REQUIRES_PHASE_SYNC=0
  INJECTION_REQUIRES_PHASE_SYNC=0
)
target_include_directories(actuator_crank_only_tests PRIVATE
  ../Core/Inc
  ../Core/Src
  actuators
)
if(NOT MSVC)
  target_link_libraries(actuator_crank_only_tests PRIVATE m)
endif()
rt_stack_strict_warnings(actuator_crank_only_tests)
add_test(NAME actuators_crank_only COMMAND actuator_crank_only_tests)

add_executable(engine_control_integration_tests
  engine_control/test_engine_control.c
  ../Core/Src/engine_control.c
  ../Core/Src/trigger_decoder.c
  ../Core/Src/trigger_recorder.c
)
target_compile_definitions(engine_control_integration_tests PRIVATE
  ENGINE_INJECTOR_TEST_API_ENABLED=1
  INJECTION_CURRENT_CALIBRATION_VALID=1
  IGNITION_REQUIRES_PHASE_SYNC=0
  INJECTION_REQUIRES_PHASE_SYNC=0
)
target_include_directories(engine_control_integration_tests PRIVATE
  engine_control
  ../Core/Inc
  ../Core/Src
)
if(NOT MSVC)
  target_link_libraries(engine_control_integration_tests PRIVATE m)
endif()
rt_stack_strict_warnings(engine_control_integration_tests)
add_test(NAME engine_control_integration COMMAND engine_control_integration_tests)

add_executable(dcdc_tests
  dcdc/test_dcdc.c
  ../Core/Src/dcdc_control.c
  ../Core/Src/dcdc_pwm.c
)
target_include_directories(dcdc_tests PRIVATE ../Core/Inc)
if(NOT MSVC)
  target_link_libraries(dcdc_tests PRIVATE m)
endif()
rt_stack_strict_warnings(dcdc_tests)
add_test(NAME dcdc COMMAND dcdc_tests)
