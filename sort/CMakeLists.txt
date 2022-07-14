add_library(sort 
  hungarian.cc
  kalman_filter.cc 
  kalman_box_tracker.cc
  linear_assignment.cc)
target_link_libraries(sort glog::glog Eigen3::Eigen)

add_executable (main main.cc)
target_link_libraries (main sort)

add_executable (hungarian_test hungarian_test.cc)
target_link_libraries(hungarian_test sort)

# google test
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(
  linear_assignment_test
  linear_assignment_test.cc
)
target_link_libraries(
  linear_assignment_test
  GTest::gtest_main sort
)

include(GoogleTest)
gtest_discover_tests(linear_assignment_test)