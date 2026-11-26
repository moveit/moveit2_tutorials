if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(tests_tutorial test/tests.cpp)
  target_link_libraries(tests_tutorial
  ${Boost_LIBRARIES})
endif()
