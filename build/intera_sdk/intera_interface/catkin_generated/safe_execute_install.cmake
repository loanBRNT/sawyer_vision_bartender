execute_process(COMMAND "/home/loan/test_script/sawyer_vision_bartender/build/intera_sdk/intera_interface/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/loan/test_script/sawyer_vision_bartender/build/intera_sdk/intera_interface/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
