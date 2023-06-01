execute_process(COMMAND "/home/loan/sawyer_vision_bartender/build/intera_sdk/intera_examples/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/loan/sawyer_vision_bartender/build/intera_sdk/intera_examples/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
