execute_process(COMMAND "/home/lee/Documents/git/planex-wp6/rosws/build/wp6imufilter/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/lee/Documents/git/planex-wp6/rosws/build/wp6imufilter/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
