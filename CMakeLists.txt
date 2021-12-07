cmake_minimum_required(VERSION 3.5)

project(dreamvu_pal_camera_description)

find_package(ament_cmake REQUIRED)


#find_package(launch_ros)
find_package(rviz2)

foreach(dir launch rviz2 urdf)
	install(DIRECTORY ${dir}/
		DESTINATION share/${PROJECT_NAME}/${dir})
endforeach(dir)

ament_package()
