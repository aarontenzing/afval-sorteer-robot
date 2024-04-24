if [ "$1" = "-b" ]; then
	colcon build --packages-select afvalrobot
fi
. install/setup.bash
ros2 launch afvalrobot launch.py

