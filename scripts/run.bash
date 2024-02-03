cd ../../ && \
colcon build --packages-select jif&& \
source install/local_setup.bash && \
ros2 run jif jif-core
