## On mac: start docker:
colima start

## Build the docker image:
docker build -t my-ros-rviz .

## Clean build:
docker build --no-cache -t my-ros-rviz .

# Run the docker container:
./run_ros_container.sh
