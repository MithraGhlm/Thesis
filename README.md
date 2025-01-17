# Autonomous Mobile Robot (Caspi)

## Directory structure

`Utils` : Scripts, data and tools to build and run the project <br>
`Report` : Final thesis report pdf<br>

The rest of the directories are ROS2 packages.

# Building

Install ROS2 humble, Gazebo 11, and other dependencies by looking at `Dockerfile` included in `Utils` directory.

1. Make a ROS2 workspace
2. Create a `src` directory.
3. Clone the content of this repository into `src`
4. Go back to ROS2 workspace and run `colcon build`
5. Refer to report appendix to run the project.