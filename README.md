# AutoServe

AutoServe is a modular robotics simulation framework built on ROS 2 and Gazebo. It integrates custom sensors, controllers, and simulation environments to enable advanced robotic applications such as navigation, mapping, and perception.

## Features

- **Custom Sensors**: Includes a dummy sensor system for simulation.
- **Simulation Environment**: Predefined Gazebo worlds and robot models.
- **Modular Design**: Separate packages for navigation, mapping, perception, and more.
- **ROS 2 Integration**: Leverages ROS 2 for communication and control.
- **CI Pipelines**: Automated builds and tests using GitHub Actions and CircleCI.
- **Dev Containers**: Development environment using Docker and Visual Studio Code.

## Dependencies

1. Gazebo Harmonic:
    ```bash
    sudo apt-get update
    sudo apt-get install curl lsb-release gnupg
    ```
    ```bash
    sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt-get update
    sudo apt-get install gz-harmonic
    ```

2. Nav2:
    ```bash
    sudo apt-get install ros-jazzy-nav2-bringup ros-jazzy-navigation2
    ```

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/JatinPatil2003/Pal_Gsoc_RFID.git
   cd AutoServe
   ```

2. Build the workspace:
   ```bash
   colcon build --symlink-install
   ```

3. Source the workspace and export dummy sensor system plugin path:
   ```bash
   source install/setup.bash
   export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)/build/DummySensorSystem
   ```

## Usage

### Launching the Simulation

1. Start Gazebo with the robot:
   ```bash
   ros2 launch autoserve_gazebo gazebo.launch.py
   ```

2. Autonomous Navigation:
   ```bash
   ros2 launch autoserve_navigation navigation.launch.py
   ```

### Development with Dev Containers

1. Open the repository in Visual Studio Code.
2. Install the "Remote - Containers" extension.
3. Reopen the folder in the Dev Container:
   - Press `F1` and select `Remote-Containers: Reopen in Container`.

### CI Pipelines

- **GitHub Actions**: Automatically builds and tests the project on every push or pull request. See the [GitHub Actions workflow](.github/workflows/AutoServe_CI.yml).
- **CircleCI**: Docker-based CI pipeline for building and publishing Docker images. See the [CircleCI configuration](.circleci/config.yml).

## License

This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please follow the standard GitHub workflow for submitting issues and pull requests.

## Acknowledgments

This project leverages the following technologies:
- [ROS 2](https://docs.ros.org/en/rolling/index.html)
- [Gazebo](https://gazebosim.org/)
- [gz-sensors](https://github.com/gazebosim/gz-sensors)
