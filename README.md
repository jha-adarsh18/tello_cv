---

# tello_cv

This repository provides ROS2 nodes to control a Tello drone, including object detection using a YOLO model and various movement commands.

## Getting Started

### Prerequisites

- ROS2 Foxy
- Follow the instructions to set up `tello_ros`:
  - Clone the `tello_ros` repository:
    ```bash
    git clone https://github.com/clydemcqueen/tello_ros.git
    ```
  - Follow the steps mentioned in the `tello_ros` repository to set up the environment up to the `teleop` section.

### Installation

1. Open a new terminal.
2. Navigate to the `src` folder of your ROS2 workspace:
    ```bash
    cd ~/your_ros2_workspace/src
    ```
3. Clone this repository:
    ```bash
    git clone https://github.com/jha-adarsh18/tello_cv.git
    ```
4. Source the ROS2 workspace setup:
    ```bash
    source ~/your_ros2_workspace/install/setup.bash
    ```
5. Source the ROS2 Foxy setup:
    ```bash
    source /opt/ros/foxy/setup.bash
    ```

## Running the Nodes

To run the various nodes, use the following commands:

- To take off the drone:
  ```bash
  ros2 run tello_cv tellotakeoff
  ```

- To land the drone:
  ```bash
  ros2 run tello_cv telloland
  ```

- To identify objects present in the COCO dataset:
  ```bash
  ros2 run tello_cv telloobject
  ```

- To yaw at 45 degrees constantly:
  ```bash
  ros2 run tello_cv tellocmd
  ```

- To yaw at the provided `yaw_rate_degrees` once and hold:
  ```bash
  ros2 run tello_cv tonce --ros-args -p yaw_rate_degrees:=45.0
  ```

- To move upwards by the provided height:
  ```bash
  ros2 run tello_cv climb --ros-args -p climb_height_cm:=50.0
  ```

- To move down by the provided height:
  ```bash
  ros2 run tello_cv fall --ros-args -p move_distance_cm:=50.0
  ```

- To run the m5stack RoverC-pro detection model:
  ```bash
  ros2 run tello_cv --ros-args -p model_path:='path/to/your/model'
  ```

Note: The `--ros-args` flag is used to pass parameters to the nodes. If you do not provide these parameters, default values will be used.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

- This repository is created using [tello_ros](https://github.com/clydemcqueen/tello_ros) which is licensed under the BSD-3-Clause License.


---
