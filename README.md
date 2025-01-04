# ROS2 AutoPilot

## Autonomous Robot Navigation Using Machine Learning

### **Context**
This project demonstrates the use of machine learning, I have used a **Random Forest model**, to enable autonomous robot navigation in diverse and challenging environments. Built on top of **ROS 2** (Humble) and leveraging the **Gazebo simulation environment**, the robot uses **360-degree LiDAR scans** to navigate through predefined worlds such as mazes, circular rooms, and square rooms. The system achieves impressive accuracy in predicting motion commands (`cmd_vel`) such as moving forward, turning, or stopping, based on LiDAR input.

---

### **Core Functionality**
- **Training a Machine Learning Model**: Using recorded LiDAR scan data and corresponding motion commands to train a Random Forest model.
- **Testing the Model**: Evaluating the trained Random Forest model's accuracy, precision, recall, and F1 score on test datasets.
- **Deploying the Model**: Real-time prediction and robot navigation in simulation environments using ROS 2.

---

### **Prerequisites**
- **ROS 2 (Humble)**:
  Install ROS 2 Humble following the official [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html).
- **Gazebo**:
  Install Gazebo packages:
  ```bash
  sudo apt install ros-humble-gazebo-ros-pkgs
  ```
- Python Dependencies: Install required Python libraries
  ```bash
  pip install numpy scikit-learn stable-baselines3 gymnasium xacro
  ```

---

# Commands to Run the Program

## 1. Install Requirements
Install the necessary Python dependencies for the project:
```bash
pip install numpy scikit-learn stable-baselines3 gymnasium xacro
```

## 2. Clone My GitHub Repo into Your Workspace
Clone the repository into your ROS 2 workspace:
```bash
git clone https://github.com/rituraj-navindgikar/ros2_auto_pilot_ml.git
```

## 3. Build and Source the Workspace
Build the ROS 2 workspace and source it:
```bash
colcon build --symlink-install
source install/setup.bash
```

## 4. Launch the Robot in Gazebo
Launch the robot in a predefined Gazebo world. Specify the `.world` file of your choice. You can also modify the spawn location depending on your world configuration:
```bash
ros2 launch robot-machine-learning robot_spawn_launch.py world:=<path to your .world file>
```

## 5. Data Recording
Start recording LiDAR data and motion commands:

```bash
ros2 run ros2_auto_pilot_ml data_recorder
```
Use teleoperations to manually move the robot for training:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
The recorded data will be saved as a CSV file in the training_data/ directory.


## 6. Training the Machine Learning Model
Train the Random Forest model (or experiment with another machine learning algorithm) using the recorded dataset. 

Example command for training with a Random Forest model:
```bash
cd tests/random_forest/
```
- open the [main.ipynb](https://github.com/rituraj-navindgikar/ros2_auto_pilot_ml/blob/main/tests/random_forest/main.ipynb) file
- Replace square_dataset.csv with the path to your dataset
- The trained model will be saved as model_square.pkl


## 7. Testing the Random Forest Model

### Launch the Robot Again in Gazebo
Run the robot in the same or a different Gazebo world for testing (you don't need to manually drive the robot this time):
```bash
ros2 launch robot-machine-learning robot_spawn_launch.py world:=<path to your .world file>
```
### Predict and Test Real-Time Navigation
After training, import the saved model into the [predict_cmd_vel.py](https://github.com/rituraj-navindgikar/ros2_auto_pilot_ml/blob/main/tests/random_forest/predict_cmd_vel.py) script and execute it to navigate the robot:
```bash
python3 predict_cmd_vel.py
```
This script will use the trained model to predict motion commands (cmd_vel) based on LiDAR data and navigate the robot autonomously.

---

### Notes
If a robot or world is not loading, double-check the paths in the [robot_spawn_launch.py](https://github.com/rituraj-navindgikar/ros2_auto_pilot_ml/blob/main/src/robot-machine-learning/launch/robot_spawn_launch.py) file

---

Creator & Developer
Rituraj Navindgikar
