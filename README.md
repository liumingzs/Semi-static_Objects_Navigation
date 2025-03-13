# Semi-Static Object Navigation
![Alt text](/picture/system_view.png)
<div align="center">
  Fig1: System Overview
</div>

### ▶ Demo Video
[Bilibili Video](https://www.bilibili.com/video/BV1uk91YnEiV/)


## 1. Project Overview  
This project aims to design an intelligent navigation system for semi-static objects using semantic-guided open-space relational reasoning. By creating an expandable semantic topological map, the system enables robots to understand scene information and object relationships, facilitating regional inference for predicting the locations of semi-static objects. 

## 2. Core Features  
- **an extensible topological map that integrates multi-source information:** 
- **a novel interactionbased semantic-guided strategy for the navigation of semistatic objects.:** 

## 3. Dependencies  
This project relies on the following frameworks and models:  
- [MobileSAM](https://github.com/ChaoningZhang/MobileSAM)  
- [CLIP](https://github.com/openai/CLIP)
- [ROS 2](https://docs.ros.org/en/galactic/index.html) 

**Ensure that the necessary software and dependencies are installed before running the project.**  

## 4. Installation & Setup  

### 1. Install MobileSAM and CLIP  
```bash
pip install git+https://github.com/ChaoningZhang/MobileSAM.git
pip install git+https://github.com/openai/CLIP.git
```
### 2. Compilation  
```bash
git clone https://github.com/your-repo/semi-static-object-navigation.git
cd semi-static-object-navigation
mkdir src
mv py_graph ram_detect src/
cd ..
colcon build
source install/setup.bash
```

## 5.Create topology map
![Alt text](/picture/Overlook.png)
<div align="center">
  Fig3: topology map overview
</div>

### 1. loading model 
```bash
python3 sam_service.py
```
### 2. Start the Gazebo scene graph
```bash
ros2 launch gazebo_ros gazebo.launch.py world:=small_house.world
```
### 3. Segment and detect objects
```bash
ros2 launch ram_detect my_launch_file.launch.py 
```
### 4. robot control
```bash
 ros2 run velocity_publisher velocity_publisher 
```
### 5. visualization results
![Alt text](/picture/121.png)
<div align="center">
  Fig2: The blue edges represent nodes connected to objects, and each group of blue edges points to the objects detected in the scene.
</div>

## 6.Semantic reasoning for the position of semi-static objects
### 1.Get navigation path
```bash
ros2 run py_graph route_plan
```
### 2.Activate model detecting
```bash
python3 sam_service_second.py
```
### 3.Save key frames and segmented objects
```bash
ros2 launch ram_detect second_guide.launch.py
```
### 4.Robot patrol strategy
```bash
ros2 run ram_detect virtual_point_to_tracking
```
### 5.Large model reasoning semi-static objects
```bash
python3 guide_model_gpt.py 
```
