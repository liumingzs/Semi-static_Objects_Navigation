# Semi-Static Object Navigation

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
