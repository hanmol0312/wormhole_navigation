## 1. Introduction

Robot navigation hopping in different rooms with changing room maps simultaneously. 

## 2. Clone the repository.

```
mkdir my_ws
cd my_ws
mkdir src
cd src
git clone https://github.com/hanmol0312/wormhole_navigation.git

```

## 3. Build the project.

```
cd ..
catkin_make
source devel/setup.bash
```

## 4. Launch Simulation And Navigation

```python
roslaunch start_anscer start_anscer.launch
# In new terminal
roslaunch anscer_navigation anscer_navigation.launch

```

## 5. Run Navigation Action Server

```python
rosrun multi_map_nav navigation_server 
```

- Send goal via terminal

```python
rostopic pub /navigation_action/goal multi_map_nav/NavigationActionGoal "{ \
  header: { seq: 0, stamp: { secs: 0, nsecs: 0 }, frame_id: \"\" }, \
  goal_id: { stamp: { secs: 0, nsecs: 0 }, id: \"\" }, \
  goal: { \
    target_map_name: \"room_1\", \
    target_pose: { \
      header: { seq: 0, stamp: { secs: 0, nsecs: 0 }, frame_id: \"map\" }, \
      pose: { \
        position: { x: 1.0, y: -6.0, z: 0.0 }, \
        orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } \
      } \
    } \
  } \
}"

```

## 5. Path generated

![wormhole_image.png](attachment:bd159cc2-0a97-4e4c-9365-0795ec84a919:wormhole_image.png)

## 6. Approach

- Reading the data from robot_pose_publisher at wormhole and updating it in the sql database.
- Action server reads the room_name and decides subsequently if there is need to change the map if yes move to the wormhole location switch map reinitialize pose with respect to new map for better navigation in new room.