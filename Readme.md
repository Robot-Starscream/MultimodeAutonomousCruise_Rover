# Multimode Autonomous Cruise of Mars Rover Prototype HITMRII

The multimode decision algorithm is recognized as a crucial component of the multimode autonomous cruise control system deployed on the Zhurong Mars rover prototype HIT-MRII.

**This repo contains**

- __Visual odometry block of the multimode autonomous cruise control system__

    1. Generate height map and obstacle map

    2. Create driving cost map by combining slope-based cost data with the obstacle map

    3. Plan path using the dynamic multi-level path planning algorithm

- __Multimode decision block of the multimode autonomous cruise control system__

    1. Obtain the elevation of the terrain by Elevation Mapping

    2. Mark the obstacles on the terrain by Connected Component Labeling 

    3. Extract geometric features of the obstacle surface by Convolution Operator

    4. Compare the desired motion plane of the wheel with the upper surface of the obstacle by Fitting Plane

    5. Evaluate the safety of the rover’s motion while moving on the obstacle by Grey Relation Analysis and Fuzzy Logic

**Main Contributions**

Our works are mainly in files under "elevation_mapping/elevation_mapping/src" as follows:

- CheckObstacle.cpp

- TraversalCheck.cpp

- ConnectAeras.cpp, ConnectAeras.h

- Connectmap_pub.cpp
  
- GRA.cpp, GRA.h
  
- SecondLayer.cpp, SecondLayer.h

- MotionSafety.cpp, MotionSafety.h
  
- ThirdLayer.cpp

**Dependencies**

- elevation_mapping (https://github.com/ANYbotics/elevation_mapping)

- grid_map (https://github.com/ANYbotics/grid_map)

- kindr (https://github.com/ANYbotics/kindr)

- kindr_ros (https://github.com/ANYbotics/kindr_ros)

**Note**

- This repo does not contain path planning or motion execution. Therefore, it is only for demonstrating algorithm logic and cannot be directly applied to real machines.
