# Multimode Autonomous Cruise of Mars Rover Prototype HITMRII

This project is a crucial component of the multimode autonomous Cruise system deployed on the Mars rover prototype HITMRII, and it also serves as supplementary materials for the research paper "Zhurong Mars Rover: Recorded Motion Analysis and Future Multimode Autonomous Cruise Enhancement".

**This repo contains**

- __Multimode decision block of the multimode-based control system__

    1. Obtain the elevation of the terrain by Elevation Mapping

    2. Mark the obstacles on the terrain by Connected Component Labeling 

    3. Extract geometric features of the obstacle surface by Convolution Operator

    4. Compare the desired motion plane of the wheel with the upper surface of the obstacle by Fitting Plane

    5. Evaluate the safety of the rover’s motion while moving on the obstacle by Grey Relation Analysis and Fuzzy Logic

**Main Contributions**

Our works are mainly in files under "elevation_mapping/elevation_mapping/src" as follows:

- CheckObstacle.cpp

- TraversalCheck.cpp

- ConnectAeras.cpp, ConnectAeras.cpp.h

- Connectmap_pub.cpp
  
- GRA.cpp, .h
  
- SecondLayer.cpp, SecondLayer.cpp.h

- MotionSafety.cpp, MotionSafety.h
  
- ThirdLayer.cpp

**Dependencies**

- elevation_mapping (https://github.com/ANYbotics/elevation_mapping)

- grid_map (https://github.com/ANYbotics/grid_map)

- kindr (https://github.com/ANYbotics/kindr)

- kindr_ros (https://github.com/ANYbotics/kindr_ros)

**Note**

- This repo does not contain path planning or motion execution. Therefore, it is only for demonstrating algorithm logic and cannot be directly applied to real machines.
