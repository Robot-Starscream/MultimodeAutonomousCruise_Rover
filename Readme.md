# Supplementary materials for "Zhurong Mars Rover: Recorded Motion Analysis and Future Multimode Autonomous Cruise Enhancement"

**This repo contains**

- __Multimode decision block of the multimode-based control system__

    - Elevation Mapping, to obtain the elevation of the terrain

    - Connected Component Labeling, to mark the obstacles on the terrain

    - Convolution Operator, to extract geometric features of the obstacle surface

    - Fitting Plane, to compare the desired motion plane of the wheel with the upper surface of the obstacle

    - Grey Relation Analysis and Fuzzy Logic, to evaluate the safety of the rover’s motion while moving on the obstacle

**Main Contributions**

Our works are mainly in files under "elevation_mapping/elevation_mapping/src" as follows:

- CheckObstacle.cpp

- TraversalCheck.cpp

- ConnectAeras.cpp, .h

- Connectmap_pub.cpp
  
- GRA.cpp, .h
  
- SecondLayer.cpp, .h

- MotionSafety.cpp, .h
  
- ThirdLayer.cpp

**Dependencies**

- elevation_mapping (https://github.com/ANYbotics/elevation_mapping)

- grid_map (https://github.com/ANYbotics/grid_map)

- kindr (https://github.com/ANYbotics/kindr)

- kindr_ros (https://github.com/ANYbotics/kindr_ros)

**Note**

- This repo does not contain path planning or motion execution. Therefore, it is only for demonstrating algorithm logic and cannot be directly applied to real machines.
