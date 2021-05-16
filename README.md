# Semantic Map for Indoor Positioning System

## Summary
This project builds upon the [Semantic SLAM](https://github.com/floatlazer/semantic_slam) repository by Xuan Zhang.
It implements ROS nodes to match the semantic octomap frame to the floorplan frame, which can be found in the ```map_localiser``` package.
The system will attempt to match coordinates in a supplied bitmap floorplan to coordinates in a semantic cloud. 
Addtional nodes for testing visualisation are also included.
Nodes to calculate the transform and visualise the semantic map are not implemented.

## Dependencies
See [Semantic SLAM](https://github.com/floatlazer/semantic_slam) for all Semantic SLAM dependencies.
Apart from the ZED ROS package, this project does not depend on any additional dependencies.

This project was built using ```catkin_make``` with ROS Melodic on Ubuntu 18.04.5 LTS.

## Installation
See [Semantic SLAM](https://github.com/floatlazer/semantic_slam) to build Semantic SLAM and its dependencies.

## Trained Models
See [Semantic SLAM](https://github.com/floatlazer/semantic_slam) for models available.

## Getting Started
1. Prepare floorplan (See [Floorplan](#floorplan))
2. Edit ```floorplan_file_path``` in [matcher.yaml](map_localiser/params/matcher.yaml) to point to bitmap file
3. Launch Semantic SLAM nodes
    ```
    roslaunch semantic_slam sem_slam.launch
    ```
4. Launch landmark extractor
    ```
    //without visualiser
    roslaunch map_localiser extractor.launch
    
    //with visulaiser
    roslaunch map_localiser extractor.launch visualiser:=true
    ```
5. Launch coordinate matcher
    ```
    //without visualiser
    roslaunch map_localiser matcher.launch
    
    //with visulaiser
    roslaunch map_localiser matcher.launch visualiser:=true
    ```
6. (Optional) Run the plotter node
    ```
    roslaunch map_localiser plotter.launch
    ```

## Project Structure

### Functional Nodes
These nodes implement the core functionality of the semantic map.
- ```LandmarkExtractorNode.cpp```
- ```CoordinatesMatcherNode.cpp```

```LandmarkExtractorNode.cpp``` subscribes to semantic octomap messages in ```/octomap_full``` to extract a pattern of landmarks.
Extracted patterns are published to ```/landmarks```.
```CoordinatesMatcherNode.cpp``` subscibes to landmark pattern messages in ```/landmarks``` and reads in a bitmap floorplan to find matching coordinates in in the floorplan for the pattern.
The possible matches are published to ```/match_results```.

### Development Nodes
These nodes aid in the development and testing by visualing the outputs of the functional nodes, as well as providing simulated inputs.
- ```LandmarkExtractorVisualiser.cpp```
- ```CoordinatesMatcherVisualiser.cpp```
- ```plotter.py```
- ```landmarkpublisher.py```

## Camera Setup
This project uses the [ZED Camera by StereoLabs](https://www.stereolabs.com/zed/) instead of the original ASUS Xtion.
Since Semantic SLAM is monocular, the left image feed for the ZED Camera was used. Relevant parameters files (```semantic_slam``` package) used in this project are:
- ```octomap_generator.yaml```
- ```semantic_cloud_zed.yaml```
- ```zed.yaml```

Ensure that resolutions and aspect ratios of the image feed used is consistent in both ```semantic_cloud.yaml``` and ```zed.yaml```.

Modifications were also done to the ZED ROS wrapper parameters (```zed-ros-wrapper``` package) to achieve downsampling of the image feed.
```semantic_cloud_zed.yaml``` and ```zed.yaml``` is adjusted to account for the downsampled resolution and aspect ratio.


## Floorplan
The coordinate matcher node ```CoordinatesMatcherNode.cpp``` reads in the floorplan as a bitmap.
It identifies different classes of landmarks by reading their RGB values in the floorplan.
By default, each pixel, when appropriately coloured, is identified a unique landmark.
The coordinates of that landmark are its pixel coordinates in the bitmap.

For reference, the RGB values of each class can be found in this [file]().

For larger sized floorplans, ```CoordinatesMatcherNode.cpp``` may also aggregate adjacent pixels of the same class to identify them as a single landmark by taking their "centre of mass".
To do this, change value of ```floorplan_landmark_aggregation``` in [matcher.yaml](map_localiser/params/matcher.yaml) from ```false``` to ```true```.


## Simulating Landmarks
The coordinates matcher node ```CoordinatesMatcherNode.cpp``` can be run independent of other ROS nodes in the system for testing purposes.
The nodes launched are:
- ```CoordinatesMatcherNode.cpp```
- ```CoordinatesMatcherVisualiser.cpp```
- ```plotter.py```
- ```landmarkpublisher.py```

The constellation of nodes will read in a bitmap floorplan and simulate the extraction process.
The extracted landmarks are visualised in yellow, and the matched landmarks in cyan.
A scatter plot of inter-landmark distances in the extracted pattern against inter-landmark distances in the matched pattern will also be displayed.

1. Prepare floorplan to simulate (See [Floorplan](#floorplan))
2. Edit ```floorplan_file_path``` in [matcher.yaml](map_localiser/params/matcher.yaml) to point to bitmap file
3. Run simulation
    ```
    roslaunch map_localiser matcher_test.launch
    ```
   
## Configuration

### Parameters for octomap_generator node and semantic_cloud node
See [Semantic SLAM](https://github.com/floatlazer/semantic_slam) for details

### Parameters for Landmark Extractor Node
[extractor.yaml](map_localiser/params/extractor.yaml)

namespace ```extractor```

- ```camera_frame_id```: frame id of camera
- ```octomap_topic```: topic of published semantic octomaps
- ```octomap_frame_id```: frame id of octomap
- ```landmarks_topic```: topic of published landmark patterns
- ```buffer_size```: maximum number of landmarks for each pattern
- ```search_radius```: raycast distance in cells (arbitrary) to search for landmarks
- ```strategy```
    - ```0``` (NEAREST): nearest landmarks to camera
    - ```1``` (UNIQUE): nearest landmarks to camera, but all landmarks in pattern are of a unique class (default and recommended)

namespace ```extractor/visualiser```
- ```point_scale```: scale of red box markers in visualiser
- ```text_scale```: scale of text markers in visualiser
- ```namespace```: namespace for markers published by the visualiser
- ```marker_array_topic```: topic where markers are published to by the visualiser

### Parameters for Coordinate Matcher Node
[matcher.yaml](map_localiser/params/matcher.yaml)

namespace ```matcher.yaml```

- ```top```: maximum number of matches to return
- ```strategy```: matching strategy (unimplemented)
- ```min_correlation_threshold```: minimum correlation of chain required, below which will be discarded
- ```min_chain_length```: mimimum length of chain to begin matching. Cannot be less than 3.
- ```max_buffer_size```: maximum no. of chains to compute per pass
- ```landmarks_topic```: topic of published landmark patterns
- ```match_result_topic```: topic of published matched results
- ```floorplan_file_path```: absolute path to floorplan bitmap
- ```floorplan_landmark_aggregation```: aggregate contiguous pixels to find CoM to use as landmark coordinate

namespace ```matcher/visualiser```
- ```image_topic```: topic of marked floorplan image feed (matches of highest correlation chain are visualised on the floorplan bitmap as cyan pixels) 

## Notes
Below are some common issues that might be encountered when setting up the repository
### Python
- ```setup.py``` for ```ptssemseg``` may not install properly, look under its requirements.text to manually install missing dependencies
- pytorch may generate "module not found" errors due to compatibility issues, manually install torch 0.4.0
###CUDA / Nvidia (for Ubuntu)
- In case nvidia drivers are not compatible, a manual reinstall is required
    - Purge all nvidia drivers: ```sudo apt-get remove --purge '^nvidia'```
    - Check for recommended driver installs: ```ubuntu-drivers devices```
    - Install recommended drivers: ```sudo ubuntu-drivers autoinstall```
    - check driver installation: ```nvidia-smi```
    - install nvcc: ```sudo install nvcc```