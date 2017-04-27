# mbzirc_task2_perception #
Perception system for MBZIRC2017 Challenge 2.

**Requirements**
- Ubuntu 16.04
- CUDA 8.0
- OpenCV 3.1.0
- ZED SDK (> 2.0.0)
- ROS Kinetic

----

## How to run Challenge2 Perception System ##

On Vision PC (pasiphae),

```
$ rosrun mbzirc_task2_perception start-image-processing.sh
```

for checking detailed usage,

```
$ rosrun mbzirc_task2_perception start-image-processing.sh -h
```

----

## Perception nodes usage ##

### BoundingBoxPublisher ###

### ExtractPanelCluster ###

### ImageClipper ###

### InSigmaNormalizer ###

### LuminanceHomogenizer ###

### MeanshiftFiltering ###

### Panel3DProjector ###

### ROIGenerator ###

### Wrench3DProjector ###

### WrenchTemplateDetector ###

