# Robotic autonomous vehicle

Autonomus robot meant to collect blue boxes around a map. The robot is a lego robot with varius sensors.

## Features

### General 

- [ ]  Odometry
- [ ]  Kalman filter
- [ ]  Path-planning
- [ ]  Perception
- [ ]  Motion control
- [ ]  Localization


### Part 1

- [x]  Coloured box detection 
- [x]  Detect numbers on boxes
- [x]  Distance to box
- [ ]  Robot construction

### Part 2

- [ ]  Cox line fit

## Parameters

| Parameter      | Type     | Description                       | Default                 |
| :--------      | :------- | :-------------------------------- | :---------------------- |
| `SCALE`    | `int`    | **Optional**. Optimization parameter |  1  |

| Parameter      | Type     | Description                       | Default                 |
| :--------      | :------- | :-------------------------------- | :---------------------- |
| `HSV_LOW_BOUND`| `Scalar`    | **Required**. HSV lower bound value for contour |  (99,130,177)  |

| Parameter      | Type     | Description                       | Default                 |
| :--------      | :------- | :-------------------------------- | :---------------------- |
| `HSV_HIGH_BOUND`| `Scalar`    | **Required**. HSV upper bound value for contour |  (117,255,255)  |


## Optimizations

Added scale hyperparameter that scales down the input image before running the contour function. Represents the integer that divides the original size of the image to create a smaller image

| Parameter | Type     | Description                       |
| :-------- | :------- | :-------------------------------- |
| `SCALE`   | `int`    | **Optional**. Optimization parameter |

