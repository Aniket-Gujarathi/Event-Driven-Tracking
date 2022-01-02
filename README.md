# Event-driven Gaze Tracking with a Particle Filter

The algorithm performs tracking of eye-gaze target in-the-wild given the event-stream from an event-camera. 

## CPU

A C++ YARP module that runs the algorithm on a standard CPU.

#### How to build
1. install [YARP](https://github.com/robotology/yarp)
1. install [event-driven](https://github.com/robotology/event-driven)
1. make sure both YARP and event-driven can be found on the environment path / project path
1. clone this repository (e.g. into `~/projects/particle-filter-tracking`)
1. cd `~/projects/particle-filter-tracking` && mkdir build && cd build
1. cmake ..
1. make
1. ensure the bin is found in the environment path

#### How to run

1. yarpserver --write
1. yarpmanager
1. open the provided yarpmanager application (`particle-filter-tracking/cpu/gaze_tracking.xml`)
1. run all
1. open a dataset with the yarpdataplayer
1. connect all
1. play the dataset on the yarpdataplayer

Note: you can substitute the yarpdataplayer with a live feed of an event-camera if it is yarp-compatible. Please contact the authors for more information if you want to make your camera yarp compatible.

#### Implementation
1. The --src folder contains two implementations for the circle-target based eye-ball tracking and the parabola-based eyebrow tracking (for efficient search space).
2. Compared to related methods, the tracking algorithm does not assume a fixed camera or head of a person.

#### Results
##### Parabola Tracking


https://user-images.githubusercontent.com/36470661/147889049-973ebfd6-1877-4875-bb86-19ca8c49a83e.mp4

##### Circle Tracking
https://user-images.githubusercontent.com/36470661/147889224-f31299fb-440a-4bd2-8636-33ce70c46988.mp4

