# Package: RLB viz
RLB viz is the standard visualization interface provided for the rlb framework. It allows for a very simplistic visualization of an ongoing experiment, and the state of the robots. It was primarily designed for real-world experiments and for monitoring turtlebots running in ONERA's Voliere.

To run:
```
ros2 run rlb_viz rlb_viz
```

Any modification of the **robot_parameters.py** script requires a re-launch of the controllers and visualizer nodes to be reflected
