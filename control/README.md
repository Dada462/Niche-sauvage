# **Niche project** bluerov underwater docking

A ros package for docking a bluerov in his underwater doghouse, using an usbl.

## Dependancies

Require Ubuntu 20.04 Ros noetic

## How to run the package

You should run:
```
roslaunch bluerov run_command_bluerov.launch
```
This will run the 'control' package along with the other packages.

In case you just want to run this package use:
```
rosrun control join_cage.py
```

## How to use the package to join a position
In **rqt**, publish on the topic 'desired_position' which is Quaternion type message. **(x,y,z)** represent the desired position to be joined. 

On the other hand, **w** represents a start/stop safety variable. If **w=1**, then the robot will join the position given in **(x,y,z)**. Else, if **w=1**, the robot will stop and idle.

## Contributors
- **Danut Pop**
- **Hugo Reubrecht**
- **Laurent Potin**
- **Rémi Porée** [[on github.com](https://github.com/Remi-Tortue)]

