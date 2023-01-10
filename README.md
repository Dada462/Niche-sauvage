# **Niche project** bluerov docking underwater

My awesome project

## Install

Install Qgroundcontrol
https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage

``` bash
bash src/Niche-sauvage/install/install_Qgroundcontrol.sh 

```

Install bluerov dependancies

``` bash
bash src/Niche-sauvage/install/install_mavros.sh 

```

Install USBL driver

``` bash
bash src/Niche-sauvage/install/install_usbl_driver.sh 

```

Install gazebo plugin

``` bash
bash src/Niche-sauvage/install/install_bluerov_gazebo.sh 
```

In `slider_publisher/slider_publisher` line 379 add the rate :

``` python
sp = SliderPublisher(content,50)
```

## Use BlueRov

- Run the bluerob with camera and joystick
Sensors data are saved on a rosbag

Warning : Desarmed and tune off the lights before kill

``` bash
roslaunch bluerov run_joy_bluerov.launch 
```
![](/images/manette_notice.png)

- Run the bluerov with a node command replacing the joystick

``` bash
roslaunch bluerov run_command_bluerov.launch 
```

Save topics :

``` bash
rosbag record --split --duration 5m  --chunksize=1024 --output-prefix=$HOME/catkin_ws/ --all
```

Display rosbag topics :

``` bash
roslaunch bluerov play_topics.launch 
```

## Use USBL

``` bash
rosrun guerledan_usbl USBL_pub
```
## Simulation

Launch gazebo simulation :

``` bash
roslaunch simulation gazebo.launch
```

## Contributors
- **Danut Pop**
- **Hugo Reubrecht**
- **Laurent Potin**
- **Rémi Porée**
- Thomas Le Mézo
- Christophe Viel
- Pierre Narvor

