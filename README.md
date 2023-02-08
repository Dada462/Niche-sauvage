# **Niche project** bluerov docking underwater

A ros package for docking a bluerov in him underwater doghouse, using usbl, lights detection and aruco qrcodes.

## Dependancies

Require Ubuntu 20.04 Ros noetic

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

Install QRcode driver

``` bash
bash src/Niche-sauvage/install/install_qr_code.sh 

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

- Run the bluerov with a node command replacing the joystick (X for switching to auto control)

Control command had to be published on `commande`

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
rosrun usbl usbl_pub
```

Display USBL command path:

```bash
rosrun mission_displayer MissionDisplayer.py
```

## Use lights

``` bash
rosrun detect_lum node_lum.py
```

## Use QR codes

``` bash
roslaunch commande_rov_aruco simu.launch
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
- **Rémi Porée** [[on github.com](https://github.com/Remi-Tortue)]
- Thomas Le Mézo
- Christophe Viel
- Pierre Narvor

