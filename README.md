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

## Use BlueRov

Run the bluerob with camera and joystick

Warning : Desarmed and tune off the lights before kill

``` bash
roslaunch bluerov run_bluerov.launch 
```
![](/images/manette_notice.png)

## Contributors
- **Danut Pop**
- **Hugo Reubrecht**
- **Laurent Potin**
- **Rémi Porée**
- Thomas Le Mézo
- Christophe Viel
- Pierre Narvor

