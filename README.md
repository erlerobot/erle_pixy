erle_pixy
=========

Erle Robot's work with the Pixy camera (CMUcam5).

Documentation of the camera can be found [here](http://www.cmucam.org/projects/cmucam5/wiki).

This code provides images from Pixy. This code is based in the orginal code of [pixymon](https://github.com/charmedlabs/pixy)

# Add permissions for Pixy USB interface

In order to communicate with Pixy over USB as a non-root user you must set permissions for Pixy.

To do this copy the 'pixy.rules' file to '/etc/udev/rules.d'

```
sudo cp pixy.rules /etc/udev/rules.d/
```

#Compile the code 
```
>>mkdir build
>>cd build
>>cmake ..
>>make
```
#Execute
```
>>./pixy_cam
```
