# JCS Tool

jcs_tool is a GUI tool for interacting with, configuring and testing an Arbite Robotics JCS network.

## Requirements

### JCS dependencies

- Need JCS host installed somewhere. Point the Makefile to jcs_host
- See jcs_public repository for jcs_host requirements.


### Imgur dependencies

- libglfw3
- libglfw3-dev


---
## Operation Notes

### Ubuntu application check alive
Ubuntu 22.04 is a bit too eager when checking for applications to become idle.
In particular, jcs_host will spin when for up to 10 seconds when waiting for a device to erase the firmware during a firmware update procedure.
By default Ubuntu will declare an application crashed if it takes longer than 5 seconds.

Set the check alive time out to something like 20 seconds:
> gsettings set org.gnome.mutter check-alive-timeout 20000
Or disable it completely:
> gsettings set org.gnome.mutter check-alive-timeout 0

