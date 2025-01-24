# mc_sg_torque_test

A single joint controller, motor controller and strain gauge interface (with force sensor) for performing motor static torque tests.

## Overview

```
    INFO   : ==============================================================================================================
    INFO   : Device/process tree:
    INFO   : 
    INFO   : `- HOST                                            id: 0          address: 0          n_ports: 1 n_procs: 0
    INFO   :    `- jc_0                                         id: 8          address: 33554432   n_ports: 2 n_procs: 0
    INFO   :       |- mc_0                                      id: 16         address: 33558528   n_ports: 0 n_procs: 0
    INFO   :       `- sg_0                                      id: 24         address: 33624064   n_ports: 0 n_procs: 0
    INFO   : 
    INFO   : ==============================================================================================================
```

In this example the motor controller (mc_0) connects to port 0 of the joint controller (jc_0) and a strain gauge interface (sg_0) connects to port 1 of the joint controller,
The joint controller connects back to the host computer (HOST).

The motor controller is configured to accept a torque producing current on it's Q-axis via host signal `host_mc_q_i`.

The motor controller will return it's actual torque producing Q-axis current `i_q` and the strain gauge interface returns scaled channel 0 data `sg_an_0`. The strain gauge data represents the force applied by a moment arm.

Obtaining the motor torque constant Kt, or the relationship between motor torque and current relies on some test rig that will convert the motors output to a linear force to be sensed by the force sensor.
It is then a matter of applying currents, recording the actual current and force, then converting the force into a torque.


## Setup

### Strain gauge interface
The strain gauge interface must be configured for the strain gauge used.
For this demo an Aliexpress 10kg load cell was used:
https://www.aliexpress.com/item/4000194932572.html

The load cell was connected to channel 0 of the strain gauge interface, and to the 2.5V reference.
The interface ADC channels are configured for a 128x gain and for differential measurement:

```
  adc_channel_0_gain: ads131m06_gain_128
  adc_channel_0_mux:  ads131m06_mux_ainp_ainn
```

To scale the measurements, the ADC values are passed through the `transform_type: linear_gain_offset` transform:
Following the steps in the configuration file `dev_sg_0.yaml`, the scale and offset for channel 0 is obtained by suspending a proof mass from the strain gauge and calculating out the values. The final strain gauge ADC measurement is force in Newtons.


### Test rig
Build a test rig and install all the components.
See `test_rig/test_rig_0.jpg` and `test_rig/test_rig_1.jpg` for some inspiration.
In this test rig, the torque arm length is kept short to increase the force seen by the load cell, for given small torques of the motor.
A point contact at the end of the torque arm compresses the load cell.

Configure the motor controller by following the steps in the `mc_torque_control` example, but omit the Kt parameter.
Note, the torque arm should be removed during motor commissioning to allow the motor to spin freely.

Once satisfactory current control performance is obtained, copy the motor controller configuration yaml into this directory.

To automatically start the motor current controller, uncomment `list_start` (and its items) in `HOST.yaml`.

Start jcs_tool with this configuration:

```
sudo ./jcs_tool -p ../examples_configure/mc_sg_torque_test/
```

Apply a ramp current to the motor controller Q-axis to generate motor torque:
- Start the system. The motor controller will start
- Under HOST, click Host Oscilloscope
- Set a sample time of 2 seconds
- Set Channel 0 source to mc_0::i_q
- Set Channel 1 source to sg_0::sg_an_0
- Click Input Signal Stimulus
- Select Ramp stimulus source
- Set Ramp end value to 5A (or what maximum current you would like)
- Set Ramp time to 1 second
- Set Stimulus signal input to HOST::host_mc_q_i
- Click start stimulus. The system will ramp a current command to the motor controller and record the actual current and strain gauge force.

When the stimulus ends the graphs will be updated. Double click them to expand to the whole data.
Write the data to a file if required.

Experiment with step stimulus for many second durations to examine the torque production performance as the motor heats up.


### Operation notes

- Manually rotate the torque arm until it contacts the load cell before applying step currents. The torque arm will bang into the load cell otherwise.
- When operating at higher currents and depending on the motor controller and the DC bus voltage used, you may hit the modulation limit of the controller. This appears as a "sag" in the commanded current (and in turn, output force). See `test_rig/max_mod_current.png` and `test_rig/max_mod_force.png`. The solution is to increase the DC bus voltage. 



### Notes on configuration

JCS configurations and parameters are not stored on the device.
If you would like a parameter to be configured at each startup, it must be present in the appropriate configuration file.

