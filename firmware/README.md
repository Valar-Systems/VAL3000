# FIRMWARE

I've added several differnt types of firmware.

For Matter, I recommend using the ESP-IDF firmware because the Arduino firmware has memory problems which cannot be fixed since Arduino is poorly optimized.

## WARNING
Matter on ESP-IDF can ONLY be used in Linux or Mac. WINDOWS WILL NOT WORK! If you have windows, you MUST use WSL and open the folder in WSL. Doing so is easy. In WSL navigate to the firmware folder above and type "Code ." to open it in VS Code.


## Stallguard

Recommended Workflow for Setting StallGuard
1.	First, I figure out the current to set. I need to have enough current to move whatever needs to be moved, but not much more. If the current is too high, StallGuard will require a lot of force to trip, so don’t set it much higher than the application calls for.
2.	Then I figure out what velocity I want and set that.
3.	Then I figure out what acceleration I want, trying to use the fastest I can for the application.
4.	Then I set the TCOOLTHRS value by monitoring the TSTEP register. It's best to set TCOOLTHRS as close as you can to the TSETP value when operating at normal velocity.  Remember TCOOLTHRS needs to be larger than TSTEP.
5.	Then I start setting SGTHRS at 0 and start moving the motor and applying pressure to it with my fingers. I slowly increase the SGTHRS number until it stalls. Just keep running the motor, applying pressure, and increasing the value. Alternatively, you can monitor SG_RESULT and set the SGTHRS to half of the value where you would like the stall to trigger.

### More Details:

Stallguard is complicated. After years of using StallGuard, here is what you need to know. StallGuard is a combination of your:
1.	Current 
2.	Velocity 
3.	Acceleration
4.	TCOOLTHRS setting 
5.	SGTHRS setting 

### Current:
The current you are using will determine the amount of back EMF created, so first set your current to the level you need for your application. A high current will make the motor louder and stronger. A low current will do the opposite. Dial that in first.

### Velocity:
Next, set the velocity. How fast do you need your motor to go.

### Acceleration:
Next, set the acceleration. StallGuard has a very big downside which is very slow speeds are not reliable for StallGuard. This is because back-EMF is not generated while the motor is in standstill. And not enough back-EMF is generated during the initial acceleration, therefore stallGuard will always trigger during an acceleration. In fact, the driver will most likely not detect any stall during acceleration, until the max speed is reached, so a very long and slow acceleration is not good. Try to accelerate as fast as you can based on your project. Just know during the acceleration phase, StallGuard may not work, so can you risk having it not work until it gets to a stable velocity?

In my window opener, I cannot risk it, because what if the window is locked when trying to open the window? Damage may occur. I need to accelerate as quickly as possible to make sure StallGuard works, but not too fast because the window is heavy and cannot move without some acceleration. Take some time to get your acceleration right.

### TCOOLTHRS setting:
Now we can set the TCOOLTHRS value. It is on page 26 of the datasheet.

This is the velocity threshold at which point StallGuard will turn on. Remember, StallGuard is not reliable at low speeds so Trinamic built this feature that automatically turns StallGuard off until a certain speed is reached. StallGuard becomes enabled when exceeding TCOOLTHRS velocity. It becomes disabled again once the velocity falls below this threshold.

We need to set the correct value, so let's look at the datasheet on page 26 to see how to set it.  

This value can be difficult to understand because we are working with two different velocities. The velocity we have been working with thus far is the one that is set in Arduino and managed by the FastAccelStepper library. However, the TMC2209 does not understand what that means. Instead, it measures velocity in a different way.  

Notice this statement in the datasheet: TCOOLTHRS ≥ TSTEP. Let's go to page 25 to the 0x12 register for TSTEP. This register outputs the velocity. What we need to do is find out the velocity that we are using normally. To do this, place this code into our main loop so we can check what velocity the motor is moving at.  

This value is the actual measured time between two 1/256 microsteps. When the motor moves slower, the value will increase because the time between two microsteps gets longer. When the motor moves faster, the value will decrease.  

Get your motor to operating velocity and check what TSTEP value you get. Notice that as the motor increases in speed, the value gets smaller. Set your TCOOLTHRS value to a number that is larger than the TSTEP value. For example, if normal operating velocity gives you a TSTEP of 300, set TCOOLTHRS to 400 or so. This means that when the motor velocity reaches 400, then StallGuard will turn on. Any slower than 400 and StallGuard will remain off to not give a false positive signal.

#### NOTE: Remember that you cannot run Serial print and the STEP pin at the same time on one processor because Serial print will slow down the STEP pulse and you will not get an accurate value. My workaround on the ESP32 (which is dual core) is to run the motor code on one processor and the print code on the other processor. If you have two Arduinos, you can also use one to generate the STEP pulse and the other to monitor the Serial output. 

You can also use the built in pulse generator and free up your IC for the print command. To use the built in pulse generator, go to page 25 of the datasheet under VACTUAL. Set VACTUAL to a velocity that you would like by using the following code, be sure to replace VELOCITY with whatever works best: 

### SGTHRS setting:
Finally, we need to set the SGTHRS value which is the main value to set. It is a value from 0-255 as seen on page 55. A value of 0 will turn StallGuard off completely. The higher you go, the more sensitive it will be to tripping. On page 55 we can see that the double of this value is compared to SG_RESULT. The stall output becomes active if SG_RESULT falls below this value.

Here is a visual to help you understand. The optimal angle for the magnet to be from the coil is always 90 degrees, which will give us a SG_RESULT value of 510. When no force is applied to the motor, the driver will always maintain the 90-degree angle as it spins the motor.

However, as force is applied to the rotor (for example, if you squeeze the motor shaft with your fingers), it will not be able to maintain the 90-degree angle and will begin to approach 0, which is considered a stall. In the example above, if we read SG_RESULT we will get a value of 255. To set SGTHRS to trigger a stall when the rotor is in this position, we set the value to 255/2 = 127.5 (But we need a whole number so let’s round to 128).

Setting the SGTHRS value to 255 will always trigger a stall. The smaller the number, the more torque is required to trigger a stall. It is up to you to figure out exactly what value you’d like the stall to trigger at.

One way to find the best SG_RESULT and SGTHRS value is to run the code below in your sketch. Squeeze the motor with your fingers and watch the value change. Use it to set your stall value.

