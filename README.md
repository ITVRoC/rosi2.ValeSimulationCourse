# rosi2.ValeSimulationCourse
Simulation codes and files for Vale course.

# Kinematics

Skid-steer identification
- lambda: 0.999
- y_cir: 0.6956

# Using joystick in CoppeliaSim

```
local numberOfJoyticks=simExtJoyGetCount()
local axes,buttons,rotAxes,slider,pov=simExtJoyGetData(deviceIndex)
```

axes = {left_horizontal[-1000,1000], left_vertical[1000,-1000], left_right_triggers[996, -996]}

buttons:
	b: 1
	a: 2
	x: 8
	y: 4
	l1: 16
	r1: 32
	start: 128
	select: 64

# Using keyboard in CoppeliaSim

```
# keyboard tests
    message, data, data2 = sim.getSimulatorMessage()
    if message==sim_message_keypress then
        print(data)
    end
```

## `data[0]` variable correspondence:

The keyboard ouput will be on `data[0]`.

- 2007: arrow up
- 2008: arrow down
- 2009: arrow left
- 2010: arrow right
- 49: num pad 1
- 50: num pad 2
- 52: num pad 4
- 53: num pad 5
- 54: num pad 6
- 113: Q button
- 119: W button
- 101: E button
- 114: R button
- 116: T button
- 121: Y button

# `data[1]` variable correspondence:
For target cartesian control we will sometimes use shift button.Its output will be on `data[1]` and for in some cases `data[0]` will be modified

- 2: shift button pressed
- 2010,2: shift + left
- 2010,2: shift + right
- 81,2: shift + Q -> Q data[0] modified!







retencao@valenet.com.br

