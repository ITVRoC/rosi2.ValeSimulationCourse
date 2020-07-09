

function sysCall_init()
    -- do some initialization here
end

function sysCall_actuation()
    
    -- retrieving the joystick list
    local numberOfJoyticks=simJoy.getCount()
    --local numberOfJoysticks = sim.
    --print(numberOfJoyticks)
    
    local axes,buttons,rotAxes,slider,pov = simJoy.getData(0)
    
    print('----')
    print(axes)
    print(buttons)
    print(slider)
    print(pov)

end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
