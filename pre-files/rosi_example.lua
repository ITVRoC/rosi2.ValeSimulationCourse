
-- wheel radius
_wheel_radius = 0.13 -- meters

-- mobile parameters
_linear_speed_max = 0.5
_angular_speed_max = 0.5

-- joystick max input value
_joy_input_max = 1000

-- keyboard input translation
_kb_arrow_up = 2007
_kb_arrow_down = 2008
_kb_arrow_left = 2009
_kb_arrow_right = 2010

function sysCall_init()
    
    -- computing the max traction joints rotation speed
    _joint_speed_max = _linear_speed_max/_wheel_radius
    
    -- getting the rosi handler
    handler_rosi = sim.getObjectHandle("rosi")

    -- Cylinders and its joints handlers and small treatments
    cylinder_handlers = {}
    cylinder_radius = {}
    cylinder_joint_handlers = {}
    path_handlers = {}
    for i=1,4,1 do
        cylinder_handlers[i] = {}
        cylinder_radius[i] = {}
        cylinder_joint_handlers[i] = {}

        for j=0,12,1 do

            -- obtaining the handler for each cylinder's joint
            cylinder_joint_handlers[i][j] = sim.getObjectHandle("joint_Cylinder"..tostring(i).."_"..tostring(j))

            -- obtaining the handler for each cylinder
            cylinder_handlers[i][j] = sim.getObjectHandle("Cylinder"..tostring(i).."_"..tostring(j))

            -- storing     local gyroData={0,0,0}the cylinders' radius size
            _,_,aux_dimensions  = sim.getShapeGeomInfo(cylinder_handlers[i][j])
            cylinder_radius[i][j] = aux_dimensions[1]/2 -- divised by 2 to become a radius
        end
    
        -- getting paths' handlers
        path_handlers[i] = sim.getObjectHandle("arm_path"..tostring(i))
    end

    -- getting arm and wheel joints handlers
    arm_joint_handlers = {}
    traction_joint_handlers = {}
    for i=1,4,1 do
        arm_joint_handlers[i] = sim.getObjectHandle("arm_dyn_joint"..tostring(i))
        traction_joint_handlers[i] = sim.getObjectHandle("wheel_joint"..tostring(i))
    end
    
end

function sysCall_actuation()
    
    -- reading the joystick
    --local numberOfJoyticks=simJoy.getCount()
    
    -- retrieves the joystick input
    local joy_axes, joy_button,_,_,_ = simJoy.getData(0)
    
    -- retrieve the keyboard input
    local keyb_input = get_keyboard()
    
    -- computes the desired velocities given the input
    vel_des = desired_vel_gen(joy_axes, keyb_input)
    
    -- computes traction joints speed given the desired velocities
    vel_traction_input = rosi_kin(vel_des['linear'], vel_des['angular'])
    
    -- applies the input velocities to the joints
    apply_joints_velocity(vel_traction_input)

end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details

-- Joystick input converter
function desired_vel_gen(joy_input, keyb_input)
    
    -- treating the joystick input
    local joy_order = {}
    joy_order['linear'] = ((-1*joy_input[2])/_joy_input_max) 
    joy_order['angular'] = ((-1*joy_input[1])/_joy_input_max) 
    
    -- treating the keyboard input
    -- tests if any message has been received
    local keyb_order= {}
    if keyb_input == 0 then
        keyb_order['linear'] = 0
        keyb_order['angular'] = 0
    else
    
        -- treating the linear part
        if keyb_input == _kb_arrow_up then
            keyb_order['linear'] = 1
        elseif keyb_input == _kb_arrow_down then
            keyb_order['linear'] = -1
        else
            keyb_order['linear'] = 0
        end
        
        -- treating the angular part
        if keyb_input == _kb_arrow_right then
            keyb_order['angular'] = -1
        elseif keyb_input == _kb_arrow_left then
            keyb_order['angular'] = 1
        else
            keyb_order['angular'] = 0
        end

    end -- end of keyboard input testing
    
    -- unifying joystick and keyboard commands
    local final_order = {}
    
    -- treats the linear input
    if keyb_order['linear'] ~= 0 then
        final_order['linear'] = keyb_order['linear']
    elseif joy_order['linear'] ~= 0 then
        final_order['linear'] = joy_order['linear']
    else
        final_order['linear'] = 0
    end
    
    -- treats the angular input
    if keyb_order['angular'] ~= 0 then
        final_order['angular'] = keyb_order['angular']
    elseif joy_order['angular'] ~= 0 then
        final_order['angular'] = joy_order['angular']
    else
        final_order['angular'] = 0
    end
    
    -- generating the final command
    desired_vel = {}
    desired_vel['linear'] = final_order['linear'] * _linear_speed_max
    desired_vel['angular'] = final_order['angular'] * _angular_speed_max
    
    return desired_vel
    
end

-- Rosi base kinematic function
function rosi_kin(vel_lin, vel_ang)

    -- equations below come from the skid-steer model
   joints_speed = {}
   joints_speed['left'] = 7.7*vel_lin - 5.35*vel_ang
   joints_speed['right'] = 7.7*vel_lin + 5.35*vel_ang 
   
   return joints_speed
end

-- retrieves the joint velocity
getJointVelocity=function(joint_handler)
local res,v=sim.getObjectFloatParameter(joint_handler,sim.jointfloatparam_velocity)
    return v
end

--- SET_TRACTION_SPEED -------------------------
-- This function receives the desired velocity for the track and applies it to the dynamic 
-- cylinders, and also the emulated track
-- INPUT
-- lin_speed in m/s
function set_traction_joint_velocity(arm_number, joint_speed)

     -- robot side corrector
    if arm_number <= 2 then
        side_factor = -1
    else
        side_factor = 1
    end

    -- constrains the joint speed
    if joint_speed > _joint_speed_max then
        joint_speed = _joint_speed_max
    elseif joint_speed < -_joint_speed_max then
        joint_speed = -_joint_speed_max
    end

    -- treating the emulated track speed
    local dt = sim.getSimulationTimeStep()
    local path_pos = sim.getPathPosition(path_handlers[arm_number])
    path_pos=path_pos+(side_factor*(_wheel_radius*joint_speed))*dt
    sim.setPathPosition(path_handlers[arm_number],path_pos) -- update the path's intrinsic position

    -- setting the angular velocity to the cylinder joints
    for j=0,12,1 do
        sim.setJointTargetVelocity(cylinder_joint_handlers[arm_number][j], side_factor * (_wheel_radius*joint_speed) / cylinder_radius[arm_number][j])
    end
    
    -- setting the wheel joint speed
    sim.setJointTargetVelocity(traction_joint_handlers[arm_number], side_factor * joint_speed)

end


-- Retrieves the keyboard information
function get_keyboard()

    local message, keyb_data, keyb_data2 = sim.getSimulatorMessage()
    if message == sim_message_keypress then
        return keyb_data[1]
    else
        return 0
    end

end

-- Applies velocity to the traction joints
function apply_joints_velocity(vel_traction_input)

    set_traction_joint_velocity(1, vel_traction_input['right'])
    set_traction_joint_velocity(2, vel_traction_input['right'])
    set_traction_joint_velocity(3, vel_traction_input['left'])
    set_traction_joint_velocity(4, vel_traction_input['left'])
    
end
