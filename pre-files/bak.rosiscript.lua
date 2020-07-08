--- ======== ROSI SIMULATION SCRIPT ===================
-- made by Filipe Rocha - f.rocha41@gmail.com


-- whell radius
wheel_radius = 0.1324 -- [m]

-- robot maximum linear speed
max_linear_speed = 10 -- [m/s]

-- track arms maximum rotation speed
max_armsRotSpeed = 1.5 --[rad/s]

-- rotational arm maximum torque
max_armsTorque = 10

-- helpful constants
_giga = math.pow(10,9)

-- initialization code
function sysCall_init()
    
        -- sends a message to the user
        print('=================================================')
        print('ROSI control srosi_interfacescript started')
        print('')
        print('rosi_control_script: Initiating modules...')

        -- treating support variables
        tractionJointSpeed_max = max_linear_speed/wheel_radius

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

        --- ENABLING ROS 2 CONECTION
        print('')
        print('Looking up for ROS connection:')
        
        if simROS2 then
            print("<font color='#0F0'>ROS2 interface was found!</font>@html")
            phandler_rosiluginNotfound = false
        else
            print("<font color='#F00'>ROS2 interface was not found. Won't run properly. Have you prior sourced ROS 2 environment?</font>@html")
            pluginNotfound = true
        end
                  
        -- Ok, now launch the ROS2 client application:
        if (not pluginNotFound) then

            local sysTime=sim.getSystemTimeInMs(-1) 
            
            -- ROS topics publisher
            pub_simulationTime = simROS2.advertise('/simulation/time','builtin_interfaces/Time')
            pub_armsPosition = simROS2.advertise('/rosi/telemetry/arms_joints_position','rosi_interfaces/RosiJointArray')
            pub_armsTorque = simROS2.advertise('/rosi/telemetry/arms_joints_torque','rosi_interfaces/RosiJointArray')
            pub_armsVelocity = simROS2.advertise('/rosi/telemetry/arms_joints_velocity','rosi_interfaces/RosiJointArray')
            pub_tractionVelocity = simROS2.advertise('/rosi/telemetry/traction_joints_velocity','rosi_interfaces/RosiJointArray')
            pub_tractionTorque = simROS2.advertise('/rosi/telemetry/traction_joints_torque','rosi_interfaces/RosiJointArray')
            
            pub_cheatPose = simROS2.advertise('/cheat/pose_platform','geometry_msgs/PoseStamped')

            -- Model ROS topics subscribers
            sub_tractionSpeed = simROS2.subscribe('/rosi/actuation/traction_speed','rosi_interfaces/RosiJointArray','callback_tractionSpeed')
            sub_armsSpeed = simROS2.subscribe('/rosi/actuation/rotationalArms_speed','rosi_interfaces/RosiJointArray','callback_armsSpeed')

            -- Now we start the client application:
            result = sim.launchExecutable('Rosi','',0)

            -- receives ROSI initial pose
            tf_rosiInitialPose = getTransformStamped(handler_rosi,'static_rosiInitialPose',-1,'map')

        end

        -- Reading ROS simulation parameters
        --_, rosParam_rendering = simROS.getParamBool('/rosi_simulation/simulation_rendering',true)
        --_, rosParam_kinectProcessing = simROS.getParamBool('/rosi_simulation/kinect_processing', true)
        --_, rosParam_velodyneProcessing = simROS.getParamBool('/rosi_simulation/velodyne_processing', true)
        --_, rosParam_hokuyoProcessing = simROS.getParamBool('/rosi_simulation/hokuyo_processing', true)
        --_, rosParam_hokuyoLines = simROS.getParamBool('/rosi_simulation/hokuyo_lines', true)
        --_, rosParam_ur5toolCam = simROS.getParamBool('/rosi_simulation/ur5toolCam_processing',true)
        --_, rosParam_fire_rendering = simROS.getParamBool('/rosi_simulation/fire_rendering',true)
        --_, rosParam_time_header_getSimTime = simROS.getParamBool('/rosi_simulation/time_header_getSimTime', false)
            
        -- setting renderization parameter
        --sim.setBoolParameter(sim.boolparam_display_enablepub_tractionTorqued,rosParam_rendering)

        --- Mesname: topic name must not end with a forward slash:sages to the user
        --print('')
        --print('Loading ROS parameters for the simulation:')
        --print('(You can change these in <rosi_defy>/config/simulation_parameters.yaml or online via *rosparam set* command)')
        
        --print('- Simulation rendering set to: '..tostring(rosParam_rendering))
        --print('- Kinect processing set to: '..tostring(rosParam_kinectProcessing))
        --print('- Velodyne processing set to: '..tostring(rosParam_velodyneProcessing))
        --print('- Hokuyo processing set to: '..tostring(rosParam_hokuyoProcessing))
        --print('- Hokuyo lines plotting set to: '..tostring(rosParam_hokuyoLines))
        --print('- UR5 tool Cam processing set to: '..tostring(rosParam_ur5toolCam))
        --print('- Fire rendering set to: '..tostring(rosParam_fire_rendering))
        --print('- Message time from Simulation set to: '..tostring(rosParam_time_header_getSimTime))
    
        --- Final message to the user
        print('')
        print('rosi_control_script: Running.')
        print('=================================================')
end

-- cleaning function
function sysCall_cleanup()
    --simROS2.shutdownPublisher(pub_simulationTime)
    --simROS2.shutdownPublisher(pub_armsPosition)
    --simROS2.shutdownSubscriber(sub_tractionSpeed)
    --simROS2.shutdownSubscriber(sub_armsSpeed)

    -- turns on again the rendering
    --sim.setBoolParameter(sim.boolparam_display_enabled,true)
end

--- Actuation Code
function sysCall_actuation()


    -- publishing simulation time
    local time_current = sim.getSimulationTime()
    
    -- splitting simulation time into secs and nsecs
    simulation_time = {}
    simulation_time["nsec"] = math.floor(time_current%1 * _giga) 
    simulation_time["sec"] = math.floor(time_current)
    
    -- receiving ROS time
    -- simulation_time = simROS2.getTime()
    
    -- creating a common header for rosi_base frame
    current_time_stamp = {sec=simulation_time["sec"] ,nanosec=simulation_time["nsec"]}
    rosi_common_header={stamp=current_time_stamp, frame_id="rosi_base"}
    
    -- creating aux variables
    local arms_position_list = {}
    local arms_torque_list = {}
    local arms_velocity_list = {}
    local traction_velocity_list = {}
    local traction_torque_list = {}
    
    -- mounting variable lists for arms and tractions joints
    for i=1,4,1 do

        -- i-th arms joints position
        arms_position_list[i] = {node_id=i, joint_var=sim.getJointPosition(arm_joint_handlers[i])}
        
        -- i-th arms joint torque
        arms_torque_list[i] = {node_id=i, joint_var=sim.getJointForce(arm_joint_handlers[i])}

        -- i-th arms joint velocity 
        arms_velocity_list[i] = {node_id=i, joint_var=getJointVelocity(arm_joint_handlers[i])}
        
        -- i-th traction joint torque
        traction_torque_list[i] = {node_id=i, joint_var=sim.getJointForce(traction_joint_handlers[i])}
        
        -- i-th traction joint velocity
        traction_velocity_list[i] = {node_id=i, joint_var=getJointVelocity(traction_joint_handlers[i])}
        
    end
    
    -- publishing simulation time
    simROS2.publish(pub_simulationTime, current_time_stamp)
    
    -- mounting final arms position message and publishing it
    -- local arms_position_message={header={stamp={sec=simulation_time["sec"] ,nanosec=simulation_time["nanosec"]},frame_id="rosi_base"}, joint_array=arms_position_list}
    local arms_position_message={header=rosi_common_header, joint_array=arms_position_list}
    simROS2.publish(pub_armsPosition, arms_position_message)
    
    -- mounting final arms torque message
    local arms_torque_message={header=rosi_common_header, joint_array=arms_torque_list}
    simROS2.publish(pub_armsTorque, arms_torque_message)
    
    -- mounting final arms position message     
    local arms_velocity_message={header=rosi_common_header, joint_array=arms_velocity_list}
    simROS2.publish(pub_armsVelocity, arms_velocity_message)
     
    -- mounting final traction torque message
    local traction_velocity_message={header=rosi_common_header, joint_array=traction_torque_list}
    simROS2.publish(pub_tractionTorque, traction_velocity_message)
    
    -- mounting final traction velocity message
    local traction_velocity_message={header=rosi_common_header, joint_array=traction_velocity_list}
    simROS2.publish(pub_tractionVelocity, traction_velocity_message)
    
    
    --- CHEAT ZONE BELOW ---
    
    -- publishing robot current Pose
    rosi_pos=sim.getObjectPosition(handler_rosi,-1) -- retrives the absolute position 
    rosi_ori=sim.getObjectQuaternion(handler_rosi,-1) -- retrieves the absolute platform orientation in quaternions
    rosi_pose={position={x=rosi_pos[1],y=rosi_pos[2],z=rosi_pos[3]},orientation={x=rosi_ori[1],y=rosi_ori[2],z=rosi_ori[3],w=rosi_ori[4]}}
    cheatPose_message={header=rosi_common_header, pose=rosi_pose}
    simROS2.publish(pub_cheatPose, cheatPose_message)

end


-- ==================================================================
-- =================== CALLBACK FUNCTIONS============================
-- ==================================================================

-- CALLBACK_TRACTIONSPEED
-- This function is triggered by a ROS topic callback
-- It receives and applies a command to ROSI traction system
function callback_tractionSpeed(msg)
    for i=1,4,1 do
        set_traction_joint_velocity(msg.joint_array[i].node_id, msg.joint_array[i].joint_var)
    end
end

-- CALLBACK_ARMSSPEED
-- This function is triggered by a ROS topic callback
-- It receives and applies a command to ROSI arm joint
function callback_armsSpeed(msg)
    for i=1,4,1 do
        set_arm_rotational_speed(msg.joint_array[i].node_id, msg.joint_array[i].joint_var)
    end
end

-- ==================================================================
-- =================== CUSTOM FUNCTIONS =============================
-- ==================================================================


--- SET_ARM_JOINT_ROTATIONAL_SPEED --------------------------------
-- This function received the desired rotational speed to the arms and applies it
-- INPUT
-- rotational speed in rad/s
function set_arm_rotational_speed(arm_number, joint_speed)

    -- robot side corrector
    local side_factor 
    if arm_number <= 2 then
        side_factor = -1
    else
        side_factor = 1
    end

    -- constrains the joint speed
    if joint_speed > max_armsRotSpeed then
        joint_speed = max_armsRotSpeed
    elseif joint_speed < -max_armsRotSpeed then
        joint_speed = -max_armsRotSpeed
    end

    -- setting the joint speed
    sim.setJointTargetVelocity(arm_joint_handlers[arm_number], side_factor * joint_speed)

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
    if joint_speed > tractionJointSpeed_max then
        joint_speed = tractionJointSpeed_max
    elseif joint_speed < -tractionJointSpeed_max then
        joint_speed = -tractionJointSpeed_max
    end

    -- treating the emulated track speed
    local dt = sim.getSimulationTimeStep()
    local path_pos = sim.getPathPosition(path_handlers[arm_number])
    path_pos=path_pos+(side_factor*(wheel_radius*joint_speed))*dt
    sim.setPathPosition(path_handlers[arm_number],path_pos) -- update the path's intrinsic position

    -- setting the angular velocity to the cylinder joints
    for j=0,12,1 do
        sim.setJointTargetVelocity(cylinder_joint_handlers[arm_number][j], side_factor * (wheel_radius*joint_speed) / cylinder_radius[arm_number][j])
    end
    
    -- setting the wheel joint speed
    sim.setJointTargetVelocity(traction_joint_handlers[arm_number], side_factor * joint_speed)

end


-- retrieves the joint velocity
getJointVelocity=function(joint_handler)
local res,v=sim.getObjectFloatParameter(joint_handler,sim.jointfloatparam_velocity)
    return v
end


-- generates a stamped transform
function getTransformStamped(objHandle,name,relTo,relToName)

    -- obtain the body pose
    t=sim.getSystemTime()
    p=sim.getObjectPosition(objHandle,relTo)
    o=sim.getObjectQuaternion(objHandle,relTo)
    
    -- mounting the transform
    return{
        header={
            stamp=t,
            frame_id=relToName
        },
        child_frame_id=name,
        transform={
            -- ROS has definition x=front y=side z=up
            translation={x=p[1],y=p[2],z=p[3]},--V-rep
            rotation={x=o[1],y=o[2],z=o[3],w=o[4]}--v-rep
        }
    }
end


-- Sends a transform 2 ROS
function sendTransformCurrentTime(tf_input)
    
    -- receives the current time
    local t = sim.getSystemTime()
    
    local tf_output = {
        header={
            stamp=t,
            frame_id=tf_input["header"]["frame_id"]
        },
        child_frame_id=tf_input["child_frame_id"],
        transform=tf_input["transform"]
    }

    simROS2.sendTransform(tf_output)
end
