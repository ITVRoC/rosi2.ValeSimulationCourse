-- UR5 ROS CONTROL SCRIPT

--- ======== ROSI SIMULATION SCRIPT ===================
-- made by Filipe Rocha - f.rocha41@gmail.com

--- UR5 parameters ---
ur5_joint_max_torque = 100

--- global variables ---

-- joints operation mode
-- 0 when recently initiated
-- 1 when in velocity mode
-- 2 when in position mode
-- 3 when in torque mode
global_opMode_initiated = 0
global_opMode_velocity = 1
global_opModa_position = 2
global_opMode_torque = 3

-- holds the ur5 operation mode
global_ur5_opMode = global_opMode_initiated


-- helpful constants
_giga = math.pow(10,9)

function sysCall_init()
 
    -- obtaining UR5 joint handlers
    handler_ur5joints = {}
    for i=1,6,1 do
        handler_ur5joints[i] = sim.getObjectHandle("UR5_joint"..tostring(i))
    end

    -- obtaining force Force/Torquen joint handler
    handler_forceSensor = sim.getObjectHandle("UR5_jointFT")
    
    -- obtaining the end effector dummy handler
    handler_endeffector = sim.getObjectHandle("sensor_set")

    --- ENABLING ROS 2 CONECTION
    if simROS2 then
        print("<font color='#0F0'>ROS2 interface was found!</font>@html")
        pluginNotfound = false
    else
        print("<font color='#F00'>ROS2 interface was not found. Won't run properly. Have you prior sourced ROS 2 environment?</font>@html")
        pluginNotfound = true
    end

    -- now, create ros publishers and subscribers
    if (not pluginNotFound) then

        -- ROS topic for ur5 joints positions publication
        pub_ur5JointsPosition = simROS2.advertise('/manipulator/telemetry/joints_position','rosi_interfaces/ManipulatorJoints')
        
        -- ROS topic for ur5 joints velocities publication
        pub_ur5JointsVelocity = simROS2.advertise('/manipulator/telemetry/joints_velocity','rosi_interfaces/ManipulatorJoints')
        
        -- ROS topic for ur5 joints torque publication
        pub_ur5JointsTorque = simROS2.advertise('/manipulator/telemetry/joints_torque','rosi_interfaces/ManipulatorJoints')

        -- ROS topic for sending ur5 force/torque sensor message
        pub_ur5ForceTorqueSensor = simROS2.advertise('/manipulator/telemetry/wrist_force_torque','geometry_msgs/TwistStamped')

        -- ROS topic for receiving ur5 velocity commands
        sub_ur5JointsVelTargetCommand = simROS2.subscribe('/manipulator/actuation/joints_velocity','rosi_interfaces/ManipulatorJoints','callbackManipulatorVelocityCommand')

        -- ROS topic for receiving ur5 position commands
        sub_ur5JointsPosTargetCommand = simROS2.subscribe('/manipulator/actuation/joints_position','rosi_interfaces/ManipulatorJoints','callbackManipulatorPositionCommand')
        
        -- ROS topic for receiving ur5 torque commands
        sub_ur5JointsTorqueTargetCommand = simROS2.subscribe('/manipulator/actuation/joints_torque','rosi_interfaces/ManipulatorJoints','callbackManipulatorTorqueCommand')

        -- CHEAT ZONE BELOW

        -- ROS topic for sending ur5 end-effector pose
        pub_cheat_ur5 = simROS2.advertise('/cheat/pose_endeffector','geometry_msgs/PoseStamped')
    end
end

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
    ur5_common_header={stamp=current_time_stamp, frame_id="ur5"}

    -- retrieves, prepares and publishes the manipulator's joints position
    local manipulator_joints_position = {}
    local manipulator_joints_velocity = {}
    local manipulator_joints_torque = {}
    for i=1,6,1 do
        manipulator_joints_position[i] = sim.getJointPosition(handler_ur5joints[i])
        manipulator_joints_velocity[i] = sim.getJointTargetVelocity(handler_ur5joints[i])
        manipulator_joints_torque[i] = sim.getJointForce(handler_ur5joints[i])
    end
    
    manipulator_joints_position_pubMessage = {header=ur5_common_header, joint_variable=manipulator_joints_position}
    manipulator_joints_velocity_pubMessage = {header=ur5_common_header, joint_variable=manipulator_joints_velocity}
    manipulator_joints_torque_pubMessage = {header=ur5_common_header, joint_variable=manipulator_joints_torque}
    
    simROS2.publish(pub_ur5JointsPosition, manipulator_joints_position_pubMessage)
    simROS2.publish(pub_ur5JointsVelocity, manipulator_joints_velocity_pubMessage)
    simROS2.publish(pub_ur5JointsTorque, manipulator_joints_torque_pubMessage)
    
    -- receiving and publishing the wrist force/torque sensor data
    local forceData={linear={x=0,y=0,z=sim.getJointForce(handler_forceSensor)}, angular={x=0,y=0,z=0}}
    local forceData_pubMessage={header=ur5_common_header, twist=forceData}
        
    simROS2.publish(pub_ur5ForceTorqueSensor, forceData_pubMessage)
    

    -- CHEAT ZONE BELOW
    
     -- publishing end-effector current Pose
    endeffector_pos=sim.getObjectPosition(handler_endeffector,-1) -- retrives the absolute position 
    endeffector_ori=sim.getObjectQuaternion(handler_endeffector,-1) -- retrieves the absolute platform orientation in quaternions
    endeffector_pose={position={x=endeffector_pos[1],y=endeffector_pos[2],z=endeffector_pos[3]},orientation={x=endeffector_ori[1],y=endeffector_ori[2],z=endeffector_ori[3],w=endeffector_ori[4]}}
    cheatPose_message={header=ur5_common_header, pose=endeffector_pose}
    simROS2.publish(pub_cheat_ur5, cheatPose_message)

end


-- clean up function
function sysCall_cleanup()    
    
    -- closing ROS connections
    simROS2.shutdownPublisher(pub_ur5JointsPosition)
    simROS2.shutdownPublisher(pub_ur5JointsVelocity)
    simROS2.shutdownPublisher(pub_ur5JointsTorque)
    simROS2.shutdownPublisher(pub_ur5ForceTorqueSensor)
    
    simROS2.shutdownSubscriber(sub_ur5JointsVelTargetCommand)
    simROS2.shutdownSubscriber(sub_ur5JointsPosTargetCommand)
    simROS2.shutdownSubscriber(sub_ur5JointsTorqueTargetCommand)
    
    simROS2.shutdownPublisher(pub_cheat_ur5)
end


-- UR5 Velocity command callback
function callbackManipulatorVelocityCommand(msg)

    -- tests if it is already operating in velocity mode
    if global_ur5_opMode ~= global_opMode_velocity then
        -- if not, some tweaks must be made ->
        
        -- indicates that now, the UR5 joints are in velocity mode
        global_ur5_opMoeda = global_opMode_velocity
        
        -- sets all joints to work in velocity mode
        for i=1,6,1 do
        
            -- disables the pid control looop
           sim.setObjectInt32Parameter(handler_ur5joints[i], sim.jointintparam_ctrl_enabled, 0)
           
           -- sets the max torque to the nominal value as the control will tune it
           sim.setJointMaxForce(handler_ur5joints[i], ur5_joint_max_torque)
           
           -- sets the current velocity to 0
           sim.setJointTargetVelocity(handler_ur5joints[i], 0)
           
        end
    end

    -- commands the ur5 joints in velocity mode

    -- iterates over all ur5 joints
    for i=1,6,1 do
        sim.setJointTargetVelocity(handler_ur5joints[i], msg.joint_variable[i])
    end

end


-- UR5 Position command callback
function callbackManipulatorPositionCommand(msg)

    -- tests if it is already operating in velocity mode
    if global_ur5_opMode ~= global_opModa_position then
        -- if not, some tweaks must be made ->
        
        -- indicates that now, the UR5 joints are in velocity mode
        global_ur5_opMoeda = global_opModa_position
        
        -- sets all joints to work in velocity mode
        for i=1,6,1 do
        
            -- enables the pid control looop
           sim.setObjectInt32Parameter(handler_ur5joints[i], sim.jointintparam_ctrl_enabled, 1)
           
           -- sets the max torque to the nominal value as the control will tune it
           sim.setJointMaxForce(handler_ur5joints[i], ur5_joint_max_torque)
           
        end
    end

    -- commands the ur5 joints in position mode

    -- iterates over all ur5 joints
    for i=1,6,1 do
        sim.setJointTargetPosition(handler_ur5joints[i], msg.joint_variable[i])
    end

end


-- UR5 Torque command callback
function callbackManipulatorTorqueCommand(msg)
    
    -- tests if it is already operating in torque mode
    if global_ur5_opMode ~= global_opMode_torque then
        -- if not, some tweaks must be made ->
        
        -- indicates that now, the UR5 joints are in torque mode
        global_ur5_opMoeda = global_opMode_torque
        
        -- sets all joints to work in torque mode
        for i=1,6,1 do
        
            -- disables the pid control looop
           sim.setObjectInt32Parameter(handler_ur5joints[i], sim.jointintparam_ctrl_enabled, 0)
           
           -- sets the current velocity to 0
           sim.setJointTargetVelocity(handler_ur5joints[i], 0)
           
           -- sets the max torque to 0, as the control will tune it
           sim.setJointMaxForce(handler_ur5joints[i], 0)
           
        end
    end
            
    -- operates the joints in torque mode using a small tweak
    
    for i=1,6,1 do
        -- iterates over all ur5 joints
        
        -- tests the torque signal
        if (msg.joint_variable[i] <= 0) then
            torque_sign = -1
        else 
            torque_sign = 1
        end 

        -- setting the joints max torque
        sim.setJointMaxForce(handler_ur5joints[i], math.abs(msg.joint_variable[i]))
        
        -- setting an absurd velocity
        -- one may notice that it will not be feasible as the torque is constrained
        sim.setJointTargetVelocity(handler_ur5joints[i], 10000*torque_sign)
    
    end

end

-- You can define additional system calls here:
--[[
function sysCall_suspend()
end

function sysCall_resume()
end

function sysCall_dynCallback(inData)
end

function sysCall_jointCallback(inData)
    return outData
end

function sysCall_contactCallback(inData)
    return outData
end

function sysCall_beforeCopy(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." will be copied")
    end
end

function sysCall_afterCopy(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." was copied")
    end
end

function sysCall_beforeDelete(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." will be deleted")
    end
    -- inData.allObjects indicates if all objects in the scene will be deleted
end

function sysCall_afterDelete(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." was deleted")
    end
    -- inData.allObjects indicates if all objects in the scene were deleted
end

function sysCall_afterCreate(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..value.." was created")
    end
end
--]]
