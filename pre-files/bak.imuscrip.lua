--- ======== ROSI SIMULATION SCRIPT ===================
-- made by Filipe Rocha - f.rocha41@gmail.com

-- initialization function
function sysCall_init()
    
    -- retrieving objects handlers
    handle_gps=sim.getObjectHandle('GPS_reference')
    handle_gyro=sim.getObjectHandle('GyroSensor_reference')

    -- saving the first transformation Matrix
    transformationMatrix_last=sim.getObjectMatrix(handle_gyro,-1)
    
    --- ENABLING ROS CONECTION       
    if simROS2 then
        print("<font color='#0F0'>ROS2 interface was found!</font>@html")
        pluginNotfound = false
    else
        print("<font color='#F00'>ROS2 interface was not found. Won't run properly. Have you prior sourced ROS 2 environment?</font>@html")
        pluginNotfound = true
    end

    -- creating ROS publishers
    if (not pluginNotFound) then
        pub_imu = simROS2.advertise('/rosi/sensors/imu','sensor_msgs/Imu')
        --pub_gps = simROS2.advertise('/rosi/sensors/gps','sensor_msgs/NavSatFix')
    end

    -- seeding the random number
    math.randomseed(os.time())

    -- random componentes limits
    -- we consider a zero centered uniform distribution with below variables x2 of limits
    rnd_uniformLimit_orientation = 0.1
    rnd_uniformLimit_angularVelocity = 0.1
    rnd_uniformLimit_linearAcceleration = 0.1

    -- computing variance matrix (and array, actually) for each data type
    local aux = math.pow(2*rnd_uniformLimit_orientation,2)/12
    rnd_variance_orientation = {aux,aux,aux,aux}

    aux = math.pow(2*rnd_uniformLimit_angularVelocity,2)/12
    rnd_variance_angularVelocity = {aux,aux,aux}

    aux = math.pow(2*rnd_uniformLimit_linearAcceleration,2)/12
    rnd_variance_linearAcceleration = {aux,aux,aux}

    -- GPS parameters
    xShiftAmplitude=0
    yShiftAmplitude=0
    zShiftAmplitude=0
    xShift=0
    yShift=0
    zShift=0
    x_variance=0.333
    y_variance=0.333
    z_variance=0.333

    -- saving the first time step
    time_last=sim.getSimulationTime()
    gyro_linearVelocity_last, _ = sim.getObjectVelocity(handle_gyro)
    
end

-- sensing function
function sysCall_sensing()

    ---==== PRE-TREATMENT

    -- simulation time treating
    local time_current=sim.getSimulationTime()
    local dt=time_current-time_last
    
    -- ros time treating
    ros_time = simROS2.getTime()
    
    --======== IMU TREATMENT =============
    -- publication variables
    local pub_imu_message = {}
    local pub_imu_orientation = {}
    local pub_imu_angularVelocity = {}
    local pub_imu_linearAcceleration = {}

    -- retrieving orientation data
    local orientationData=sim.getObjectQuaternion(handle_gyro, -1)

    -- mounting orientation data
    pub_imu_orientation["x"]=orientationData[1] + math.random(-rnd_uniformLimit_orientation,rnd_uniformLimit_orientation)
    pub_imu_orientation["y"]=orientationData[2] + math.random(-rnd_uniformLimit_orientation,rnd_uniformLimit_orientation)
    pub_imu_orientation["z"]=orientationData[3] + math.random(-rnd_uniformLimit_orientation,rnd_uniformLimit_orientation)
    pub_imu_orientation["w"]=orientationData[4] + math.random(-rnd_uniformLimit_orientation,rnd_uniformLimit_orientation)

    -- retrieving velocity data
    gyro_linearVelocity, gyro_angularVelocity=sim.getObjectVelocity(handle_gyro)

    -- mounting angular velocity
    pub_imu_angularVelocity["x"]=gyro_angularVelocity[1] + math.random(-rnd_uniformLimit_angularVelocity,rnd_uniformLimit_angularVelocity)
    pub_imu_angularVelocity["y"]=gyro_angularVelocity[2] + math.random(-rnd_uniformLimit_angularVelocity,rnd_uniformLimit_angularVelocity)
    pub_imu_angularVelocity["z"]=gyro_angularVelocity[3] + math.random(-rnd_uniformLimit_angularVelocity,rnd_uniformLimit_angularVelocity)

    -- computing object linear acceleration
    pub_imu_linearAcceleration["x"]=((gyro_linearVelocity[1]-gyro_linearVelocity_last[1])/dt) + math.random(-rnd_uniformLimit_linearAcceleration,rnd_uniformLimit_linearAcceleration)
    pub_imu_linearAcceleration["y"]=((gyro_linearVelocity[2]-gyro_linearVelocity_last[2])/dt) + math.random(-rnd_uniformLimit_linearAcceleration,rnd_uniformLimit_linearAcceleration)
    pub_imu_linearAcceleration["z"]=((gyro_linearVelocity[3]-gyro_linearVelocity_last[3])/dt) + math.random(-rnd_uniformLimit_linearAcceleration,rnd_uniformLimit_linearAcceleration)

    -- mounting message for publishing
    pub_imu_message["header"]={stamp={sec=ros_time["sec"], nanosec=ros_time["nanosec"]}, frame_id="rosi_base"}
    pub_imu_message["orientation"]=pub_imu_orientation
    pub_imu_message["orientation_covariance"]=rnd_variance_orientation
    pub_imu_message["angular_velocity"]=pub_imu_angularVelocity
    pub_imu_message["angular_velocity_covariance"]=rnd_variance_angularVelocity
    pub_imu_message["linear_acceleration"]=pub_imu_linearAcceleration
    pub_imu_message["linear_acceleration_covariance"]=rnd_variance_linearAcceleration

    -- publish on ROS the IMU data
    simROS2.publish(pub_imu, pub_imu_message)

    --========== GPS Treatment ==================
    --[[ local pub_gps_message = {}

    xNoiseAmplitude=sim.getScriptSimulationParameter(handle_gps,'xNoiseAmplitude')
    if not xNoiseAmplitude or xNoiseAmplitude<0 then xNoiseAmplitude=0 end
    if xNoiseAmplitude>100 then xNoiseAmplitude=100 end
    
    yNoiseAmplitude=sim.getScriptSimulationParameter(handle_gps,'yNoiseAmplitude')
    if not yNoiseAmplitude or yNoiseAmplitude<0 then yNoiseAmplitude=0 end
    if yNoiseAmplitude>100 then yNoiseAmplitude=100 end
    
    zNoiseAmplitude=sim.getScriptSimulationParameter(handle_gps,'zNoiseAmplitude')
    if not zNoiseAmplitude or zNoiseAmplitude<0 then zNoiseAmplitude=0 end
    if zNoiseAmplitude>100 then zNoiseAmplitude=100 end
    
    xShiftAmplitudeN=sim.getScriptSimulationParameter(handle_gps,'xShiftAmplitude')
    if not xShiftAmplitudeN then xShiftAmplitudeN=0 end
    if xShiftAmplitudeN<0 then xShiftAmplitudeN=0 end
    if xShiftAmplitudeN>100 then xShiftAmplitudeN=100 end
    if (xShiftAmplitudeN~=xShiftAmplitude) then
        xShiftAmplitude=xShiftAmplitudeN
        xShift=2*(math.random()-0.5)*xShiftAmplitude
    end
    
    yShiftAmplitudeN=sim.getScriptSimulationParameter(handle_gps,'yShiftAmplitude')
    if not yShiftAmplitudeN or ShiftAmplitudeN<0  then yShiftAmplitudeN=0 end
    if yShiftAmplitudeN>100 then yShiftAmplitudeN=100 end
    if (yShiftAmplitudeN~=yShiftAmplitude) then
        yShiftAmplitude=yShiftAmplitudeN
        yShift=2*(math.random()-0.5)*yShiftAmplitude
    end
    
    zShiftAmplitudeN=sim.getScriptSimulationParameter(handle_gps,'zShiftAmplitude')
    if not zShiftAmplitudeN or  zShiftAmplitudeN<0 then zShiftAmplitudeN=0 end
    if zShiftAmplitudeN>100 then zShiftAmplitudeN=100 end
    if (zShiftAmplitudeN~=zShiftAmplitude) then
        zShiftAmplitude=zShiftAmplitudeN
        zShift=2*(math.random()-0.5)*zShiftAmplitude
    end 
    
    objectAbsolutePosition=sim.getObjectPosition(handle_gps,-1)
    
    -- Now add some noise to make it more realistic:
    pub_gps_message["latitude"]=objectAbsolutePosition[1]+2*(math.random()-0.5)*xNoiseAmplitude+xShift
    pub_gps_message["longitude"]=objectAbsolutePosition[2]+2*(math.random()-0.5)*yNoiseAmplitude+yShift
    pub_gps_message["altitude"]=objectAbsolutePosition[3]+2*(math.random()-0.5)*zNoiseAmplitude+zShift

    -- add the variance information
    pub_gps_message["position_covariance"]={x_variance,0,0,0,y_variance,0,0,0,z_variance}
    pub_gps_message["position_covariance_type"]=2 -- 2 states for known covariance matrix diagonal 

    -- adding the header to the message
    pub_gps_message["header"]={stamp={sec=ros_time["sec"] ,nanosec=ros_time["nanosec"]},frame_id="rosi_base"}

    -- publishing to ROS
    simROS2.publish(pub_gps,pub_gps_message) ]]

    --========== Post treatment
    gyro_linearVelocity_last=gyro_linearVelocity
    time_last=time_current

end