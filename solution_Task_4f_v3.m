% Use this script as a template for your code
% You might have to make several copies of the script for the different subtasks
% Communication to v-rep is already incorporated
% To see the robot in action, please ensure that v-rep is running before you execute this script

% Handles:
%  - The handle called bodyFrame will return the robot's position in the inertial frame 
%  - The motor handles give you access to the motor in v-rep
%  - No need for sensors at this point. Only robot position should serve as feedback

% For errors or questions send me an e-mail: nasser.gyagenda@uni-siegen.de

%% DO NOT CHANGE ANYTHING HERE - Setting up the remote api
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
connected = false;

if (clientID>-1)
    connected = true;
    disp('Connected to remote API server');
end

% Vehicle parameters
a = 0.158516;       % COM to any front/back wheels [m]
b = 0.228020;       % COM to any right/left wheels [m]
R = 0.050369;     % Wheel radius [m]

if(connected)
    % handles to wheel motors
    [returnCode,wheelFL]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_blocking);
    [returnCode,wheelFR]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_blocking);
    [returnCode,wheelRR]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_blocking);
    [returnCode,wheelRL]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_blocking);
    
    % handle to robot body reference frame
    [returnCode,bodyFrame]=vrep.simxGetObjectHandle(clientID,'body_frame',vrep.simx_opmode_blocking);

end

%% To Do
if(connected)
    % initialization
    % time
    t=clock;
    startTime=t(6);
    currentTime=t(6);

    % vehicle parameters
    a = 0.158516;
    b = 0.228020;
    R = 0.050369;
    alpha = 0.785;
    set_error = 0.1;
    
    % target position matrix
    target_pos_path  =[  -0.63, -0.05;
                        -0.675, -1.95;
                         -2.22, -1.95;
                         -2.22,  0.27;
                         -1.27,  1.92;
                           1.9,  1.95;
                           1.9, -2.12;
                          0.15, -2.12;
                          0.15, -0.75;
                          1.12,  -0.7;
                          1.12,  1.22;
                          0.22,  1.24];

    
    for n=1:size(target_pos_path)
        target_pos = target_pos_path(n,:);
        error_sum = 10;
        last_rotation_angle = 1.57;
        last_real_ornt = 1.57;
        dis_error_sum = 0;
        ornt_error_sum = 0;
        error_past = 0;
        time_past = 0;
        % loop
        while (error_sum > 0.05)
        % while(currentTime-startTime < 10) % this condition just used for debug
            [returnCode,real_pos] = vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_streaming);% return the absolute position of this robot
            [returnCode,real_ornt] = vrep.simxGetObjectOrientation (clientID,bodyFrame,-1,vrep.simx_opmode_streaming);% return the absolute orientation  of this robot
            
            
            % drop the wrong orientation information
            if (real_ornt(2)==real_ornt(3))
                continue;
            end
            
            fprintf('target_pos:%d,%d\n',target_pos(1),target_pos(2));
            fprintf('real_pos:%d,%d\n',real_pos(1),real_pos(2));
            fprintf('Real_ornt:%d\n',real_ornt(3));
            
            % calculate part
            dis =sqrt( (target_pos(1)-real_pos(1))^2+(target_pos(2)-real_pos(2))^2);
            %fprintf('dis:%d\n',dis);
            theta_1 = atan2((target_pos(2)-real_pos(2)),target_pos(1)-real_pos(1));
            fprintf('Theta_1:%d\n',theta_1);
            
            % orientation correction
            if(sign(real_ornt(3)) ~= sign(theta_1))
                if(theta_1>=pi/2 && real_ornt(3)<= 0)
                    theta_1 = theta_1 - pi;
                    real_ornt(3) = real_ornt(3)+ pi;
                elseif(theta_1<= -pi/2 && real_ornt(3)>= 0)
                    theta_1 = theta_1 + pi;
                    real_ornt(3) = real_ornt(3) - pi;
                end
                fprintf('ÐÞÕý1-->');
                fprintf('Real_ornt:%d\n',real_ornt(3));
                fprintf('       Theta_1:%d\n',theta_1);
                fprintf('       Rotation_angle:%d\n',rotation_angle);
            end
            rotation_angle = real_ornt(3)-theta_1;
            
            if(abs(rotation_angle)>pi)
                rotation_angle = 2*pi-abs(rotation_angle);
                fprintf('ÐÞÕý2-->');
            end
            
            fprintf('Rotation_angle:%d\n',rotation_angle);
            % orientation controller
            if(abs(rotation_angle) >  set_error)               
                [u, ornt_error_sum, error_past, time_past]=pid_controller(rotation_angle, ornt_error_sum, error_past, time_past);
                
                control_effect = 1/R * [-(a + b), (a+b), -(a+b), a+b] * u ;
	
            % position controller
            else
                [u, dis_error_sum, error_past, time_past]=pid_controller(dis, dis_error_sum, error_past, time_past);   
                control_effect = 1/R * u * [1, 1, 1, 1];
            end
            
 
            if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFL,control_effect(1),vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFR,control_effect(2),vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRL,control_effect(3),vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRR,control_effect(4),vrep.simx_opmode_blocking);
            end
            
            % calculate total error
            error_pos = [target_pos(1) - real_pos(1),target_pos(2) - real_pos(2),0];
            error_sum =  abs(error_pos(1)) + abs(error_pos(2)) + abs(error_pos(3));

            t=clock;
            currentTime=t(6);

            disp(n)
        end
        % disp('Stage movement completed')
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFL,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFR,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRL,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRR,0,vrep.simx_opmode_blocking);
    end
    disp('Done Move');    
        
	% Now send some data to CoppeliaSim in a non-blocking fashion:
	vrep.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',vrep.simx_opmode_oneshot);

	% Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
	vrep.simxGetPingTime(clientID);
    %----------------------------------------------------------
    
    % Now close the connection to V-REP:    
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!

function [u, error_sum, error_past, time_past]=pid_controller(error, error_sum, error_past, time_past)
    % pid parameters
    kp_ = 0.1;
    ki_ = 0.01;
    kd_ = 0.07;
    max_error_sum = 100;
    
    fprintf('\n error:%d\n', error)
    time =clock;
    time = time(6);
    error_sum =error_sum + error;
    if(abs(error_sum)> max_error_sum)
        error_sum = max_error_sum * sign(error_sum);
    end
    
    % fprintf('error_sum:%d\n', error_sum)
    delta_error = error - error_past;
    error_past = error;
    delta_time = time - time_past;
    time_past = time;
    
    p_p = kp_ * error;
    p_i = ki_ * error_sum;
    p_d = kd_ * (delta_error/delta_time);
    
%     fprintf('p_p:%d\n', p_p);
%     fprintf('p_i:%d\n', p_i);
%     fprintf('p_d:%d\n', p_d);

    u = abs(p_p + p_i + p_d);
    
    fprintf('u:%d\n',u);
end