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
    [returnCode,bodyFrame]=vrep.simxGetObjectHandle(clientID,'body_frame',vrep.simx_opmode_blocking)

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
    trans_matrix = [1  1  -(a+b);
                    1 -1    a+b ;
                    1 -1  -(a+b);
                    1  1    a+b ];
    
    % target position
    traget_pos_1=[1, 0, 0];
    traget_pos_2=[0, 1, 0];
    traget_pos_3=[1, 1, 0];
    traget_pos_4=[0, 0, 0];
    
    for n=1:4
        eval(['traget_pos','=','traget_pos_',num2str(n),';']);
        error_sum = 10;
        % loop
        while (error_sum > 0.1) 
            [returnCode,real_pos] = vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_streaming);% return the absolute position of this robot
            [returnCode,real_ornt] = vrep.simxGetObjectOrientation (clientID,bodyFrame,-1,vrep.simx_opmode_streaming);% return the absolute orientation  of this robot

            % calculate error in each dimesion
            error_pos = [traget_pos(1) - real_pos(1),traget_pos(2) - real_pos(2),0];
            disp(error_pos)
            % wheels_effect = 0.1*1/R*trans_matrix*error_pos';
            % disp(wheels_effect)

            % disp(control_effect)    
            if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
                control_effect =  move(error_pos);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFL,control_effect(1),vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFR,control_effect(2),vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRL,control_effect(3),vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRR,control_effect(4),vrep.simx_opmode_blocking);
                error_sum =  abs(error_pos(1)) + abs(error_pos(2)) + abs(error_pos(3));
            end
            
            error_sum =  abs(error_pos(1)) + abs(error_pos(2)) + abs(error_pos(3));
            disp(error_sum)
            t=clock;
            currentTime=t(6);
            % disp(currentTime-startTime)
        end
        disp('Stage movement completed')
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


%% Functions
function [effect] = move(error)
    if(error(1)>0.01)
        if(error(2)>0.01)
            effect=[1, 0, 0, 1];
        elseif(error(2)<-0.01)
            effect=[0,-1,-1,0];
        else    
            effect=[1, -1, -1, 1];
        end
    elseif(error(1)< -0.01)
        if(error(2)>0.01)
            effect=[0, 1, 1, 0];
        elseif(error(2)<-0.01)
            effect=[-1,0,0,-1];
        else
        effect=[-1, 1, 1, -1];
        end
    else
        if(error(2)>0.01)
            effect=[1, 1, 1, 1];
        elseif(error(2)<-0.01)
            effect=[-1,-1,-1,-1];
        else
            effect=[0,0,0,0];
        end    
    end
end
