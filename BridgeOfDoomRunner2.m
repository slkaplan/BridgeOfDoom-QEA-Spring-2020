clc, clear all
save bridgeData.mat
collectDataset_sim('bridgeData.mat')
starterCode();
beta = .32367 
disp(i)   

syms u t

assume(0 <= u <= 3.2)
d = 0.235; % wheel distance
steps = 100; % number of simulation steps

u = beta * t;

%parametric curve
ri=4*(0.3960*cos(2.65*(u+1.4)));
rj=-4*(0.99*sin(u+1.4));
rk=0*u;

r=[ri,rj,rk];

dr=diff(r,t); %unit tangent
V = norm(dr); %linear speed

T_hat_ugly=dr./norm(dr);
T_hat=simplify(T_hat_ugly);

dT_hat = diff(T_hat,t);

%angular velocity
omega=simplify(cross(T_hat,dT_hat));

%left and right wheel speeds
VL = simplify(V - omega(3) * (d / 2));
VR = simplify(V + omega(3) * (d / 2));

t_num = linspace(0,3.2/beta,steps);

%set up so I can send velocities to the robot
pub = rospublisher('/raw_vel')
message = rosmessage(pub);
for n=1:length(t_num)
    %calculating left and right wheel speeds
    VL_num(n,:)=double(subs(VL,[t],[t_num(n)]));
    VR_num(n,:)=double(subs(VR,[t],[t_num(n)]));
end
r = rosrate(10.5);
reset(r)

for i = 1:100
    %sending speeds to the robot
    message.Data = [VL_num(i,:), VR_num(i,:)];
    send(pub, message);
    waitfor(r);
    disp(i)
    if i>=100
    message.Data = [0, 0];
    send(pub, message);
    end
end
function starterCode()
% Insert any setup code you want to run here

% define u explicitly to avoid error when using sub functions
% see: https://www.mathworks.com/matlabcentral/answers/268580-error-attempt-to-add-variable-to-a-static-workspace-when-it-is-not-in-workspace
u = [];
% u will be our parameter
syms u;

% this is the equation of the bridge
R = 4*[0.396*cos(2.65*(u+1.4));...
       -0.99*sin(u+1.4);...
       0];

% tangent vector
T = diff(R);    

% normalized tangent vector
That = T/norm(T);

pub = rospublisher('raw_vel');

% stop the robot if it's going right now
stopMsg = rosmessage(pub);
stopMsg.Data = [0 0];
send(pub, stopMsg);

bridgeStart = double(subs(R,u,0));
startingThat = double(subs(That,u,0));
placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), startingThat(2));

% wait a bit for robot to fall onto the bridge
pause(2);

% time to drive!!

% For simulated Neatos only:
% Place the Neato in the specified x, y position and specified heading vector.
function placeNeato(posX, posY, headingX, headingY)
    svc = rossvcclient('gazebo/set_model_state');
    msg = rosmessage(svc);

    msg.ModelState.ModelName = 'neato_standalone';
    startYaw = atan2(headingY, headingX);
    quat = eul2quat([startYaw 0 0]);

    msg.ModelState.Pose.Position.X = posX;
    msg.ModelState.Pose.Position.Y = posY;
    msg.ModelState.Pose.Position.Z = 1.0;
    msg.ModelState.Pose.Orientation.W = quat(1);
    msg.ModelState.Pose.Orientation.X = quat(2);
    msg.ModelState.Pose.Orientation.Y = quat(3);
    msg.ModelState.Pose.Orientation.Z = quat(4);

    % put the robot in the appropriate place
    ret = call(svc, msg);
end
end
function collectDataset_sim(datasetname)
% This script provides a method for collecting a dataset from the Neato
% sensors suitable for plotting out a 3d trajectory.  To launch the
% application run:
%
%    collectDataset_sim('nameofdataset.mat')
%
% Where you should specify where you'd like to the program to save the
% the dataset you collect.
%
% The collected data will be stored in a variable called dataset.  Dataset
% will be a nx6 matrix where each row contains a timestamp, the encoder
% values, and the accelerometer values.  Specifically, here is the row
% format.
%
% [timestamp, positionLeft, positionRight, AccelX, AccelY, AccelZ];
%
% To stop execution of the program, simply close the figure window.

    function myCloseRequest(src,callbackdata)
        % Close request function 
        % to display a question dialog box
        % get rid of subscriptions to avoid race conditions
        clear sub_encoders;
        clear sub_accel;
        delete(gcf)
    end

    function processAccel(sub, msg)
        % Process the encoders values by storing by storing them into
        % the matrix of data.
        lastAccel = msg.Data;
    end

    function processEncoders(sub, msg)
        % Process the encoders values by storing by storing them into
        % the matrix of data.
        if ~collectingData
            return;
        end
        currTime = rostime('now');
        currTime = double(currTime.Sec)+double(currTime.Nsec)*10^-9;
        elapsedTime = currTime - start;
        dataset(encoderCount + 1,:) = [elapsedTime msg.Data' lastAccel'];
        encoderCount = encoderCount + 1;
    end

    function keyPressedFunction(fig_obj, eventDat)
        % Convert a key pressed event into a twist message and publish it
        ck = get(fig_obj, 'CurrentKey');
        switch ck
            case 'space'
                if collectingData
                    collectingData = false;
                    dataset = dataset(1:encoderCount, :);
                    save(datasetname, 'dataset');
                    disp('Stopping dataset collection');
                else
                    start = rostime('now');
                    start = double(start.Sec)+double(start.Nsec)*10^-9;
                    encoderCount = 0;
                    dataset = zeros(100000, 6);
                    collectingData = true;
                    disp('Starting dataset collection');
                end
        end
    end
    global dataset start encoderCount lastAccel;
    lastAccel = [0; 0; 1];      % set this to avoid a very unlikely to occur race condition
    collectingData = false;
    sub_encoders = rossubscriber('/encoders', @processEncoders);
    sub_accel = rossubscriber('/accel', @processAccel);

	f = figure('CloseRequestFcn',@myCloseRequest);
    title('Dataset Collection Window');
    set(f,'WindowKeyPressFcn', @keyPressedFunction);
end
