% Generate path with specific segments 
% Plot created path on Rviz


clear all;
%% set paramters
rosinit;
segments_3d = [6 0 0 0 0 0 0 0 3.2 0 0 0
               -6.2 0 0 0 1.2 0 0 0 3.2 0 0 0 
               ];
          
transit_pose_3d = [2.5 0.5 3.5
                    -2.8 3.2 3.2
    ];
           
params.t0 = 0;
params.T = 2;   % segment duration
params.kr = 4; % derivative order
params.frame = 100;    % number of frames
params.num_st = 4; % number of constrained states

%% generate trajectory
path = create_3d_qp_trajectory(segments_3d,transit_pose_3d,params);

%% plot
run_trajectory(segments_3d,transit_pose_3d,path);

%%
pose_pub = rospublisher('/pose','geometry_msgs/PoseStamped');
path_pub = rospublisher('/trajectory','nav_msgs/Path');

pause(2);
pose_count = size(path,1);
pose_msg = rosmessage(pose_pub);
path_msg = rosmessage(path_pub);

for i=1:pose_count
    path_array_msg(i) = rosmessage('geometry_msgs/PoseStamped');
    path_array_msg(i).Pose.Position.X = path(i,2);
    path_array_msg(i).Pose.Position.Y = path(i,3);
    path_array_msg(i).Pose.Position.Z = path(i,4);    
end

path_msg.Header.FrameId = 'map';
path_msg.Poses = path_array_msg;
send(path_pub,path_msg);

t = timer('StartDelay', 4, 'Period', 0.02, 'TasksToExecute', size(path,1), ...
'ExecutionMode', 'fixedRate');
t.TimerFcn = @pub_timer_callback;
t.StartFcn = @init_timer_func;

t.UserData.path = path;
t.UserData.pose_msg = pose_msg;
t.UserData.path_msg = path_msg;
t.UserData.pose_pub = pose_pub;
t.UserData.path_pub = path_pub;


start(t);

function init_timer_func(obj,event)
    hdata = obj.UserData;
    hdata.i = 1;
    obj.Userdata = hdata;
end



