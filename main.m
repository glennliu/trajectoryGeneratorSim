clear all;
%% set paramters

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
path_pub = rospublisher('/path','geometry_msgs/PoseStamped');
pause(2);
path_msg = rosmessage(path_pub);
t = timer('StartDelay', 4, 'Period', 0.02, 'TasksToExecute', size(path,1), ...
'ExecutionMode', 'fixedRate');
t.TimerFcn = @pub_timer_callback;
t.StartFcn = @init_timer_func;

t.UserData.path = path;
t.UserData.path_msg = path_msg;
t.UserData.path_pub = path_pub;

start(t);

function init_timer_func(obj,event)
    hdata = obj.UserData;
    hdata.i = 1;
    obj.Userdata = hdata;
end

