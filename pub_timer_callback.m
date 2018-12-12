
function pub_timer_callback(obj,event)



hdata = obj.UserData;
path = hdata.path;
pose_pub = hdata.pose_pub;
path_pub = hdata.path_pub;
pose_msg =hdata.pose_msg;
path_msg = hdata.path_msg;
i = hdata.i;

disp(hdata.i);

pose_msg.Pose.Position.X = path(i,2);   %x
pose_msg.Pose.Position.Y = path(i,3);   %y
pose_msg.Pose.Position.Z = path(i,4);   %z
pose_msg.Header.FrameId = 'map';

send(pose_pub,pose_msg);
send(path_pub,path_msg);

hdata.i = hdata.i +1;
obj.UserData = hdata;

end

