
function pub_timer_callback(obj,event)



hdata = obj.UserData;
path = hdata.path;
pub = hdata.path_pub;
msg =hdata.path_msg;
i = hdata.i;

disp(hdata.i);

msg.Pose.Position.X = path(i,2);   %x
msg.Pose.Position.Y = path(i,3);   %y
msg.Pose.Position.Z = path(i,4);   %z
msg.Header.FrameId = 'map';

send(pub,msg);

hdata.i = hdata.i +1;
obj.UserData = hdata;

end

