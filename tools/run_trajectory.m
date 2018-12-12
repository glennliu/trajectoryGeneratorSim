function run_trajectory(segment,transit_pose,path)
frame_count = size(path,1);
velocity = zeros(frame_count,4);
acce = zeros(frame_count,4);

% position
subplot(3,2,1);
plot(path(:,1),path(:,2:4));
grid on;
title('Position (m)');
xlabel('t (s)');
legend('x','y','z');

% velocity
subplot(3,2,3);
velocity(:,1) = path(:,1);
velocity(:,2) = gradient(path(:,2));
velocity(:,3) = gradient(path(:,3));
velocity(:,4) = gradient(path(:,4));
plot(velocity(:,1),velocity(:,2:4));
title('Velocity (m)');
xlabel('t (s)');
grid on;
legend('vx','vy','vz');

% acceleration
subplot(3,2,5);
acce(:,1) = velocity(:,1);
acce(:,2) = gradient(velocity(:,2));
acce(:,3) = gradient(velocity(:,3));
acce(:,4) = gradient(velocity(:,4));
plot(acce(:,1),acce(:,2:4));
title('Accelearation (m)');
xlabel('t (s)');
legend('ax','ay','az');
grid on;

% 3d Trajectory
subplot(3,2,[2,4,6]);
title('Trajectory in 3d');
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
plot3(path(:,2),path(:,3),path(:,4));
grid on;
axis([-8 8 -8 8 0 5]);
hold on;
plot3(segment(1,1),segment(1,5),segment(1,9),'*')
hold on;
plot3(segment(2,1),segment(2,5),segment(2,9),'*')
hold on;
plot3(transit_pose(:,1),transit_pose(:,2),transit_pose(:,3),'+');
legend()


end