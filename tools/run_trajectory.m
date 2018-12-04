function run_trajectory(path)

subplot(3,2,1);
plot(path(:,1),path(:,2:4));
grid on;
title('Position (m)');
xlabel('t (s)');
legend('x','y','z');

subplot(3,2,3);
title('Velocity (m)');
xlabel('t (s)');
% legend('x','y','z');

subplot(3,2,5);
title('Orientation (m)');
xlabel('t (s)');
% legend('roll','pitch','yaw');

end