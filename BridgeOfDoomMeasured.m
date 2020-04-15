load('bridgeData.mat', 'dataset')
load('predictData.mat', 'r_num', 'T_hat_num', 'N_hat_num', 'B_hat_num', 'linear_velocity_num', 'omega_num', 'V_l_num', 'V_r_num')
time = dataset(:,1);
pos_l = dataset(:,2);
pos_r = dataset(:,3);

% figure(1)
% plot(pos_l)
% hold on
% plot(pos_r)
% xlabel('Index')
% ylabel('Linear Travel [m]')
% legend('Left Wheel','Right Wheel')
% title('Raw Encoder Data')

%remove offset
pos_l = pos_l-mean(pos_l(1:10));
pos_r = pos_r-mean(pos_r(1:10));

%Graph data w/o offset
% figure(2)
% plot(pos_l)
% hold on
% plot(pos_r)
% xlabel('Index')
% ylabel('Linear Travel [m]')
% legend('Left Wheel','Right Wheel')
% title('Encoder Data with Offset Removed')

%find velocities
dt = diff(time);
dL = diff(pos_l);
dR = diff(pos_r);

vL = dL./dt;
vR = dR./dt;

%plot velocities
figure(1)
plot(t_num, V_r_num, t_num, V_l_num) % plot predicted
hold on
plot(time(1:end-1),vL,'--') % plot measured
hold on
plot(time(1:end-1),vR,'--') % plot measured
xlabel('Time [s]')
ylabel('Wheel Speed [m/s]')
legend('Predicted Right Wheel','Predicted Left Wheel','Measured Left Wheel','Measured Right Wheel')
title('Predicted Wheel Speed vs. Measured Wheel Speed')
hold off

%find linear and angular velocity
d = 0.235;
linear_velocity = (vL+vR)./2;
omega = (vR-vL)./d;

%plot linear and angular velocity
figure(4)
yyaxis left
plot(time(1:end-1),linear_velocity, t_num, linear_velocity_num) % plot predicted versus measured
ylim([-3 3])
xlabel('Time [s]')
ylabel('Linear Speed [m/s]')

yyaxis right
plot(time(1:end-1),omega, t_num, omega_num) % plot predicted versus measured
ylabel('Angular Velocity [rad/s]')
title('Predicted Linear and Angular Velocity vs. Measured Linear and Angular Velocity')
legend('Measured Linear Velocity','Predicted Linear Velocity','Measured Angular Velocity','Predicted Angular Velocity')

%reconstruct the path
r = zeros(length(time),2);
theta = zeros(length(time),1);

for n=1:length(dt)
    r(n+1,1)=r(n,1)+linear_velocity(n)*cos(theta(n))*dt(n);
    r(n+1,2)=r(n,2)+linear_velocity(n)*sin(theta(n))*dt(n);
    theta(n+1) = theta(n) + omega(n)*dt(n);
end

%plot predicted vs. measured robot path
figure(5)
hold on
plot3(r_num(:,1),r_num(:,2),r_num(:,3)) % plot predicted
plot(r(:,1),r(:,2),'--') % plot measured
hold on
quiver(r(1:20:length(r),1),r(1:20:length(r),2),cos(theta(1:20:length(r))),sin(theta(1:20:length(r)))); % plot linear velocity of measured

quiver(r_num(1:200:length(r_num),1),r_num(1:200:length(r_num),2),T_hat_num(1:200:length(r_num),1),T_hat_num(1:200:length(r_num),2),'r') % plot the unit tangent

quiver(r_num(1:200:length(r_num),1),r_num(1:200:length(r_num),2),N_hat_num(1:200:length(r_num),1),N_hat_num(1:200:length(r_num),2),'b') % plot the unit normal
hold off
axis equal
legend('Predicted Robot Position','Measured Robot Position','Measured Linear Velocity','Predicted Unit Tangent', 'Predicted Unit Normal')
xlabel('x position [m]')
ylabel('y position [m]')
title('Predicted Positon and Tangent Vectors vs. Measured Position and Tangent Vectors')
