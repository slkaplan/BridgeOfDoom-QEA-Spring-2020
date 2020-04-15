clc, clear all

syms u t %defining symbolic vars
d = 0.235; %wheel distance
beta = .15; 
num_steps = 1000; 
assume(0 <= u <= 3.2)
u = beta * t; %trying to tune beta so wheel velocities stay under 2 m/s

%setting up the equation
ri=4*0.3960*cos(2.65 * (u + 1.4));
rj=4* -0.99*sin(u + 1.4);
rk=0 * u;
r=[ri; rj; rk];

%computing linear velocity
dr=diff(r, t);
linear_velocity = norm(dr);

%computing the unit vectors, kappa, and tau for later 
T_hat_ugly=dr./norm(dr);
T_hat=simplify(T_hat_ugly);

dT_hat=diff(T_hat,t);
N_hat=dT_hat/norm(dT_hat);
N_hat=simplify(N_hat);

B_hat=cross(T_hat,N_hat);
B_hat=simplify(B_hat);

kappa=norm(dT_hat)/norm(dr);
kappa=simplify(kappa);

tau=-N_hat*(diff(B_hat,t)/norm(dr))';


%computing angular velocity
omega = cross(T_hat, diff(T_hat, t));

%computing left and right wheel velocities
V_l=simplify(linear_velocity - (omega(3) * (d / 2)))
V_r=simplify(linear_velocity + (omega(3) * (d / 2)))

%making time non-symbolic
t_num =  linspace(0, 3.2/beta, num_steps);

%finding position and unit vectors and pretty much everything else
figure
hold on
for n=1:length(t_num)
    r_num(n,:)=double(subs(r,[t],[t_num(n)])); %getting actual values for the curve
    
    T_hat_num(n,:)=double(subs(T_hat,[t],[t_num(n)])); %getting actual values for the unit tanget vector
    
    N_hat_num(n,:)=double(subs(N_hat,[t],[t_num(n)])); %getting actual values for the unit normal vector
    
    B_hat_num(n,:)=double(subs(B_hat,[t],[t_num(n)])); %getting actual values for the unit binormal vector
    
    linear_velocity_num(n,:)=double(subs(linear_velocity,[t],[t_num(n)])); %getting actual values for linear velocity
    
    omega_num(n,:)=double(subs(omega,[t],[t_num(n)])); %getting actual values for angular velocity
    
    V_l_num(n,:)=double(subs(V_l,[t],[t_num(n)])); %getting actual values for left wheel velocity
    
    V_r_num(n,:)=double(subs(V_r,[t],[t_num(n)])); %getting actual values for right wheel velocity
    
    
    
    
    
    
    
end
plot3(r_num(:,1),r_num(:,2),r_num(:,3)), axis([-10 10 -10 10 -10 10]),hold on % plot the entire curve
quiver(r_num(1:200:length(r_num),1),r_num(1:200:length(r_num),2),T_hat_num(1:200:length(r_num),1),T_hat_num(1:200:length(r_num),2),'r') % plot the unit tangent
quiver(r_num(1:200:length(r_num),1),r_num(1:200:length(r_num),2),N_hat_num(1:200:length(r_num),1),N_hat_num(1:200:length(r_num),2),'b') % plot the unit normal
xlabel('x position [m]')
ylabel('y position [m]')
title("Predicted Position")
legend("Neato Position","Predicted Unit Tangent","Predicted Unit Normal")
hold off

%Plotting predicting left and right wheel speeds
hold on
figure
plot(t_num, V_r_num, t_num, V_l_num); hold on
legend("Right Wheel", "Left Wheel"); title("Predicted Wheel Speed")
xlabel("Time [s]"); ylabel("Wheel Velocity [m/s]")
hold off;

%plotting predicted linear and angular velocity
hold on
figure
yyaxis left
plot(t_num, linear_velocity_num)
hold on
ylim([-3 3])
xlabel("Time [s]")
ylabel("Linear Velocity [m/s]")

yyaxis right
plot(t_num, omega_num)
ylabel('Angular Velocity [rad/s]')
title("Predicted Linear and Angular Velocity")
legend("Linear Velocity", "Angular Velocity")
hold off;


save predictData.mat r_num T_hat_num N_hat_num B_hat_num linear_velocity_num omega_num V_l_num V_r_num

