clear all;
close all;
clc;

Ts = 0.01;
Tmax = 8;
time_hist = 0:Ts:Tmax;

robot_num = 4;

n = length(time_hist);
x_hist = zeros(n, 2*robot_num);
x_hist(1,:) = [-0.6,0.6,-0.2,0.4,0.2,0.5,0.6,0.6];

goal = [1.2,2.5,-0.4,2.5,0.4,2.5,-1.2,2.5];
goal_agents = zeros(n, 2*robot_num);
goal_agents(1,:) = goal;

u_gtg = zeros(8,1);
u_hist = zeros(n,8);
h_hist = zeros(n,6);
dist_hist = zeros(n,6);

for k = 1:n-1
    goal_agents(k,:) = goal;
    x_k = x_hist(k,:);
    
    u_gtg = go_to_goal_control_input(x_k, goal);

    [u,h_x] = quadprogram(x_k, u_gtg);

    x_hist(k+1,:) = x_k +Ts*u';
    u_hist(k,:) = u';
    h_hist(k,:) = h_x';
    dist_hist(k,:) = (sqrt(h_x + ones(6,1)*0.2^2))';

end

t_span = time_hist;
figure();
plot(x_hist(:,1),x_hist(:,2));
grid on
hold on
plot(x_hist(:,3),x_hist(:,4));
plot(x_hist(:,5),x_hist(:,6));
plot(x_hist(:,7),x_hist(:,8));
scatter(goal(:,1),goal(:,2),40,"red",'filled')
scatter(goal(:,3),goal(:,4),40,"yellow",'filled')
scatter(goal(:,5),goal(:,6),40,"cyan",'filled')
scatter(goal(:,7),goal(:,8),40,"blue",'filled')
grid on
xlabel('x-axis');
ylabel('y-axis');
legend('agent1','agent2','agent3','agent4', ...
    'goal1','goal2','goal3','goal4');

figure();
subplot(4,1,1);
plot(t_span,goal_agents(:,1) - x_hist(:,1));
hold on
grid on
plot(t_span,goal_agents(:,2) - x_hist(:,2));
ylabel('distance');
legend('x_d1 - x1 in x-axis','x_d1 - x1 in y-axis');

subplot(4,1,2);
plot(t_span,goal_agents(:,3) - x_hist(:,3));
hold on
grid on
plot(t_span,goal_agents(:,4) - x_hist(:,4));
ylabel('distance');
legend('x_d2 - x2 in x-axis','x_d2 - x2 in y-axis');

subplot(4,1,3);
plot(t_span,goal_agents(:,5) - x_hist(:,5));
hold on
grid on
plot(t_span,goal_agents(:,6) - x_hist(:,6));
ylabel('distance');
legend('x_d3 - x3 in x-axis','x_d3 - x3 in y-axis');

subplot(4,1,4);
plot(t_span,goal_agents(:,7) - x_hist(:,7));
hold on
grid on
plot(t_span,goal_agents(:,8) - x_hist(:,8));
ylabel('distance');
legend('x_d4 - x4 in x-axis','x_d4 - x4 in y-axis');

xlabel('Time')

%-----------------------------------

figure();
subplot(3,2,1)
plot(t_span,dist_hist(:,1));
grid on
ylabel('distance');
legend('distance x1 and x2');
subplot(3,2,2)
plot(t_span,dist_hist(:,2));
grid on
ylabel('distance');
legend('distance x1 and x3');
subplot(3,2,3)
plot(t_span,dist_hist(:,3));
grid on
ylabel('distance');
legend('distance x1 and x4');
subplot(3,2,4)
plot(t_span,dist_hist(:,4));
grid on
ylabel('distance');
legend('distance x3 and x2');
subplot(3,2,5)
plot(t_span,dist_hist(:,5));
grid on
ylabel('distance');
legend('distance x4 and x2');
subplot(3,2,6)
plot(t_span,dist_hist(:,6));
grid on
ylabel('distance');
legend('distance x3 and x4');
xlabel('Time');

%---------------------------
figure();
subplot(4,1,1);
plot(t_span,u_hist(:,1));
hold on
grid on
plot(t_span,u_hist(:,2));
ylabel('control input u1');
legend('u1_x','u1_y');

subplot(4,1,2);
plot(t_span,u_hist(:,3));
hold on
grid on
plot(t_span,u_hist(:,4));
ylabel('control input u2');
legend('u2_x','u2_y');

subplot(4,1,3);
plot(t_span,u_hist(:,5));
hold on
grid on
plot(t_span,u_hist(:,6));
ylabel('control input u3');
legend('u3_x','u3_y');

subplot(4,1,4);
plot(t_span,u_hist(:,7));
hold on
grid on
plot(t_span,u_hist(:,8));
ylabel('control input u4');
legend('u4_x','u4_y');

xlabel('Time')


%---------------------------

figure();
subplot(4,1,1);
plot(t_span,x_hist(:,1));
hold on
grid on
plot(t_span,x_hist(:,2));
plot(t_span,goal_agents(:,1));
plot(t_span,goal_agents(:,2));
ylabel('distance');
legend('x1_x','x1_y','xd1_x','xd1_y' );

subplot(4,1,2);
plot(t_span,x_hist(:,3));
hold on
grid on
plot(t_span,x_hist(:,4));
plot(t_span,goal_agents(:,3));
plot(t_span,goal_agents(:,4));
ylabel('distance');
legend('x2_x','x2_y','xd2_x','xd2_y' );

subplot(4,1,3);
plot(t_span,x_hist(:,5));
hold on
grid on
plot(t_span,x_hist(:,6));
plot(t_span,goal_agents(:,5));
plot(t_span,goal_agents(:,6));
ylabel('distance');
legend('x3_x','x3_y','xd3_x','xd3_y' );

subplot(4,1,4);
plot(t_span,x_hist(:,7));
hold on
grid on
plot(t_span,x_hist(:,8));
plot(t_span,goal_agents(:,7));
plot(t_span,goal_agents(:,8));
ylabel('distance');
legend('x4_x','x4_y','xd4_x','xd4_y' );

xlabel('Time')

%-------------------


function [u,h_x]=quadprogram(x, u_gtg)
Q = 2 * eye(8); %Q 8x1
c = -2 * u_gtg'; %u_gtg 8x1

% H 6x8
H = -2*[(x(:,1)-x(:,3)),(x(:,2)-x(:,4)), (x(:,3)-x(:,1)),(x(:,4)-x(:,2)), 0, 0, 0, 0;
    (x(:,1)-x(:,5)),(x(:,2)-x(:,6)), 0, 0, (x(:,5)-x(:,1)),(x(:,6)-x(:,2)), 0, 0;
    (x(:,1)-x(:,7)),(x(:,2)-x(:,8)), 0, 0, 0, 0, (x(:,7)-x(:,1)),(x(:,8)-x(:,2));
    0, 0, (x(:,3)-x(:,5)),(x(:,4)-x(:,6)), (x(:,5)-x(:,3)),(x(:,6)-x(:,4)), 0, 0;
    0, 0, (x(:,3)-x(:,7)),(x(:,4)-x(:,8)), 0, 0, (x(:,7)-x(:,3)),(x(:,8)-x(:,4));
    0, 0, 0, 0, (x(:,5)-x(:,7)),(x(:,6)-x(:,8)), (x(:,7)-x(:,5)),(x(:,8)-x(:,6))];
% h_x 6x1
h_x = function_h(x);
b = 20*h_x.^3;
u = quadprog(Q,c,H,b);
end

function u_gtg = go_to_goal_control_input(x, x_g)
u_gtg = x_g-x;
end

function h_x = function_h(x)
h_x = zeros(6,1);
h_x(1) = (x(:,1)-x(:,3))^2 + (x(:,2)-x(:,4))^2 - 0.2^2;
h_x(2) = (x(:,1)-x(:,5))^2 + (x(:,2)-x(:,6))^2 - 0.2^2;
h_x(3) = (x(:,1)-x(:,7))^2 + (x(:,2)-x(:,8))^2 - 0.2^2;
h_x(4) = (x(:,3)-x(:,5))^2 + (x(:,4)-x(:,6))^2 - 0.2^2;
h_x(5) = (x(:,3)-x(:,7))^2 + (x(:,4)-x(:,8))^2 - 0.2^2;
h_x(6) = (x(:,5)-x(:,7))^2 + (x(:,6)-x(:,8))^2 - 0.2^2;
end


