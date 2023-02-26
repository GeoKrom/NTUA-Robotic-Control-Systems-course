%% Robot Simulation file

clear;
clc;
clf;
tic;

% Parameters of KUKA Robotic Manipulator
l0 = 0.810;
l1 = 0.200;
l2 = 0.600;
l3 = 0.030;
l4 = 0.140;
l5 = 0.550;
l6 = 0.100;
l7 = 0.100;

%% Sampling period for kinematic simulation %%
dt = 0.0001;                            %sec


%% Motion Profile - Task (or Cartesian) Space %%
tf = 10.0;                              % sec
t = 0:dt:tf;
t1 = 0:dt:20;
t2 = 10:dt:20;
kmax=tf/dt + 1;

% Trajectories 
disp("Initialising Desired Task-Space Trajectory (Motion Profile) ...");
disp(" ");

% Trajectory from point pA to pB
x1 = 0.85;% + 0.0085*t.^3 - 0.001275*t.^4 + 0.000051*t.^5; 
x1_dot = 0;%0.0255*t.^2 - 0.0051*t.^3 + 0.000255*t.^4;
y1 = 0.13 + 0.0013*t.^3 - 0.000195*t.^4 + 0.0000078*t.^5;
y1_dot = 0.0039*t.^2 - 0.00078*t.^3 + 0.000039*t.^4;
z1 = 1.55;
z1_dot = 0;

% Trajectory from point pB to pA
x2 = 0.85;% 1.7 - 0.0085*t.^3 + 0.001275*t.^4 - 0.000051*t.^5;
x2_dot = 0;%-0.0255*t.^2 + 0.0051*t.^3 - 0.000255*t.^4;
y2 = 0.26 - 0.0013*t.^3 + 0.000195*t.^4 - 0.0000078*t.^5;
y2_dot = -0.0039*t.^2 + 0.00078*t.^3 - 0.000039*t.^4;
z2 = 1.55;
z2_dot = 0;


disp("Kinematic loop has started");
disp(" ");

l = [l0, l1, l2, l3, l4, l5, l6, l7];
N = 2*numel(t)-1;
v_d = zeros(3,N);
p_d = zeros(3,N);
q_out = zeros(6,N);
q_out_dot = zeros(3,N);
Dp = zeros(3,N);
Dp_dot = zeros(3,N);
p_comp = zeros(3,N);
p_comp_dot = zeros(3,N);

% Desired Position

for i = 1:((N+1)/2)
    p_d(:,i) = [x1; y1(i); z1];
end

k = 0;
for j = ((N+1)/2+1):N
    k = k + 1;
    p_d(:,j) = [x2; y2(k); z2];
end


% Desired Velocity

for i = 1:((N+1)/2)
    v_d(:,i) = [x1_dot; y1_dot(i); z1_dot];
end

k = 0;
for j = ((N+1)/2+1):N
    k = k + 1;
    v_d(:,j) = [x2_dot; y2_dot(k); z2_dot];
end

%% Main Kinematic Loop %%

disp("Start of computation for joint and end effector position & velocity...");
disp(" ");

% Usage of Incremental Algorithm
K = 0.00001*eye(3);
i = 0;
% Compute first position and velocity

q_out(:,1) = KUKA_6DOF_Inverse_Kinematics(p_d(:,1), l);
[~, p_comp(:,1)] = KUKA_6DOF_Forward_Kinematics(q_out(:,1), l);

% Compute the rest points
for t_sim = t1
    
    % Algorithm for computing joint position and velocity as well end
    % effector position and velocity
    i = i + 1;
    if i < size(p_d,2) 
        % A different approach of the algorithm
        % Siciliano book, page 132, uses a gain parameter for position
        % error computation
        Dp_dot(:,i) = K*(p_d(:,i) - p_comp(:,i)) + v_d(:,i);
    end
    % Compute joint velocities
    [q_out_dot(:,i), ~] = KUKA_6DOF_Inverse_Diff_Kinematics(q_out(:,i), Dp_dot(:,i), l);

    if i ~= 1
        % Compute joint positions 
        q_out(:,i) = q_out(:,i-1) + [q_out_dot(:,i-1); 0; 0; 0]*dt;
    end
    % Compute end effector position and velocity
    [p_comp_dot(:,i), ~, ~] = KUKA_6DOF_Forward_Diff_Kinematics(q_out(:,i), q_out_dot(:,i), l);
    [~, p_comp(:,i)] = KUKA_6DOF_Forward_Kinematics(q_out(:,i), l);

end

disp("End of computation for joint and end effector position & velocity...");
disp(" ");

disp("Kinematic loop has ended...");
disp(" ");

%% End effector and Joint Plots %%

% KUKA_6DOF_Plots(t1, p_comp, x1, N, t, y1, t2, y2, z1, p_comp_dot, y1_dot, y2_dot, q_out, q_out_dot);

%% Robot KUKA plot %%

figure(5);
 
axis([-1 1 -0.5 0.5 0 2.0]); 
axis on;
grid on;
hold on;
xlabel("x (m)");
ylabel("y (m)");
zlabel("z (m)");
plot3(p_d(:,1),p_d(:,2),p_d(:,3),'k-');
dtk = 10000;
plot3([0],[0],[0],"g*");
line([0],[0],[l0],'Color','k');
plot3([0],[0],[l0],"g*");
%plot3([0],[0],[l0],"k-");
plot3([l1],[0],[l0],"g*");
plot3([l1],[l3],[l0],"g*");
plot3([l1],[l3],[l0+l2],"g*");
plot3([l1],[0],[l0+l2],"g*");
plot3([l1],[0],[l0+l2+l4],"g*");
plot3([l1+l5],[0],[l0+l2+l4],"g*");
plot3([l1+l5],[0],[l0+l2+l4],"g*");
plot3([l1+l5+l7],[l6],[l0+l2+l4],"g*");