%% Robot Simulation file
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

%% Trajectories %%
disp("Initialising Desired Task-Space Trajectory (Motion Profile) ...");
disp(" ");
% Trajectory from point pA to pB
x1 = 0.85;% + 0.0085*t.^3 - 0.001275*t.^4 + 0.000051*t.^5; 
x1_dot = 0.0255*t.^2 - 0.0051*t.^3 + 0.000255*t.^4;
y1 = 0.13 + 0.0013*t.^3 - 0.000195*t.^4 + 0.0000078*t.^5;
y1_dot = 0.0039*t.^2 - 0.00078*t.^3 + 0.000039*t.^4;
z1 = 1.55;
z1_dot = 0;
% Trajectory from point pB to pA
x2 = 1.7 - 0.0085*t.^3 + 0.001275*t.^4 - 0.000051*t.^5;
x2_dot = -0.0255*t.^2 + 0.0051*t.^3 - 0.000255*t.^4;
y2 = 0.26 - 0.0013*t.^3 + 0.000195*t.^4 - 0.0000078*t.^5;
y2_dot = -0.0039*t.^2 + 0.00078*t.^3 - 0.000039*t.^4;
z2 = 1.55;
z2_dot = 0;

%% Main Kinematic Loop %%
disp("Kinematic loop has started");
disp(" ");
l = [l0, l1, l2, l3, l4, l5, l6, l7];
N = numel(t);
p_d = [x1*ones(1,N); y1; z1*ones(1,N)];
q_out = zeros(6,N);
q_out_dot = zeros(6,N);
p_comp = zeros(3,N);
p_comp_dot = zeros(3,N);
i = 0;


for t_sim = t
   
    i = i + 1;
    q_out(:,i) = KUKA_6DOF_Inverse_Kinematics(p_d(:,i),l);
    %q_out_dot(:,i) = (q_out(:,i+1) - q_out(:,i))/dt;

end
i = 0;
for t_sim = t
      i = i + 1;
      [~, p_comp(:,i)] = KUKA_6DOF_Forward_Kinematics(q_out(:,i),l);

end


%% Plots
figure(1);
subplot(3,1,1);
plot(t, p_comp(1,:), 'b-');
grid;
hold on;
plot(t, x1*ones(1,N), 'r-.');
xlabel("time [sec]");
ylabel("x(t) [m]");
legend('Computed', 'Desired', 'Location', 'Southwest');
hold off;

subplot(3,1,2);
plot(t, p_comp(2,:), 'b-');
grid;
hold on;
plot(t, y1, 'r-.');
xlabel("time [sec]");
ylabel("y(t) [m]");
legend('Computed', 'Desired', 'Location', 'Northwest');
hold off;

subplot(3,1,3);
plot(t, p_comp(3,:), 'b-');
grid;
hold on;
plot(t, z1*ones(1,N), 'r-.');
xlabel("time [sec]");
ylabel("z(t) [m]");
legend('Computed', 'Desired', 'Location', 'Northwest');
hold off;

figure(2);
subplot(3,1,1);
plot(t, q_out(1,:), 'r-');
grid;
xlabel("time [sec]");
ylabel("q_1(t) [rad]");

subplot(3,1,2);
plot(t, q_out(2,:), 'r-');
grid;
xlabel("time [sec]");
ylabel("q_2(t) [rad]");

subplot(3,1,3);
plot(t, q_out(3,:), 'r-');
grid;
xlabel("time [sec]");
ylabel("q_3(t) [rad]");