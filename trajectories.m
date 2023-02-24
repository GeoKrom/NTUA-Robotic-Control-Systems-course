clc;
clf;
clear;


t = 0:0.0001:10;

xhi1 = 0.85;% + 0.0085*t.^3 - 0.001275*t.^4 + 0.000051*t.^5; 
xhi1_dot = 0;%0.0255*t.^2 - 0.0051*t.^3 + 0.000255*t.^4;
xhi1_2dot = 0;%0.051*t - 0.0153*t.^2 + 0.00102*t.^3;

xhi2 = 0.85;%1.7 - 0.0085*t.^3 + 0.001275*t.^4 - 0.000051*t.^5; 
xhi2_dot = 0;%-0.0255*t.^2 + 0.0051*t.^3 - 0.000255*t.^4;
xhi2_2dot = 0;%-0.051*t + 0.0153*t.^2 - 0.00102*t.^3;

xhi3 = 0.13 + 0.0013*t.^3 - 0.000195*t.^4 + 0.0000078*t.^5;
xhi3_dot = 0.0039*t.^2 - 0.00078*t.^3 + 0.000039*t.^4;
xhi3_2dot = 0.0078*t - 0.00234*t.^2 + 0.000156*t.^3;

xhi4 = 0.26 - 0.0013*t.^3 + 0.000195*t.^4 - 0.0000078*t.^5;
xhi4_dot = -0.0039*t.^2 + 0.00078*t.^3 - 0.000039*t.^4;
xhi4_2dot = -0.0078*t + 0.00234*t.^2 - 0.000156*t.^3;

figure(1)
subplot(3,1,1);
title("Trajectory of x from p_A to p_B");
plot(t, xhi1*ones(1,numel(t)),'r-');
grid;
xlabel("t [sec]","FontSize",12);
ylabel("x_1 (t) [m]","FontSize",12);
legend("Position Trajectory", "Location","northwest");
subplot(3,1,2);
plot(t, xhi1_dot*ones(1,numel(t)),'r-');
grid;
xlabel("t [sec]","FontSize",12);
ylabel("v_x_1(t) [m/sec]","FontSize",12);
legend("Velocity Trajectory", "Location","northwest");
subplot(3,1,3);
plot(t, xhi1_2dot*ones(1,numel(t)),'r-');
grid;
xlabel("t [sec]","FontSize",12);
ylabel("a_x_1(t) [m/sec^2]","FontSize",12);
legend("Acceletation Trajectory", "Location","southwest");

figure(2)

subplot(3,1,1);
title("Trajectory of x from p_B to p_A");
plot(t, xhi2*ones(1,numel(t)),'r-');
grid;
xlabel("t [sec]","FontSize",12);
ylabel("x_2(t) [m]","FontSize",12);
legend("Position Trajectory", "Location","southwest");
subplot(3,1,2);
plot(t, xhi2_dot*ones(1,numel(t)),'r-');
grid;
xlabel("t [sec]","FontSize",12);
ylabel("v_x_2(t) [m/sec]","FontSize",12);
legend("Velocity Trajectory", "Location","southwest");
subplot(3,1,3);
plot(t, xhi2_2dot*ones(1,numel(t)),'r-');
grid;
xlabel("t [sec]","FontSize",12);
ylabel("a_x_2(t) [m/sec^2]","FontSize",12);
legend("Acceletation Trajectory", "Location","southeast");

figure(3)

subplot(3,1,1);
title("Trajectory of y from p_A to p_B");
plot(t, xhi3,'r-');
grid;
xlabel("t[sec]","FontSize",12);
ylabel("y_1 (t) [m]","FontSize",12);
legend("Position Trajectory", "Location","northwest");
subplot(3,1,2);
plot(t, xhi3_dot,'r-');
grid;
xlabel("t[sec]","FontSize",12);
ylabel("v_y_1 (t) [m/sec]","FontSize",12);
legend("Velocity Trajectory", "Location","southwest");
subplot(3,1,3);
plot(t, xhi3_2dot,'r-');
grid;
xlabel("t[sec]","FontSize",12);
ylabel("a_y_1 (t) [m/sec^2]","FontSize",12);
legend("Acceletation Trajectory", "Location","southeast");

figure(4)

subplot(3,1,1);
title("Trajectory of y from p_B to p_A");
plot(t, xhi4,'r-');
grid;
xlabel("t[sec]","FontSize",12);
ylabel("y_2 (t) [m]","FontSize",12);
legend("Position Trajectory", "Location","southwest");
subplot(3,1,2);
plot(t, xhi4_dot,'r-');
grid;
xlabel("t[sec]","FontSize",12);
ylabel("v_y_2 (t) [m/sec]","FontSize",12);
legend("Velocity Trajectory", "Location","southwest");
subplot(3,1,3);
plot(t, xhi4_2dot,'r-');
grid;
xlabel("t[sec]","FontSize",12);
ylabel("a_y_2 (t) [m/sec^2]","FontSize",12);
legend("Acceletation Trajectory", "Location","southeast");

