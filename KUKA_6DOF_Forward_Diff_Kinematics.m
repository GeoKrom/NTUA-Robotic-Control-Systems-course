function [v, w, J] = KUKA_6DOF_Forward_Diff_Kinematics(q, q_dot, len)

% KUKA_6DOF_Forward_Diff_Kinematics
% Input: Joint Positin, Joint Velocity, Link Length
% Output: Linear Velocity, Angular Velocity, Jacobian Matrix

%% Basic parameters for Jacobian
c1 = cos(q(1));
s1 = sin(q(1));
c2 = cos(q(2));
s2 = sin(q(2));
c3 = cos(q(3));
s3 = sin(q(3));
c23 = cos(q(2) + q(3));
s23 = sin(q(2) + q(3));
le = len(4) + len(7);
d = l(2) + l(3)*s2 + l(5)*s23 + (l(6)+l(8))*c23;
k = l(3)*c2 + l(5)*c23 - (l(6)+l(8))*s23;

%% Compute Linear Velocity Jacobian Submatrix
JL11 = -(d*s1 + le*c1);
JL12 = k*c1;
JL13 = (k - len(3)*c2);
JL21 = d*c1 - len*s1;
JL22 = k*s1;
JL23 = (k - len(3)*c2);
JL31 = 0;
JL32 = len(2) - d;
JL33 = len(2) + len(3)*s2 - k;

JL = [JL11 JL12 JL13;
      JL21 JL22 JL23;
      JL31 JL32 JL33];

%% Compute Angular Velocity Jacobian Submatrix

JA11 = 0;
JA12 = -s1;
JA13 = -s1;
JA21 = 0;
JA22 = c1;
JA23 = c1;
JA31 = 1;
JA32 = 0;
JA33 = 0;

JA = [JA11 JA12 JA13;
      JA21 JA22 JA23;
      JA31 JA32 JA33];

%% Jacobian Matrix
J = [JL; JA];

%% Linear Velocity
v = JL*q_dot;

%% Angular Velocity
w = JA*q_dot;


end