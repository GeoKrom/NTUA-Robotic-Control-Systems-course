function [q_dot, InvJL] = KUKA_6_DOF_Inverse_Diff_Kinematics(p_dot, len)

% KUKA_6_DOF_Inverse_Diff_Kinematics
% Input: Linear Velocity, link length
% Output: Joint Velocity

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
d = l(2) + l(3)*s2 + l(5)*s23 + (l(6) + l(8))*c23;
k = l(3)*c2 + l(5)*c23 - (l(6) + l(8))*s23;

%% Inverse Linear Velocity Jacobian Submatrix

adj_JL11 = (len(5)*s3 + (len(6) + len(8))*c3)*l(3)*s1;
adj_JL12 = -(len(5)*s3 + (len(6) + len(8))*c3)*l(3)*c1;
adj_JL13 = 0;
adj_JL21 = -(d*c1 - le*s1)*(l(2) + len(3)*s2 - d);
adj_JL22 = -(d*c1 + le*s1)*(l(2) + len(3)*s2 - d);
adj_JL23 = (k - l(2)*c2)*d;
adj_JL31 = (len(2) - d)*(len(2) + len(3)*s2 - d);
adj_JL32 = (d*c1 +le*s1)*(len(2) - d);
adj_JL33 = d*k;

detJL = (len(5)*s3 + (len(6) + len(8))*c3)*len(2)*d;
InvJL = (1/detJL)*[adj_JL11 adj_JL12 adj_JL13;
                adj_JL21 adj_JL22 adj_JL23;
                adj_JL31 adj_JL32 adj_JL33];
%% Angular Velocity

q_dot = InvJL*p_dot;

end