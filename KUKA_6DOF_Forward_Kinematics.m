function [A0E, End_Effector_Position] = KUKA_6DOF_Forward_Kinematics(q, len)

% KUKA_6DOF_Forward_Kinematics 
% Input : Joint Position, link length 
% Output: End Effector Position,Frame of Robot

% Matrix from Frame 0 to Frame 1
A01 = [cos(q(1))  0   -sin(q(1))  len(2)*cos(q(1));
       sin(q(1))  0    cos(q(1))  len(2)*sin(q(1));
       0         -1    0          len(1);
       0          0    0          1];
% Matrix from Frame 1 to Frame 2
A12 = [ sin(q(2))  cos(q(2))   0    len(3)*sin(q(2));
       -cos(q(2))  sin(q(2))   0   -len(3)*cos(q(2));
        0          0           1    len(4);
        0          0           0    1];
% Matrix from Frame 2 to Frame 3
A23 = [cos(q(3))  0   -sin(q(3))  len(5)*cos(q(3));
       sin(q(3))  0    cos(q(3))  len(5)*sin(q(3));
       0         -1    0          0;
       0          0    0          1];
% Matrix from Frame 3 to Frame 4
A34 = [cos(q(4))  0    sin(q(4))  0;
       sin(q(4))  0   -cos(q(4))  0;
       0          1    0          len(6);
       0          0    0          1];
% Matrix from Frame 4 to Frame 5
A45 = [cos(q(5))  0   -sin(q(5))  0;
       sin(q(5))  0    cos(q(5))  0;
       0         -1    0          len(7);
       0          0    0          1];
% Matrix from Frame 5 to Frame E
A5E = [cos(q(6))  -sin(q(6))  0    0;
       sin(q(6))   cos(q(6))  0    0;
       0           0          1    len(8);
       0           0          0    1];

A0E = A01*A12*A23*A34*A45*A5E;
End_Effector_Position = A0E(1:3,4);
end