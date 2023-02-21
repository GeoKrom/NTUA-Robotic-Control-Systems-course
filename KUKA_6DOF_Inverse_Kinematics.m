function q = KUKA_6DOF_Inverse_Kinematics(p, len)

format short
% KUKA 6DOF Inverse Kinematics 
% Input : End Effector Position 
% Output: Joint Position


le = len(4) + len(7);
d = sqrt(p(1).^2 + p(2).^2 - le.^2);
m0 = ((p(3)-len(1)).^2 + (d-len(2)).^2 - len(3).^2 - len(5).^2 - (len(6)+len(8)).^2)/(2*len(3));
q1 = atan2(d*p(2) - le*p(1), (d*p(2) + le*p(1)));
a = sqrt(len(5).^2 + (len(6)+len(8)).^2 - m0.^2)
q3 = atan2(sqrt(len(5).^2 + (len(6)+len(8)).^2 - m0.^2), m0) - atan2(len(6)+len(8), len(5));
q2 = atan2(d - len(2), p(3) - len(1)) - atan2(len(5)*sin(q3)+ (len(6)+len(8))*cos(q3), len(3) + len(5)*cos(q3) - (len(6)+len(8))*sin(q3));
q4 = 0;
q5 = 0;
q6 = 0;

q = [q1 q2 q3 q4 q5 q6]';
end