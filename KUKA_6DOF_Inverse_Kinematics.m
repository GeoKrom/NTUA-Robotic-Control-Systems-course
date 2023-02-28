function q = KUKA_6DOF_Inverse_Kinematics(p, len)


% KUKA 6DOF Inverse Kinematics 
% Input : End Effector Position 
% Output: Joint Position

% Parameters for computing joint position
le = len(4) + len(7);                                                                               % l = l3 + l6
d = sqrt(p(1).^2 + p(2).^2 - le.^2);
m0 = ((p(3)-len(1)).^2 + (d-len(2)).^2 - len(3).^2 - len(5).^2 - (len(6)+len(8)).^2)/(2*len(3));

% q1 joint computation, Two Solutions
  q1 = atan2(d*p(2) - le*p(1), d*p(1) + le*p(2)); % Positive solution 
% q1 = atan2((-d)*p(2) - le*p(1), (-d)*p(1) + le*p(2)); % Negative Solution
% a = sqrt(len(5).^2 + (len(6)+len(8)).^2 - m0.^2);

% q3 joint computation, Two Solutions Total
% q3 = atan2(sqrt(len(5).^2 + (len(6)+len(8)).^2 - m0.^2), m0) - atan2(len(6)+len(8), len(5)); % Positive Solution
  q3 = atan2(sqrt(len(5).^2 + (len(6)+len(8)).^2 - m0.^2), m0) - atan2(len(6)+len(8), len(5)); % Negative Solution

% q2 joint computation, Four Solutions Total
% q2 = atan2(d - len(2), p(3) - len(1)) - atan2(len(5)*sin(q3)+ (len(6)+len(8))*cos(q3), len(3) + len(5)*cos(q3) - (len(6)+len(8))*sin(q3)); % Double Positive Solution
  q2 = atan2(d - len(2), p(3) - len(1)) - atan2(len(5)*sin(-q3)+ (len(6)+len(8))*cos(-q3), len(3) + len(5)*cos(-q3) - (len(6)+len(8))*sin(-q3)); % Positive Negative Solution
% q2 = atan2((-d) - len(2), p(3) - len(1)) - atan2(len(5)*sin(q3)+ (len(6)+len(8))*cos(q3), len(3) + len(5)*cos(q3) - (len(6)+len(8))*sin(q3)); % Negative Positive Solution
% q2 = atan2((-d) - len(2), p(3) - len(1)) - atan2(len(5)*sin(-q3)+ (len(6)+len(8))*cos(-q3), len(3) + len(5)*cos(-q3) - (len(6)+len(8))*sin(-q3)); % Double Negative Solution


% End Effector joints are shutdown 
q4 = 0;
q5 = 0;
q6 = 0;

q = [q1 q2 q3 q4 q5 q6]';

end