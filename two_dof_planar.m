%% *** Robot (kinematic) model parameters *** 
clear all; 
close all; 
l(1) = 10.0;  %% in cm 
l(2) = 10.0;  

%% *** sampling period *** 
%% *** for the robot motion, kinematic simulation: 
dt = 0.001; %dt = 0.001; i.e. 1 msec)   

%% *** Create (or load from file) reference signals *** 
%% *** DESIRED MOTION PROFILE - TASK SPACE *** 
Tf=10.0; 	% 10sec duration of motion 
t=0:dt:Tf;  

%xd0,td0,yd1: initial/final end-point position --> desired task-space trajectory  
xd0 = 15.0;	
xd1 =  15.0; 
yd0 = 5.00; 
yd1 = -5.00;  

% Example of desired trajectory : linear segment (x0,y0)-->(x1,y1); Time duration: Tf; 
disp('Initialising Desired Task-Space Trajectory (Motion Profile) ...'); %% 
disp(' ');   
xd(1) = xd0; 
yd(1) = yd0; 
lambda_x = (xd1-xd0)/Tf; 
lambda_y = (yd1-yd0)/Tf; 
kmax=Tf/dt + 1; 
for k=2:kmax;    
   xd(k) = xd(k-1) + lambda_x*dt;    
   yd(k) = yd(k-1) + lambda_y*dt; 
end  
 
 
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% ****** KINEMATIC SIMULATION - Main loop ****** 
disp('Kinematic Simulation ...'); %% 
disp(' '); %%  

%% ***** INVESRE KINEMATICS  -->  DESIRED MOTION - JOINT SPACE ***** 
%% compute the reference joint-motion vectors: 
%% {qd(k,i), i=1,...,n (num of degrees of freedom), with k=1,..., kmax,} 
%% and reference joint (angular) velocities {qd_1(k,i)} 
rd2 = xd(:).^2 + yd(:).^2; 
qd(:,2) = acos( (rd2(:)-l(1)^2-l(2)^2)./(2*l(1)*l(2)) ); %% 1st solution: elbow down 	
%% or qd(:,2) = -acos( (rd2(:)-l(1)^2-l(2)^2)./(2*l(1)*l(2)) ); %% 2nd solution: elbow up 
s2 = sin(qd(:,2)); 
qd(:,1) = atan2(yd(:),xd(:)) - asin(l(2)*s2(:)./sqrt(rd2(:)));   
   
%% ***** FORWARD KINEMATICS  JOINT MOTION -->  CARTESIAN POSITIONS ***** 
%%(xd1, yd1) : cartesian position of the 1st link's local reference frame 
xd1 = l(1)*cos(qd(:,1));   
yd1 = l(1)*sin(qd(:,1)); 
%%(xd2, yd2) : cartesian position of the 2nd link's local reference frame 
xd2 = l(1)*cos( qd(:,1) ) + l(2)*cos( qd(:,1)+qd(:,2) );   
yd2 = l(1)*sin( qd(:,1) ) + l(2)*sin( qd(:,1)+qd(:,2) );  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

%% *** SAVE and PLOT output data *** %%** use functions plot(...)  
save;  %% --> save data to 'matlab.mat' file   

fig1 = figure;  
subplot(2,2,1); 
plot(t,xd); 
ylabel('xd (cm)'); 
xlabel('time t (sec)');  

subplot(2,2,2); 
plot(t,yd); 
ylabel('yd (cm)'); 
xlabel('time t (sec)');  

subplot(2,2,3); 
plot(t,qd(:,1)); 
ylabel('qd1 (rad)'); 
xlabel('time t (sec)');  

subplot(2,2,4); 
plot(t,qd(:,2)); 
ylabel('qd2 (rad)'); 
xlabel('time t (sec)');    



%%*** stick diagram --> animate robot motion ... (**optional**) 
%% within a for (or while) loop, use periodic plot(...) functions to draw the geometry (current pos)  
%% of the robot, and thus animate its motion ...  

fig2 = figure; 
axis([-5 25 -15 15]) %%set xy plot axes (caution: square axes, i.e. dx=dy) 
axis on 
hold on 
xlabel('x (cm)'); 
ylabel('y (cm)'); 
plot(xd,yd,'rs'); 
dtk=1000; %% plot robot position every dtk samples, to animate its motion 
plot([0],[0],'o'); 
for tk=1:dtk:kmax,    %%% 	
   pause(0.1);	%% pause motion to view successive robot configurations    
   plot([0,xd1(tk)],[0,yd1(tk)]);					
   plot([xd1(tk)],[yd1(tk)],'o');    
   plot([xd1(tk),xd2(tk)],[yd1(tk),yd2(tk)]);	
   plot([xd2(tk)],[yd2(tk)],'y*');    
   plot([xd(tk)],[yd(tk)],'g+');  
end       
