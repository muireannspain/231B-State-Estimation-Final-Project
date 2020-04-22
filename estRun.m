function [x,y,theta,internalStateOut] = estRun(time, dt, internalStateIn, steeringAngle, pedalSpeed, measurement)
x = internalStateIn.x;
y = internalStateIn.y;
theta = internalStateIn.theta;
%Pm = internalStateIn.Pm;

% wheel radius with uniform uncertainty
r = 0.40375+0.0425*rand(1,1);

% wheel base with uniform uncertainty
B = 0.72+0.16*rand(1,1);

%calculate the linear speed of the bicycle
v_linear = 5*pedalSpeed*r; 


if ~isnan(measurement(1)) & ~isnan(measurement(2))
    % have a valid measurement
    x_meas = measurement(1);
    y_meas = measurement(2);
else

    x_meas=0.99*(x+0.5*cos(theta))+0.02*(x+0.5*cos(theta))*rand(1,1);
    y_meas=0.99*(y+0.5*B*sin(theta))+0.02*(y+0.5*B*sin(theta))*rand(1,1);
end

Pm=[10 0 0;0 100 0;0 0 0.1];
%Implementation of the Extended Kalman Filter
z=[x_meas;y_meas];
%system dynamics
L=eye(3);
M=eye(2);
V=eye(3);
W=eye(2);
%Extended Kalman Filter
%initialize
XM=[x;y;theta];
v=sqrt(V)*rand(3,1);
w=sqrt(W)*rand(2,1);
%prior update 
A = [v_linear*cos(XM(3)) 0 0; 
     0 v_linear*sin(XM(3)) 0;
     0 0 (v_linear/B)*tan(steeringAngle)];
XP=[XM(1)+v_linear*cos(XM(3));
    XM(2)+v_linear*sin(XM(3));
    XM(3)+(v_linear/B)*tan(steeringAngle)];
Pp=A*Pm*(A')+L*V*L';

H=[1 0 -0.5*sin(theta);
    0 1 0.5*B*cos(theta)];
%Update Step
K=Pp*H'*inv(H*Pp*H'+M*W*M');
XM=XP+K*(z-([XP(1)+0.5*cos(XP(3)); XP(2)+0.5*B*sin(XP(3))]));
Pm=(eye(3)-K*H)*Pp;


%% OUTPUTS %%
% Update the internal state (will be passed as an argument to the function
% at next run), must obviously be compatible with the format of
% internalStateIn:

internalStateOut.x = XM(1);
internalStateOut.y = XM(2);
internalStateOut.theta = XM(3);
internalStateOut.Pm = Pm;
end

