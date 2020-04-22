function [internalState, studentNames, estimatorType] = estInitialize


internalState.x = -0.5+rand(1,1);
internalState.y = -0.5+rand(1,1);
internalState.theta = pi/2; % rad (heading approximately North East)
%internalState.Pm=[0.01 0 0;0 0.01 0;0 0 0.0000001];

studentNames = ['Muireann Spain']
estimatorType = 'EKF'

end


