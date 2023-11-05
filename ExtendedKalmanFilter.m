% Written by Nikilesh Ramesh - 02/11/2023

% state = [\lambda offset_vector] where \lambda = [roll {\phi}, pitch {\theta}, yaw {\psi}]
function [state, P] = ExtendedKalmanFilter(p_state, p_cov, data)

% data
omega  = data.gyro; %needs to be column
[i,~] = size(state); % YOU SURE???
%state space calc
sr = sin(p_state(1)); %sin roll
tp = tan(p_state(2)); %tan pitch
cr = cos(p_state(1)); %cos roll
cp = cos(p_state(2)); %cos pitch

A = [-1, -sr.*tp, -cr.*tp; 0, -cr, sr; 0 -sr./cp ];
B = -A;

%state transition
Z3 = zero(3,3);
stM_r1 = [Z3 A];
stM_r2 = [Z3 Z3];
At = vertcat(stM_r1, stM_r2);
Bt = vertcat(B,Z3);

%predict
state_dt = At*p_state + Bt*omega + K*(meas - p_meas); %WHAT IS THIS SHIT
P = At*p_cov*At' + Q;
%update
K = P*H'\(H*P*H' + R);
P = (eye(i) - K*H)*P;

%
state = 0;
P = 0;
end