% Written by Nikilesh Ramesh - 02/11/2023

% state = [\lambda offset_vector] where \lambda = [roll {\phi}, pitch {\theta}, yaw {\psi}]
function [state, cov] = ExtendedKalmanFilter(p_state, p_cov, data)

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
stM = vertcat(stM_r1, stM_r2);

%
state = 0;
cov = 0;
end