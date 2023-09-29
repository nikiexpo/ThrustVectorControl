
%constants
massB = 4.2525;
Inertia = diag([0.035, 4.6, 4.6]);
Length = 1.842;
RailAngle = 90.0;
referenceArea = 81.7*10^-4;
controlSurfaceArea = 34.5*10^-4;
nomAB_DC = 1.17;
airBrakePosition = 1.54;
maxABLength = 0.021;
ABonDC = 0.79;
ABoffDC = 0.35;
normalDC = 13.6;
dampDC = 4.88;
stability = 0.408;
CoM = [0, 0, 1.05/2]; %needs to be measured for a rocket

% Dimensions of nose, body and fins (m)
LEN_NOSECONE = 150*10^-3;
ROOTCHORD_FIN = 85*10^-3;
TIPCHORD_FIN = 50*10^-3;
AVERAGECHORD_FIN = (ROOTCHORD_FIN + TIPCHORD_FIN)/2;
THICKNESS_FIN = 0.238*10^-3;
HEIGHT_FIN = 53*10^-3;
MIDCHORD_FIN = 53*10^-3;
SWEEPANGLE_FIN = 24*pi/180;
SWEEPLENGTH_FIN = 23.3*10^-3;
AREA_FIN = (ROOTCHORD_FIN + TIPCHORD_FIN)/2 * HEIGHT_FIN * 4;
DIAMETER_BODY = 60*10^-3;
LEN_ROCKET = 1140*10^-3;
KINEMATIC_VISC = 1.495*10^-5;
AREA_FRONTBODY = pi*DIAMETER_BODY^2/4;
maxABLength = 0.021; % == x, assumption for boundary layer
CD_0 = 1.17;
referenceArea = 81.7*10^-4;
controlSurfaceArea = 34.5*10^-4;

%wind = [0,0,0]; %constant for now
ref_roll = [0,0,1]; %roll axis

g = [0,0,-9.81]; %gravity
u = 0.0;
% var_w = 1.8*2^2*(position(3)/500)^(2/3) * (1 - 0.8 * position(3)/500)^2; %variance of wind
% std_w = sqrt(var_w);    %standard deviation
wind = [0,0,2];