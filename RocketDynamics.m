classdef RocketDynamics
    %VEHICLEDYNAMICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        data 
        dt double
        x(13,1) double 
    end
    
    methods
        function obj = RocketDynamics(deltaTime)
            %VEHICLEDYNAMICS Construct an instance of this class
            %   Detailed explanation goes here
            %timestep
            obj.dt = deltaTime;
            %constants
            obj.data.massB = 4.2525;
            obj.data.Inertia = diag([0.035, 4.6, 4.6]);
            obj.data.Length = 1.842;
            obj.data.RailAngle = 90.0;
            obj.data.referenceArea = 81.7*10^-4;
            obj.data.controlSurfaceArea = 34.5*10^-4;
            obj.data.nomAB_DC = 1.17;
            obj.data.airBrakePosition = 1.54;
            obj.data.maxABLength = 0.021;
            obj.data.ABonDC = 0.79;
            obj.data.ABoffDC = 0.35;
            obj.data.normalDC = 13.6;
            obj.data.dampDC = 4.88;
            obj.data.stability = 0.408;
            obj.data.CoM = [0, 0, 1.05/2]; %needs to be measured for a rocket
            
            % Dimensions of nose, body and fins (m)
            obj.data.LEN_NOSECONE = 150*10^-3;
            obj.data.ROOTCHORD_FIN = 85*10^-3;
            obj.data.TIPCHORD_FIN = 50*10^-3;
            obj.data.AVERAGECHORD_FIN = (obj.data.ROOTCHORD_FIN + obj.data.TIPCHORD_FIN)/2;
            obj.data.THICKNESS_FIN = 0.238*10^-3;
            obj.data.HEIGHT_FIN = 53*10^-3;
            obj.data.MIDCHORD_FIN = 53*10^-3;
            obj.data.SWEEPANGLE_FIN = 24*pi/180;
            obj.data.SWEEPLENGTH_FIN = 23.3*10^-3;
            obj.data.AREA_FIN = (obj.data.ROOTCHORD_FIN + obj.data.TIPCHORD_FIN)/2 * obj.data.HEIGHT_FIN * 4;
            obj.data.DIAMETER_BODY = 60*10^-3;
            obj.data.LEN_ROCKET = 1140*10^-3;
            obj.data.KINEMATIC_VISC = 1.495*10^-5;
            obj.data.AREA_FRONTBODY = pi*obj.data.DIAMETER_BODY^2/4;
            obj.data.maxABLength = 0.021; % == x, assumption for boundary layer
            obj.data.CD_0 = 1.17;
            obj.data.referenceArea = 81.7*10^-4;
            obj.data.controlSurfaceArea = 34.5*10^-4;
            
            %wind = [0,0,0]; %constant for now
            obj.data.ref_roll = [0,0,1]; %roll axis
            
            obj.data.g = [0,0,-9.81]; %gravity
            obj.data.u = 0.0;
            % var_w = 1.8*2^2*(position(3)/500)^(2/3) * (1 - 0.8 * position(3)/500)^2; %variance of wind
            % std_w = sqrt(var_w);    %standard deviation
            obj.data.wind = [0,0,2];
        end
        
        function dxdt = method1(obj)
            dxdt = rocketODE(obj.dt, obj.x, obj.data);
        end
    end
end

