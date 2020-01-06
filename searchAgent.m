classdef searchAgent
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Constants
        initPos = [0 0];
        initHeading = pi/2;
        range = 0.5;
        FOV = pi/2;
        velocity = 1; % m/s
        timeStep = 0.01 % s
        
        basePgon; % Origin polygon
        polyRes = 10; % Segments in arc of polygon
        
        wiggleOdds;
        
        currPos;
        currHeading;
        
    end
    
    methods
        % Constructor
        function obj = searchAgent(initPos,initHeading, range, FOV, velocity, timeStep)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            if nargin ~= 0 % Use default values if no inputs included
                obj.initPos = initPos;
                obj.initHeading = initHeading;
                obj.range = range;
                obj.FOV = FOV;
                obj.velocity = velocity;
                obj.timeStep = timeStep;
            end
            
            obj.currPos = obj.initPos;
            obj.currHeading = obj.initHeading;
            obj.wiggleOdds = 0.1 * timeStep;
            
            theta = linspace(-obj.FOV/2, obj.FOV/2, obj.polyRes);
            x = [0 obj.range.*cos(theta) 0]';
            y = [0 obj.range.*sin(theta) 0]';
            obj.basePgon = [x y];
        end
        
        % Returns position and heading of agent
        function [pos, heading] = getPose(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            pos = obj.currPos;
            heading = obj.currHeading;
        end
        
        % Returns polygon of the agent's view area in the room
        function [pgon] = getViewPoly(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            %pRot = rotate(obj.basePgon, rad2deg(obj.currHeading));
            %pgon = translate(pRot, obj.currPos);
            pRot = obj.basePgon * [cos(-obj.currHeading) -sin(-obj.currHeading); sin(-obj.currHeading) cos(-obj.currHeading)];
            %pRot = obj.basePgon * [cos(obj.currHeading) -sin(obj.currHeading); sin(obj.currHeading) cos(obj.currHeading)];
            pgon = pRot + obj.currPos;
        end
        
        % Random change heading and move forward
        function [obj, pos, heading] = wiggleMove(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            if rand() < obj.wiggleOdds % random wiggle
               obj.currHeading = rand()*2*pi; % random heading 
            end
            
            % Move forward
            obj.currPos(1) = obj.currPos(1) + obj.velocity * obj.timeStep * cos(obj.currHeading);
            obj.currPos(2) = obj.currPos(2) + obj.velocity * obj.timeStep * sin(obj.currHeading);
            %obj.currPos = obj.currPos + obj.velocity .* obj.timeStep .* [cos(obj.currHeading) sin(obj.currHeading)];
            
            pos = obj.currPos;
            heading = obj.currHeading;
        end
        
        % Bounce off wall if in wall
        function [obj, pos, heading] = wallBounce(obj, wallSide)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            if wallSide == 'T' % top or bottom
                obj.currHeading = atan2(-sin(obj.currHeading), cos(obj.currHeading));
            elseif wallSide == 'S' % side
                obj.currHeading = atan2(sin(obj.currHeading), -cos(obj.currHeading));
            end
            
            pos = obj.currPos;
            heading = obj.currHeading;
        end
        
        % Plotting function (not yet implemented)
        function [] = plotAgent(obj,ax)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            if nargin ~= 2
                ax = axes;
            end
            
            plot(ax, obj.currPos(1), obj.currPos(2), 'ro')
            
        end
    end
end

