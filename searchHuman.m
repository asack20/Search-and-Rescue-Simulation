classdef searchHuman
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Constants
        initPos = [66 24];
        initHeading = pi/2;
        viewRange = 10;
        FOV = pi;
        pickupRange = 1;
        velocity = 2; % ft/s
        timeStep = 0.1; % s
        
        endurance = 80;
        objectsCarried = 0;
        
        tPick = 1;
        pickTimer;
        
        type = 'human';
        
        circleTime = NaN;
        
        state = 'sleep';
        
        basePgon; % Origin polygon
        polyRes = 30; % Segments in arc of polygon
        
        detectOdds = 0.075; %0.15 drone)
        
        
        wiggleOdds;
        path = [];
        currPos;
        currHeading;
        
        
    end
    
    methods
        % Constructor
        function obj = searchHuman()
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes her
            
            obj.currPos = obj.initPos;
            obj.currHeading = obj.initHeading;
            
            theta = linspace(-obj.FOV/2, obj.FOV/2, obj.polyRes);
            x = [0 obj.viewRange.*cos(theta) 0]';
            y = [0 obj.viewRange.*sin(theta) 0]';
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
            pRot = obj.basePgon * [cos(-obj.currHeading) -sin(-obj.currHeading); sin(-obj.currHeading) cos(-obj.currHeading)];
            pgon = pRot + obj.currPos;
        end
        
        function [obj] = spinCircle(obj)
            obj.state = 'spin';
            if isnan(obj.circleTime)
                obj.currHeading = 0;
                obj.circleTime = 0;
            end
            
            obj.circleTime = obj.circleTime + obj.timeStep;
            
            %disp(obj.circleTime);    
            if mod(round(obj.circleTime,3), 3) == 0
                obj.currHeading = obj.currHeading + pi/3;
            end
            if mod(round(obj.circleTime,3), 18) == 0
                obj.state = 'idle';
            end
        
                
        end
        
        function [obj, pos, heading] = moveToTarget(obj, targX, targY)
            %Calculate Heading
            obj.currHeading = atan2(targY-obj.currPos(2), targX - obj.currPos(1));
            
            if obj.dist2Targ(targX, targY) < obj.velocity * obj.timeStep
               obj.currPos(1) = targX;
               obj.currPos(2) = targY;
            
            else % Move forward
            obj.currPos(1) = obj.currPos(1) + obj.velocity * obj.timeStep * cos(obj.currHeading);
            obj.currPos(2) = obj.currPos(2) + obj.velocity * obj.timeStep * sin(obj.currHeading);
            end
            
            pos = obj.currPos;
            heading = obj.currHeading;
            
        end
        
        
        function [obj] = pathMove(obj, env)
            obj.state = 'straight';
            obj.pickTimer = 0;
            obj.endurance = obj.endurance - obj.timeStep * env;
            if obj.endurance <= 0
                obj.state = 'sleep';
                return
            end
            
            if size(obj.path, 1) > 0
                targX = obj.path(1,1);
                targY = obj.path(1,2);
                oldPos = obj.currPos;
                
                obj.currHeading = atan2(targY-obj.currPos(2), targX - obj.currPos(1));
                obj.currPos(1) = obj.currPos(1) + obj.velocity * obj.timeStep * cos(obj.currHeading);
                obj.currPos(2) = obj.currPos(2) + obj.velocity * obj.timeStep * sin(obj.currHeading);
                
                if sqrt((obj.currPos(1)-oldPos(1)).^2 + (obj.currPos(2)-oldPos(2)).^2) > sqrt((targX-oldPos(1)).^2 + (targY-oldPos(2)).^2)
                    obj.path(1,:) = [];
                end
                
                % Avoid going out of bounds
                if obj.currPos(1) >= 80
                    obj.currPos(1) = 79.9;
                end
                if obj.currPos(2) >= 32
                    obj.currPos(2) = 31.9;
                end
                if obj.currPos(1) <= 0
                    obj.currPos(1) = 0.1;
                end
                if obj.currPos(2) <= 0
                    obj.currPos(2) = 0.1;
                end
                
                
            else
                obj.state = 'idle';
            end     
        end
        
        function [obj] = collectObject(obj, env)
            obj.state = 'collecting';
            obj.endurance = obj.endurance - obj.timeStep * env;
            if obj.endurance <= 0
                obj.state = 'sleep';
                return
            end
            
            obj.pickTimer = obj.pickTimer + obj.timeStep;
            
            if obj.pickTimer >= obj.tPick
                obj.state = 'idle';
                obj.objectsCarried = obj.objectsCarried + 1;
                obj.pickTimer = 0;
            end
        end
        
        function dist = dist2Targ(obj, targX, targY)
            dist = sqrt((obj.currPos(1)-targX).^2 + (obj.currPos(2)-targY).^2);
        end
        
    end
end

