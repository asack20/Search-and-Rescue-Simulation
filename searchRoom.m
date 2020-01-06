classdef searchRoom
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %% Room Basics
        roomSize = [80 32];
        entrance = [66 24];
        tStep = 0.1;
        time = 0;
        
        
        %% Plot Handles
        f;
        ax;
        leg;
        
        ptHidden;
        ptFound;
        ptCollected;
        agentPoly;
        
        %% Fire
        v = 0.2; % ft/s
        sigma = 0.3; % ft
        dt = 1; % s
        xw = -10;
        xc = -10;
        flame;
        pFlame;
        
        %% Object Locations
        hiddenObjects = [45.7803796456933,5.77892393310043;
                        45.2053727821743,18.3458094826425;
                        65.9053253781087,5.23411700428971;
                        10.0879061986065,28.9936555384163;
                        24.0093422027008,2.47496912426626;
                        0.169761986656845,10.8331088045336;
                        76.0885886312218,18.5797615167083;
                        61.3039490665597,13.8075250397529;
                        60.1043329750520,25.7702555592170;
                        11.1091869712748,16.9849005111938;
                        27.9455634114100,7.27393061255321;
                        12.1072448397982,22.7035192062874;
                        39.7376542606339,4.75624170255520;
                        64.6921802329752,21.0597177991483;
                        50.6295034752917,20.2874415324830;
                        55.0721198154279,7.33780004690469;
                        51.1656174284205,5.83129648872647;
                        58.3457380994854,5.32328024756641;
                        68.7876566468296,4.78742693179972;
                        50.1564303531659,6.48791672776312];
%         hiddenObjects = [69.0867925825275,3.03924906838765;
%                         22.7419438754570,12.9177849837097;
%                         14.5907413280259,12.1820800710445;
%                         54.1638503596225,15.5940899693858;
%                         22.1542915900196,23.3264965504957;
%                         40.0643262949536,8.57423874813615;
%                         25.7334860648572,20.5188586412552;
%                         72.2607654650159,21.9132240240488;
%                         38.2055190017531,6.08407597671706;
%                         26.6981558412336,10.0536411808289;
%                         43.0799396003368,3.95978010859057;
%                         4.37682643531176,19.7274281606053;
%                         52.0260110569021,9.66460486631715;
%                         58.3259126595606,11.5158173807743;
%                         43.3932158474879,20.6520026656335;
%                         15.3825080726320,2.20164042804969;
%                         7.56546242129419,25.0387511477330;
%                         12.5974921894367,25.8624019931699;
%                         2.64235422136762,24.2800648103728;
%                         34.0897227661749,2.90909585424665];
                    
        hideLevel = [0.359,0.499,0.781,0.712,0.985,0.511,0.298,0.897,0.834,...
           0.883,0.862,0.987,0.459,0.397,0.351,0.217,0.413,0.413,0.933,0.587];
        
%         hideLevel = [0.782,0.884,0.631,0.221,0.870,0.612,0.549,0.494,0.709,0.642,...
%             0.239,0.761,0.742,0.241,0.572,0.809,0.319,0.687,0.608,0.956];
         
        objectTracker;
        viewedObjects = nan(20,2);
        foundObjects = nan(20,2);
        collectedObjects = nan(20,2);
        savedObjects = nan(20,2);
        
        currCollecting = [NaN NaN NaN];
        
        
        searchAgents; % Agent 1 is a drone, 2 and 3 are Humans
        beamTarg = [13 16];
        beamChecked = false;
        beamTop;
        beamBottom;
        
        
        %% Polygon Room Definition
        xBorder = [0 80 80 64 64 56 56 16 16 0  0];
        yBorder = [0 0  24 24 32 32 24 24 32 32 0];
        
        xTallObst = [16,48,48,16,16,NaN,48,50,50,48,48,NaN,48,50,50,48,48];
        yTallObst = [16,16,18,18,16,NaN,0,0,6,6,0,NaN,10,10,18,18,10]
        
        xShortObst = [58,64,64,58,58,NaN,9,12,12,9,9,NaN,9,12,12,9,9];
        yShortObst = [14,14,19,19,14,NaN,5,5,15,15,5,NaN,18,18,24,24,18];
        
        xGround;
        yGround;
        pGround;
        
        xAir;
        yAir;
        pAirRaw;
        pAir;
        
        targetList = [[1:2:9]', ones(5,1); [9:-2:1]', ones(5,1)*3; [1:2:9]', ones(5,1)*2; [9:-2:1]', ones(5,1)*4];
        targetNum = [0 21];
        addVal = [1 -1];
        areaGraph;
        
    end
    
    methods
        %% constructor
        function obj = searchRoom(searchAgents, hiddenObjects)
            obj.searchAgents = searchAgents;
            if nargin ~= 1 % Use default values if no inputs included
                obj.hiddenObjects = hiddenObjects;
            end
            
            % Create human searcher graph 
            obj.areaGraph.ground = ones(obj.roomSize);
            
            for i = 1:obj.roomSize(1)
                for j = 1:obj.roomSize(2)
                    if ~inpolygon(i-0.5, j-0.5, obj.xBorder, obj.yBorder) || inpolygon(i-0.5, j-0.5, obj.xTallObst, obj.yTallObst) || inpolygon(i-0.5, j-0.5, obj.xShortObst, obj.yShortObst)
                        obj.areaGraph.ground(i,j) = 0;
                    end
                end
            end
            
            obj.areaGraph.ground(15,:) = zeros(1, 32);
            
            % Create drone graph
            obj.areaGraph.air = ones(obj.roomSize);
            
            for i = 1:obj.roomSize(1)
                for j = 1:obj.roomSize(2)
                    if ~inpolygon(i-0.5, j-0.5, obj.xBorder, obj.yBorder) || inpolygon(i-0.5, j-0.5, obj.xTallObst, obj.yTallObst)
                        obj.areaGraph.air(i,j) = 0;
                    end
                end
            end
            
            obj.objectTracker = zeros(size(obj.hiddenObjects,1), size(searchAgents,2));
            
            obj.f = figure;
            obj.ax = axes(obj.f);
            hold(obj.ax, 'on')

            ps = polyshape(obj.xBorder, obj.yBorder);
            pTall = polyshape(obj.xTallObst, obj.yTallObst);
            pShort = polyshape(obj.xShortObst, obj.yShortObst);
            
            obj.xGround = [obj.xBorder NaN obj.xTallObst NaN obj.xShortObst];
            obj.yGround = [obj.yBorder NaN obj.yTallObst NaN obj.yShortObst];
            obj.pGround= polyshape(obj.xGround, obj.yGround);
            
            obj.xAir = [obj.xBorder NaN obj.xTallObst];
            obj.yAir = [obj.yBorder NaN obj.yTallObst];
            obj.pAirRaw = polyshape(obj.xAir, obj.yAir);
            obj.pAir = obj.pAirRaw;
            
            %% Plot Prep
            
            plot(obj.ax, ps,'FaceColor','w', 'FaceAlpha', 1,'LineWidth',2) % polygon
            plot(obj.ax, pTall)
            plot(obj.ax, pShort)
            axis equal
            set(obj.ax, 'YDir','reverse')
            set(obj.ax, 'Color',[0.8 0.8 0.8])
            xlim([-0.5 80.5])
            ylim([-0.5 32.5])
            title('Search Domain')
            xlabel('x (ft)')
            ylabel('y (ft)')    
            
            xFlame = [-11 obj.xw obj.xc -11 -11];
            yFlame = [0 0 16 16 0];
            obj.pFlame = polyshape(xFlame, yFlame);
            obj.flame = plot(obj.pFlame,'FaceColor',[0.8500 0.3250 0.0980], 'FaceAlpha', 0.8,'LineWidth',2);
            
            %Plot beams
            obj.beamTop = plot(obj.ax, [16 16], [0 16], 'y-', 'LineWidth',2);
            obj.beamBottom = plot(obj.ax, [16 16], [16 32], 'y-', 'LineWidth',2);
            
            obj.ptHidden = plot(obj.ax, obj.hiddenObjects(:,1), obj.hiddenObjects(:,2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize' , 10);
            obj.ptFound = plot(obj.ax, obj.foundObjects(:,1), obj.foundObjects(:,2), 'yo', 'MarkerFaceColor', 'y', 'MarkerSize' , 10);
            obj.ptCollected = plot(obj.ax, obj.collectedObjects(:,1), obj.collectedObjects(:,2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize' , 10);
            
            colors = ['m', 'c', 'c'];
            
            for i = 1:length(obj.searchAgents)
                [pgon] = obj.searchAgents{i}.getViewPoly();
                %[pos, ~] = obj.searchAgents{i}.getPose();
                %plot(pos(1), pos(2), 'd', 'MarkerFaceColor', colors(i));
                obj.agentPoly{i} = plot(polyshape(pgon), 'FaceColor', colors(i));
            end
            obj.leg = legend([obj.ptHidden obj.ptFound obj.ptCollected obj.agentPoly{1} obj.agentPoly{2} obj.agentPoly{3}], {},'Location', 'southoutside', 'Orientation', 'vertical', 'NumColumns', 2);
        end
        %%
        function obj = tickTime(obj)
            obj.time = obj.time + obj.tStep;
            %disp(obj.time)
            % Update Fire at 1 Hz
            if mod(obj.time, 1) < 0.1 % mod is acting funny
                obj = obj.updateFire();
            end
              
            %% Agent
            
            for i = 1:length(obj.searchAgents) % For each agent
                
                % Stagger human start time
                if i == 2 && round(obj.time,1) == 140
                    obj.searchAgents{i}.state = 'idle';
                elseif i==3 && round(obj.time,1) == 220
                    obj.searchAgents{i}.state = 'idle';
                end
                
                if strcmp(obj.searchAgents{i}.type, 'drone') && inpolygon(obj.searchAgents{i}.currPos(1), obj.searchAgents{i}.currPos(1), obj.pFlame.Vertices(:,1), obj.pFlame.Vertices(:,2))
                    obj.searchAgents{i}.state = 'sleep';
                end
                
                if strcmp(obj.searchAgents{i}.state, 'sleep')
                    continue
                end
                %break;
                % Determine next Target
                if strcmp(obj.searchAgents{i}.state, 'idle') % start a new path
                    if ~isnan(obj.currCollecting(i)) && obj.searchAgents{i}.dist2Targ(obj.foundObjects(obj.currCollecting(i),1), obj.foundObjects(obj.currCollecting(i),2)) <= 2
                        obj.searchAgents{i}.state = 'collecting';
                    else
                        [obj, path] = obj.managePath(obj.searchAgents{i}, i);
                        obj.searchAgents{i}.state = 'straight';
                        obj.searchAgents{i}.path = path;   
                    end
                end
                
                % Check if in flame for endurance multiplier
                if inpolygon(obj.searchAgents{i}.currPos(1), obj.searchAgents{i}.currPos(1), obj.pFlame.Vertices(:,1), obj.pFlame.Vertices(:,2))
                    env = 2;
                else
                    env = 1;
                end
                    
                
                % Path already in process 
                if strcmp(obj.searchAgents{i}.state, 'straight')
                    % make sure drone doesn't go in fire
                    if strcmp(obj.searchAgents{i}.type, 'drone') && inpolygon(obj.searchAgents{i}.currPos(1), obj.searchAgents{i}.currPos(1), obj.pFlame.Vertices(:,1)+3, obj.pFlame.Vertices(:,2))
                        obj.searchAgents{i}.state = 'idle';
                        obj.searchAgents{i}.path = [];
                    else
                        obj.searchAgents{i} = obj.searchAgents{i}.pathMove(env);
                    end
                elseif strcmp(obj.searchAgents{i}.state, 'collecting')
                    obj.searchAgents{i} = obj.searchAgents{i}.collectObject(env);
                    if strcmp(obj.searchAgents{i}.state, 'idle')
                        obj.collectedObjects(obj.currCollecting(i),:) = obj.foundObjects(obj.currCollecting(i),:);
                        obj.foundObjects(obj.currCollecting(i),:) = [NaN NaN];
                        obj.currCollecting(i) = NaN;
                    end    
                end
                    
                % Check if object found
                pgon = obj.searchAgents{i}.getViewPoly();
                
                %Check for beam
                if ~obj.beamChecked && strcmp(obj.searchAgents{i}.type, 'drone')
                    in = inpolygon(obj.beamTarg(1),obj.beamTarg(2),pgon(:,1),pgon(:,2)); 
                    if in
                        obj.beamChecked = true;
                        if rand < 0.5 % pick a side
                            %disp('Top Safe')
                            obj.beamTop.Color = 'g';
                            obj.beamBottom.Color = 'r';
                            obj.areaGraph.ground(15,1:16) = ones(1, 16);
                        else
                            %disp('Bottom Safe')
                            obj.beamTop.Color = 'r';
                            obj.beamBottom.Color = 'g';
                            obj.areaGraph.ground(15,17:32) = ones(1, 16);
                        end
                    end
                end
                
                % Drop off objects at entrance
                if strcmp(obj.searchAgents{i}.type, 'human') && obj.searchAgents{i}.dist2Targ(obj.entrance(1), obj.entrance(2)) <= 1
                    obj.searchAgents{i}.objectsCarried = 0;
                end
                
                for j = 1:length(obj.hiddenObjects)
                    % Convert Hidden to Found
                    in = inpolygon(obj.hiddenObjects(j,1),obj.hiddenObjects(j,2),pgon(:,1),pgon(:,2)); 
                    if in
                        obj.viewedObjects(j,:) = obj.hiddenObjects(j,:);
                        if rand() < obj.searchAgents{i}.detectOdds * obj.hideLevel(j)
                            obj.foundObjects(j,:) = obj.hiddenObjects(j,:);
                            obj.hiddenObjects(j,:) = [NaN NaN];
                        end
                    end
                    
                    % Collect Found
                    
                end
            end
            
            
        end
        
        function [obj, path] = managePath(obj, agent, i)
            % Select correct map
            if strcmp(agent.type, 'drone')
                map = obj.pAir;
            else
                map = obj.pGround;
            end
            
            
            [ind, mindist] = obj.closestObject(agent);
            if strcmp(agent.type, 'drone') && ~obj.beamChecked % Send drone to check beam first
                targX = obj.beamTarg(1);
                targY = obj.beamTarg(2);
            % If drone near fire, head to entrance
            elseif strcmp(agent.type, 'drone') && inpolygon(agent.currPos(1), agent.currPos(1), obj.pFlame.Vertices(:,1)+5, obj.pFlame.Vertices(:,2))
                targX = obj.entrance(1);
                targY = obj.entrance(2);
            elseif isnan(mindist) || strcmp(agent.type, 'drone')
                [targX, targY] = obj.randTarg(map);  
            elseif strcmp(agent.type, 'human') && agent.objectsCarried >= 5
                targX = obj.entrance(1);
                targY = obj.entrance(2);
            else
                targX = obj.foundObjects(ind,1);
                targY = obj.foundObjects(ind,2);
                obj.currCollecting(i) = ind;
            end

            % Gen Path to Target
            startX = agent.currPos(1);
            startY = agent.currPos(2);
                        
            % Check if straight line is an option before doing A*
            if obj.validLine(map, startX, startY, targX, targY)
                path = [targX targY];
                return
            end
                
            % Otherwise use A* to start Path
            path = [];
            count = 0;
            while isempty(path) % ensure a path is generated
                count = count + 1;
                if strcmp(agent.type, 'drone')
                    path = createPathGraph(obj, obj.areaGraph.air, targX, targY,startX, startY);
                else 
                    path = createPathGraph(obj, obj.areaGraph.ground, targX, targY,startX, startY);
                end

                if isempty(path)
                    if count > 10
                        path = [];
                        return
                    end
                    if strcmp(agent.type, 'drone') && inpolygon(agent.currPos(1), agent.currPos(1), obj.pFlame.Vertices(:,1)+5, obj.pFlame.Vertices(:,2))
                        targX = obj.entrance(1);
                        targY = obj.entrance(2);
                    else
                        [targX, targY] = obj.randTarg(map);  
                    end
                end
            end
            
            begX = startX;
            begY = startY;
            m = 0;
            n = size(path, 1);
            while n > m
                if obj.validLine(map, begX, begY, path(n,1), path(n,2))
                    m = m+1;
                    path(m:n-1, :) = [];
                    n = size(path, 1);
                    begX = path(m,1);
                    begY = path(m,2);
                else
                    n = n-1;
                end
                    
            
            end
        end
        
        %%
        function valid = validLine(obj, map, startX, startY, endX, endY)
            [in, out] = intersect(map, [startX startY; endX endY]);
            if isempty(out)
                valid = true;
            else
                valid = false;
            end
        end
                
        
        %%
        function obj = updateFire(obj)
            % Calc Fire Movement
            obj.xw = obj.xw + obj.v * obj.dt + randn()*obj.sigma;
            obj.xc = obj.xc + obj.v * obj.dt + randn()*obj.sigma;
            
            % Update Graphs
            flameInterp = interp1([0 16], [obj.xw obj.xc], 1:32,'linear', 'extrap');
            for i = 1:32
                if ceil(flameInterp(i)) > 0
                    obj.areaGraph.ground(ceil(flameInterp(i)),i) = 2;
                    obj.areaGraph.air(ceil(flameInterp(i)),i) = 0;
                end
            end
            flameEnd = interp1([0 16], [obj.xw obj.xc], 32,'linear', 'extrap');
            % Update Polygon
            xFlame = [-11 obj.xw flameEnd -11 -11];
            yFlame = [0 0 32 32 0];
            obj.pFlame = polyshape(xFlame, yFlame);
            obj.pAir = subtract(obj.pAirRaw, polyshape(xFlame+3, yFlame));
            %figure(2);
            %plot(obj.pAir);
            
        end
        
        %%
        function obj = plotRoom(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            %hold on
            
            %hold(obj.ax, 'on')
            
            % Flame
            obj.flame.Shape = obj.pFlame;
            
            % Objects
            obj.ptHidden.XData = obj.hiddenObjects(:,1);
            obj.ptHidden.YData = obj.hiddenObjects(:,2);
            obj.ptFound.XData = obj.foundObjects(:,1);
            obj.ptFound.YData = obj.foundObjects(:,2);
            obj.ptCollected.XData = obj.collectedObjects(:,1);
            obj.ptCollected.YData = obj.collectedObjects(:,2);
            
            % Agents
            for i = 1:length(obj.searchAgents)
                if strcmp(obj.searchAgents{i}.state, 'sleep')
                    obj.agentPoly{i}.Visible = 'off';
                else
                    obj.agentPoly{i}.Visible = 'on';
                end
                [pgon] = obj.searchAgents{i}.getViewPoly();
                obj.agentPoly{i}.Shape = pgon;
                
            end
            
            % Update Legend
            sH = sprintf('Hidden Objects: %d/20', size(obj.hiddenObjects(all(~isnan(obj.hiddenObjects),2),:),1));
            sF = sprintf('Found Objects: %d/20', size(obj.foundObjects(all(~isnan(obj.foundObjects),2),:),1));
            sC = sprintf('Collected Objects: %d/20', size(obj.collectedObjects(all(~isnan(obj.collectedObjects),2),:),1));
            
            s1 = 'Drone (Endurance: N/A)';
            s2 = sprintf('Human 1 (Endurance: %0.1f)',obj.searchAgents{2}.endurance);
            s3 = sprintf('Human 2 (Endurance: %0.1f)',obj.searchAgents{3}.endurance);
            
            obj.leg.String = {sH, sF, sC, s1, s2, s3};
            %hold(obj.ax, 'off')
        end
        %%
        function [path] = createPathGraph(obj, areaGraph, targX, targY, startX, startY)
            
            % Calc dist to end for all points
            rSize = obj.roomSize;
            R_dist = zeros(rSize);
            for i = 1:size(areaGraph, 1)
                for j = 1:size(areaGraph, 2)
                    R_dist(i, j) = 1 * (abs(ceil(targX) - i) + abs(ceil(targY) - j));
                end
            end
            
            % Initialize Structure
            pathGraph.weight = areaGraph;
            pathGraph.dist = R_dist;
            pathGraph.minDist = NaN(rSize); % Initialize distances
            pathGraph.expectedDist = NaN(rSize);
            pathGraph.prevNode = NaN(rSize(1),rSize(2), 2); % row col
            pathGraph.isExplored = false(rSize);
            
            % A* Algorithm
            currRow = ceil(startX);
            currCol = ceil(startY);
            
            endRow = ceil(targX);
            endCol = ceil(targY);
            
            pathGraph.minDist(currRow, currCol) = 0;

            x_list = NaN(rSize(1)*rSize(2),1);
            y_list = NaN(size(x_list));
            count = 1;

            while ~(currRow == endRow && currCol == endCol)
               %fprintf('r: %d\tc: %d\n', currRow, currCol);
               y_list(count) = currCol;
               x_list(count) = currRow;
               count = count + 1;
               pathGraph.isExplored(currRow, currCol) = true;
               N = getNeighbors(pathGraph, currRow, currCol);
               for i = 1:size(N,1) % Look at each neighbor
                   if isnan(pathGraph.prevNode(N(i,1),N(i,2))) % if neighbor node is not explored
                       edge = pathGraph.weight(N(i,1), N(i,2));
                       newDist = pathGraph.minDist(currRow, currCol) + edge;  % calc real dist to that point

                       if  isnan(pathGraph.minDist(N(i,1), N(i,2))) ||  newDist < pathGraph.minDist(N(i,1), N(i,2)) %found shorter path
                           pathGraph.minDist(N(i,1), N(i,2)) = newDist; % update distance
                           pathGraph.prevNode(N(i,1), N(i,2),:) = [currRow, currCol]; % update path
                           pathGraph.expectedDist(N(i,1),N(i,2))= newDist + pathGraph.dist(N(i,1),N(i,2)); %add expected total dist to array
                       end
                   end

               end
               minVal = min(pathGraph.expectedDist, [], 'all', 'omitnan');
               [minRow, minCol] = find(pathGraph.expectedDist == minVal);

               %k = randi(length(minRow));
             
               if isempty(minRow) % Path is impossible
                   path = [];
                   return;
               end
               
               currRow = minRow(length(minRow));
               currCol = minCol(length(minRow));
               pathGraph.expectedDist(currRow, currCol) = NaN;
            end

            y_list(count) = currCol;
            x_list(count) = currRow;
            
            
            currRow = endRow;
            currCol = endCol;

            ypath = currCol;
            xpath = currRow;

            while ~isnan(pathGraph.prevNode(currRow,currCol,1))
                xpath = [xpath pathGraph.prevNode(currRow,currCol,1)];
                ypath = [ypath pathGraph.prevNode(currRow,currCol,2)];

                nextRow = pathGraph.prevNode(currRow,currCol,1);
                nextCol = pathGraph.prevNode(currRow,currCol,2);

                currRow = nextRow;
                currCol = nextCol;
            end
            
            path = flip([xpath' ypath']);
        
        end
        
        %%
        function [ind, mindist] = closestObject(obj, agent)
            dist = nan(20,1);
            
            for i = 1:size(obj.foundObjects, 1)
               if ~isnan(obj.foundObjects(i,1))
                  dist(i) =  sqrt((agent.currPos(1)-obj.foundObjects(i,1)).^2 + (agent.currPos(2)-obj.foundObjects(i,2)).^2);
               end
            end
            
            [mindist, ind] = min(dist);
            
        end
        
        %%
        function [targX, targY] = randTarg(obj, mapPoly)
            badTarget = true;
            
            while badTarget 
                targX = rand() * obj.roomSize(1);
                targY = rand() * obj.roomSize(2);
                
                badTarget = ~inpolygon(targX, targY, mapPoly.Vertices(:,1), mapPoly.Vertices(:,2));
                
                if ~obj.beamChecked && targX <= 16 % Can't get to far side before beam checked
                    badTarget = true;
                elseif targX > 46 && targX < 49 && targY > 10 && targY < 17
                    badTarget = true;
                end
                 
            end
            
        end

    end
end
