clear all;
close all;

%% Initialize Objects
tStep = 0.1;
time = 0:tStep:300;

doPlot = true;

numRuns = 1;
numFound = zeros(1, numRuns);
numCollected = zeros(1, numRuns);

%parfor
for m = 1:numRuns
    disp(m)
    drone = searchDrone();
    human1 = searchHuman();
    human2 = searchHuman();

    room = searchRoom({drone human1 human2});
    frames = [];
    v = VideoWriter('samplevid');
    v.FrameRate = 10;
    open(v);
    for i = 1:length(time)
        %disp(time(i))
        room = room.tickTime();
        if doPlot == true %&& mod(i, 10) == 1
           room = room.plotRoom();
           title(room.ax, sprintf('Search Domain (Time: %0.1f s)', time(i)))
           %title('Original Targets', 'fontsize',16)
           writeVideo(v,getframe(room.f));
           %pause(0.01);
        end
    end
    
    found = room.foundObjects;
    found(any(isnan(found), 2), :) = [];
    
    collected = room.collectedObjects;
    collected(any(isnan(collected), 2), :) = [];
    
    numFound(m) = size(found,1) + size(collected,1);
    numCollected(m) = size(collected,1);
    
end
disp('numFound')
disp(mean(numFound))
disp('numCollected')
disp(mean(numCollected))

figure;
histogram(numFound,[-0.5:1:20.5]);
xticks([0:20])
ylabel('# Trials')
xlabel('# Objects Found')
title('Humans Only (100 Trials)')

figure;
histogram(numCollected,[-0.5:1:20.5]);
xticks([0:20])
ylabel('# Trials')
xlabel('# Objects Collected')
title('Humans Only (100 Trials)')
%disp(numFound);

close(v);
