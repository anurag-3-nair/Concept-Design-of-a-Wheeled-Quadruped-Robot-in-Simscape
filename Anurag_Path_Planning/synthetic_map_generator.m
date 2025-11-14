% Synthetic map creation for path planning
close all; clc;

% Workflow for this code:
% 1. Generating a synthetic map 
% 2. Defining a local coordinate grid (in 'm')
% 3. Obstacle detection
% 4. Cost map creation for A* path-planning algo

% creating synthetic map 
n = 400; %grid size
L = 1000; 
x = linspace(0, L, n); y = linspace(0, L, n);
[X, Y] = meshgrid(linspace(0,L,n), linspace(0,L,n));

% terrain (random peaks)
Agray = 0.2 + 0.02*rand(size(X));

% rectangle objects
BW = false(size(Agray));
rects = [200 200 150 100; 600 600 120 180; 300 700 200 100];

for i = 1:size(rects,1)
    x1 = rects(i,1); y1 = rects(i,2);
    w = rects(i,3); h = rects(i,4);
    BW(Y > y1 & Y < y1+h & X > x1 & X < x1+w) = true;
end

% Circle objects
circles = [500 300 80; 750 200 60];
for i = 1:size(circles,1)
    cx = circles(i,1); cy = circles(i,2); r = circles(i,3);
    BW((X-cx).^2 + (Y-cy).^2 < r^2) = true;
end

% elevation for objects
Agray(BW) = Agray(BW) + 0.5;
Agray = (Agray - min(Agray(:))) / (max(Agray(:)) - min(Agray(:))); % normalise

% plotting the synthetic map
figure;
imagesc(x, y, Agray);
%set(gca, 'YDir', 'normal');
axis equal tight xy;
xlabel('X (m)'); ylabel('Y (m)');
title('Synthetic Elevation Map')
colormap("parula"); colorbar;

% binary obstacle map
figure;
imshow(BW);
title("Binary Obstacle Map");

% path planning cost map 
costMap = ones(size(Agray));
costMap(BW) = inf; % for impassable zones
costMap = costMap + 3*Agray;

figure;
imagesc(x, y, costMap)
set(gca, 'YDir', 'normal');
axis equal tight; colorbar;
xlabel('Easting (m)');
ylabel('Northing (m)');
title('Path planning Cost Map');