% Map creation for path planning
close all; clc;

% Workflow for this code:
% 1. Reading the .hgt file for alpine area below Munich 
% 2. Converting lat/lon --> Universal Transverse Mercator [UTM] (measurements for path planning)
% 3. Elevation in meters

% reading the  .hgt file with the map for Munich and Alps area
[A,R] = readgeoraster("N47E011.hgt");
A = double(A);

% grids (lat, lon)
[lat_grid, lon_grid] = geographicGrid(R);

% 'projcrs' to assign the coordinate reference system by using UTM 
utm_proj = projcrs(32632); % EPSG code for alps area below munich -- 32632

% converting to UTM (in m)
[x_UTM, y_UTM] = projfwd(utm_proj, lat_grid, lon_grid);

% normalising to grayscale
Agray = (A - min(A(:))) / (max(A(:)) - min(A(:)));
Agray = imresize(Agray, [400 400]);
X_UTM = imresize(x_UTM, size(Agray));
Y_UTM = imresize(y_UTM, size(Agray));
R = georefcells(R.LatitudeLimits, R.LongitudeLimits, size(Agray));

% Elevation map
figure;
geoshow(flipud(Agray), R, 'DisplayType', 'surface');
title('Elevation Map of Alpine area (near Munich)');
xlabel('Longitude (degrees)'); ylabel('Latitude (degrees)');
colormap("parula"); colorbar;

% UTM map
figure;
imagesc(X_UTM(1,:), Y_UTM(:,1), Agray);
axis equal tight xy;
xlabel('Easting (m)');
ylabel('Northing (m)');
title('UTM Elevation Map')
colormap("parula"); colorbar;

% binary obstacle map creation (no-go zones)
BW_elev = Agray > 0.6;

% Slope detection
[Gx, Gy] = gradient(Agray);
slope = sqrt(Gx.^2 + Gy.^2);
BW_slope = slope > 0.02;

% combine maps to detect 'obstacles'
BW = BW_elev | BW_slope;

figure;
imshow(BW);
title("Binary Obstacle Map");

% path planning cost map 
costMap = ones(size(Agray));
costMap = costMap + 10*slope; % cost for slope (steep)
costMap(BW) = inf; % for impassable zones

figure;
imagesc(X_UTM(1,:), Y_UTM(:,1), costMap)
set(gca, 'YDir', 'normal');
axis equal tight; colorbar;
xlabel('Easting (m)');
ylabel('Northing (m)');
title('Path planning Cost Map');