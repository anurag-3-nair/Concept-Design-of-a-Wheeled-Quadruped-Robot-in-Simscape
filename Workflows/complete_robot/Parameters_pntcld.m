% Grid Surface Ball Parameters Initialization

% Copyright 2023 The MathWorks, Inc.

%% Ball %%

% Ball.Radius                   = 0.01;                     % m
% Ball.Density                  = 1000;                    % kg/m^3
% Ball.PointCloud.MarkerRadius  = 5e-4*1.5;                % m
% pointDensity = 15;
% [X,Y,Z] = sphere(round(pointDensity));
% Ball.sphere_ptcld = unique([reshape(X*Ball.Radius,[],1) reshape(Y*Ball.Radius,[],1) reshape(Z*Ball.Radius,[],1)],'rows');

goldenAngle = pi * (3 - sqrt(5));
numPoints = 200;
points = zeros(numPoints, 3);
Ball.Radius = 0.01;                     % m
Ball.Density = 1000;                    % kg/m^3

for i = 0:numPoints-1
    theta = goldenAngle * i;
    z = (2 * i / numPoints) - 1;
    rZ = sqrt(1 - z^2);

    x = rZ * cos(theta);
    y = rZ * sin(theta);

    points(i+1,:) = Ball.Radius * [x, y, z];
end


 Ball.PointCloud.MarkerRadius  = 5e-4*1.5;                % m
% pointDensity = 15;
% [X,Y,Z] = sphere(round(pointDensity));
% Ball.sphere_ptcld = unique([reshape(X*Ball.Radius,[],1) reshape(Y*Ball.Radius,[],1) reshape(Z*Ball.Radius,[],1)],'rows');
Ball.sphere_ptcld = unique([points(:,1), points(:,2), points(:,3)], 'rows');


%% Leg %%
t_leg =0.005;                                    % m
L = 0.047;                                       % m
legLength = t_leg;  
legWidth = t_leg;   
legHeight = L;   
Leg.PointCloud.MarkerRadius  = 5e-4*1.5;          % m

% Create X, Y, Z Grid Coordinates 
[x, y, z] = ndgrid(linspace(-legLength/2, legLength/2, 10), ...
                   linspace(-legWidth/2, legWidth/2, 5), ...
                   linspace(0, legHeight, 15));


leg_PointCloud = unique([x(:), y(:), z(:)], 'rows');

%% Bottle %%
Length_torso = 0.2;
Width_torso = 0.1;
height_bottle = Width_torso * 0.5;
radius_bottle = Length_torso * 0.07;
density_bottle = 100;
bottle_point_density = 15;

% [X,Y] = circle(round(bottle_point_density));
% theta = linspace(0,2*pi,bottle_point_density);
theta = 0:0.3:2*pi;
% x_top =radius_bottle*cos(theta);
% y_top = radius_bottle*sin(theta);
% z_top = height_bottle/2*ones(size(theta));

x_top = [radius_bottle*cos(theta), radius_bottle*cos(theta)];
y_top = [radius_bottle*sin(theta), radius_bottle*sin(theta)];
z_top = [-(height_bottle/2)*ones(1,size(x_top,2)/2), (height_bottle/2)*ones(1,size(x_top,2)/2)];
% z_top = height_bottle/2*ones(size(theta));

% x_bottom =radius_bottle*cos(theta);
% y_bottom = radius_bottle*sin(theta);
% z_bottom = -height_bottle/2*ones(size(theta));

% Create X, Y, Z Grid Coordinates 
[x, y, z] = ndgrid(x_top, ...
                   y_top, ...
                   linspace(0, height_bottle, 2));


% Bottle_PointCloud = transpose(unique([x_top, y_top, z_top;x_bottom,y_bottom,z_bottom],'rows'));

Bottle_PointCloud = unique([x_top(:), y_top(:), z_top(:)], 'rows');

%% Occupancy Grid %%

grid_step = 0.2;

x_grid_vector = 0:grid_step:10;
y_grid_vector = 0:grid_step:5;

z_heights = zeros(length(x_grid_vector), length(y_grid_vector));

for i = 1:1:5
    z_heights(i,:) = 0;
end

for i = 5:1:9
    z_heights(i+1,:) = z_heights(i,:); % + 0.02;
end

for i = 10:1:39
    z_heights(i+1,:) = z_heights(i,:);
end

for i = 40:1:44
    z_heights(i+1,:) = z_heights(i,:); % - 0.02;
end

for i = 46:1:51
    z_heights(i,:) = 0;
end





% Wall Data
Wall_Height = 0.5;
Wall_Length = 0.01;
Wall_Width = 0.8;



% Wall #1
Wall1.index = [14,12];
Wall1.position = [x_grid_vector(Wall1.index(1)), y_grid_vector(Wall1.index(2)), z_heights(Wall1.index(1), Wall1.index(2))];
Wall1.size = [Wall_Width, Wall_Length, Wall_Height]; 


% Wall #3
Wall3.index = [24, 18];
Wall3.position = [x_grid_vector(Wall3.index(1)), y_grid_vector(Wall3.index(2)), z_heights(Wall3.index(1), Wall3.index(2))];
Wall3.size = [Wall_Width, Wall_Length, Wall_Height];


% Compute the angle between Wall 1 & 3
delta_x = (Wall3.position(1)-Wall3.size(1)/2) - (Wall1.position(1) + Wall1.size(1)/2) ;
delta_y = y_grid_vector(Wall3.index(2)) - y_grid_vector(Wall1.index(2));
theta = atan2(delta_y, delta_x);  
theta_val1 = theta;

% Wall #2
Wall2.index = [(Wall1.index(1)+Wall3.index(1))/2, (Wall1.index(2)+Wall3.index(2))/2];
Wall2.position = [x_grid_vector(Wall2.index(1)), y_grid_vector(Wall2.index(2)), z_heights(Wall2.index(1), Wall2.index(2))];
Wall2.size = [sqrt((delta_x)^2 + (delta_y)^2), Wall_Length, Wall_Height];


% Wall #4
Wall4.index = [14,6];
Wall4.position = [x_grid_vector(Wall4.index(1)), y_grid_vector(Wall4.index(2)), z_heights(Wall4.index(1), Wall4.index(2))];
Wall4.size = [Wall_Width, Wall_Length, Wall_Height];


% Wall #6
Wall6.index = [24, 12];
Wall6.position = [x_grid_vector(Wall6.index(1)), y_grid_vector(Wall6.index(2)), z_heights(Wall6.index(1), Wall6.index(2))];
Wall6.size = [Wall_Width, Wall_Length, Wall_Height];

% Compute the angle between Wall 4 & 6
delta_x_2 = (Wall6.position(1)-Wall6.size(1)/2) - (Wall4.position(1) + Wall4.size(1)/2) ;
delta_y_2 = y_grid_vector(Wall6.index(2)) - y_grid_vector(Wall4.index(2));
theta_2 = atan2(delta_y_2, delta_x_2);  
theta_val2 = theta_2;

% Wall #5
Wall5.index = [(Wall4.index(1)+Wall6.index(1))/2, (Wall4.index(2)+Wall6.index(2))/2];
Wall5.position = [x_grid_vector(Wall5.index(1)), y_grid_vector(Wall5.index(2)), z_heights(Wall5.index(1), Wall5.index(2))];
Wall5.size = [sqrt((delta_x_2)^2 + (delta_y_2)^2), Wall_Length, Wall_Height];

% Wall #8
Wall8.index = [32, 16];
Wall8.position = [x_grid_vector(Wall8.index(1)), y_grid_vector(Wall8.index(2)), z_heights(Wall8.index(1), Wall8.index(2))];
Wall8.size = [Wall_Width, Wall_Length, Wall_Height];

% Compute the angle between Wall 3 & 8
delta_x_3 = (Wall8.position(1)-Wall8.size(1)/2) - (Wall3.position(1) + Wall3.size(1)/2) ;
delta_y_3 = y_grid_vector(Wall8.index(2)) - y_grid_vector(Wall3.index(2));
theta_3 = atan2(delta_y_3, delta_x_3);  
theta_val3 = theta_3;

% Wall #7
Wall7.index = [(Wall3.index(1)+Wall8.index(1))/2, (Wall3.index(2)+Wall8.index(2))/2 ];
Wall7.position = [x_grid_vector(Wall7.index(1)), y_grid_vector(Wall7.index(2)), z_heights(Wall7.index(1), Wall7.index(2))];
Wall7.size = [sqrt((delta_x_3)^2 + (delta_y_3)^2), Wall_Length, Wall_Height];

% Wall #10
Wall10.index = [32, 10];
Wall10.position = [x_grid_vector(Wall10.index(1)), y_grid_vector(Wall10.index(2)), z_heights(Wall10.index(1), Wall10.index(2))];
Wall10.size = [Wall_Width, Wall_Length, Wall_Height];

% Compute the angle between Wall 6 & 10
delta_x_4 = (Wall10.position(1)-Wall10.size(1)/2) - (Wall6.position(1) + Wall6.size(1)/2) ;
delta_y_4 = y_grid_vector(Wall10.index(2)) - y_grid_vector(Wall6.index(2));
theta_4 = atan2(delta_y_4, delta_x_4);  
theta_val4 = theta_4;

% Wall #9
Wall9.index = [(Wall6.index(1)+Wall10.index(1))/2, (Wall6.index(2)+Wall10.index(2))/2];
Wall9.position = [x_grid_vector(Wall9.index(1)), y_grid_vector(Wall9.index(2)), z_heights(Wall9.index(1), Wall9.index(2))];
Wall9.size = [sqrt((delta_x_4)^2 + (delta_y_4)^2), Wall_Length, Wall_Height];








occupancy = zeros(length(x_grid_vector), length(y_grid_vector));

occupancy(((Wall1.index(1) - ((Wall_Width / 2) / grid_step)):(Wall1.index(1) + (Wall_Width / 2) / grid_step)), Wall1.index(2)) = 1; % wall 1 transform to occupancy grid 
occupancy(((Wall3.index(1) - ((Wall_Width / 2) / grid_step)):(Wall3.index(1) + (Wall_Width / 2) / grid_step)), Wall3.index(2)) = 1; % wall 3 transform to occupancy grid
occupancy(((Wall4.index(1) - ((Wall_Width / 2) / grid_step)):(Wall4.index(1) + (Wall_Width / 2) / grid_step)), Wall4.index(2)) = 1; % wall 4 transform to occupancy grid 
occupancy(((Wall6.index(1) - ((Wall_Width / 2) / grid_step)):(Wall6.index(1) + (Wall_Width / 2) / grid_step)), Wall6.index(2)) = 1; % wall 6 transform to occupancy grid
occupancy(((Wall8.index(1) - ((Wall_Width / 2) / grid_step)):(Wall8.index(1) + (Wall_Width / 2) / grid_step)), Wall8.index(2)) = 1; % wall 8 transform to occupancy grid
occupancy(((Wall10.index(1) - ((Wall_Width / 2) / grid_step)):(Wall10.index(1) + (Wall_Width / 2) / grid_step)), Wall10.index(2)) = 1; % wall 10 transform to occupancy grid

n_x = delta_x / grid_step;
n_y = delta_y / grid_step;
n_x_2 = delta_x_2 / grid_step;
n_y_2 = delta_y_2 / grid_step;
n_x_3 = delta_x_3 / grid_step;
n_y_3 = abs(delta_y_3 / grid_step);
n_x_4 = delta_x_4 / grid_step;
n_y_4 = abs(delta_y_4 / grid_step);


v = n_y / n_x;
v_2 = n_y_2 / n_x_2;
v_3 = n_y_3 / n_x_3;
v_4 = n_y_4 / n_x_4;

% wall 2 transform to occupancy grid
for i = 0:n_y
    occupancy(((Wall1.index(1) + ((Wall_Width / 2) / grid_step) + round((1/v) * i)):(Wall1.index(1) + (Wall_Width / 2) / grid_step) + round((1/v) * (i + 1))), Wall1.index(2) + i) = 1;

    occupancy(((Wall1.index(1) + ((Wall_Width / 2) / grid_step) + round((1/v) * i)):(Wall1.index(1) + (Wall_Width / 2) / grid_step) + round((1/v) * (i + 1))), Wall1.index(2) + (i + 1)) = 1;
end
% wall 5 transform to occupancy grid
for i = 0:n_y
    occupancy(((Wall4.index(1) + ((Wall_Width / 2) / grid_step) + round((1/v_2) * i)):(Wall4.index(1) + (Wall_Width / 2) / grid_step) + round((1/v_2) * (i + 1))), Wall4.index(2) + i) = 1;

    occupancy(((Wall4.index(1) + ((Wall_Width / 2) / grid_step) + round((1/v_2) * i)):(Wall4.index(1) + (Wall_Width / 2) / grid_step) + round((1/v_2) * (i + 1))), Wall4.index(2) + (i + 1)) = 1;
end

% wall 7 transform to occupancy grid
for i = 0:n_y_3
    occupancy(((Wall3.index(1) + ((Wall_Width / 2) / grid_step) + round((1/v_3) * i)):(Wall3.index(1) + (Wall_Width / 2) / grid_step) + round((1/v_3) * (i + 1))), Wall3.index(2) + i) = 1;

    occupancy(((Wall3.index(1) + ((Wall_Width / 2) / grid_step) + round((1/v_3) * i)):(Wall3.index(1) + (Wall_Width / 2) / grid_step) + round((1/v_3) * (i + 1))), Wall3.index(2) + (i + 1)) = 1;
end


% wall 9 transform to occupancy grid

for i = 0:n_y_4
    occupancy(((Wall6.index(1) + ((Wall_Width / 2) / grid_step) + round((1/v_4) * i)):(Wall6.index(1) + (Wall_Width / 2) / grid_step) + round((1/v_4) * (i + 1))), Wall6.index(2) + i) = 1;

    occupancy(((Wall6.index(1) + ((Wall_Width / 2) / grid_step) + round((1/v_4) * i)):(Wall6.index(1) + (Wall_Width / 2) / grid_step) + round((1/v_4) * (i + 1))), Wall6.index(2) + (i + 1)) = 1;
end

% % Bump Data
% Bump_Height = 0.02;
% Bump_Length = 0.3;
% Bump_Width = 0.4;
% 
% %Bump 1
% Bump1.index = [35, 13];
% Bump1.position = [x_grid_vector(Bump1.index(1)), y_grid_vector(Bump1.index(2)), z_heights(Bump1.index(1), Bump1.index(2))];
% Bump1.size = [Bump_Width, Bump_Length, Bump_Height]; 
% 
% %Bump 2
% Bump2.index = [36, 13];
% Bump2.position = [x_grid_vector(Bump2.index(1)), y_grid_vector(Bump2.index(2)), z_heights(Bump2.index(1), Bump2.index(2))];
% Bump2.size = [Bump_Width, Bump_Length, Bump_Height]; 
% 
% %Bump 3
% Bump3.index = [37, 13];
% Bump3.position = [x_grid_vector(Bump3.index(1)), y_grid_vector(Bump3.index(2)), z_heights(Bump3.index(1), Bump3.index(2))];
% Bump3.size = [Bump_Width, Bump_Length, Bump_Height]; 
% 
% % occupancy((Bump1.index(1) - 1):(Bump1.index(1) + 1), Bump1.index(2)) = 1;
% % occupancy((Bump2.index(1) - 1):(Bump2.index(1) + 1), Bump2.index(2)) = 1;
% % occupancy((Bump3.index(1) - 1):(Bump3.index(1) + 1), Bump3.index(2)) = 1;