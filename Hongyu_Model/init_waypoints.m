% init_waypoints.m
% choose the path：1=straight, 2="S" Shape, 3=Circle
path_type = 2;   % Change it by yourself

switch path_type
    case 1  % straight
        t = linspace(0,30,200)'; 
        x = t; 
        y = zeros(size(t));
        waypoints = [x y];

    case 2  % "S" Curve
        t = linspace(0,50,300)'; 
        x = t; 
        y = 2*sin(0.2*t);
        waypoints = [x y];

    case 3  % Circle
        theta = linspace(0,2*pi,300)'; 
        R = 5;
        waypoints = [R*cos(theta)+5, R*sin(theta)];
end
