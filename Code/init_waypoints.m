% init_waypoints.m
% choose the path：1=straight, 2="S" Shape, 3=Circle
path_type = 1;   % Change it by yourself

switch path_type
    case 1  % straight
        t = linspace(0,100,400)'; 
        x = t; 
        y = zeros(size(t));
        waypoints = [x y];

    case 2  % "S" Curve
        t = linspace(0,100,400)'; 
        x = t; 
        y = 15*sin(0.05*t);
        waypoints = [x y];

    case 3  % Circle
        theta = linspace(0,2*pi,300)'; 
        R = 5;
        waypoints = [R*cos(theta)+5, R*sin(theta)];
end
