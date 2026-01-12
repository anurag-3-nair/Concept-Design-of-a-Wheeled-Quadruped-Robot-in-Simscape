% Control function to determine angle of inclination of robo torso

function [Sh_front_angle, Sh_back_angle, Si_left_angle, Si_right_angle, c]= change_of_inclination_fcn(z_heights, Length_torso, Width_torso, grid_step, x,y,Sh_front_angle_before,Sh_back_angle_before,Si_left_angle_before,Si_right_angle_before,Control)
 
Sh_temp_front = 0;
Sh_temp_back = 0;
Si_temp_left = 0;
Si_temp_right = 0;
speed_of_change = 0;
 
c = 0;
 
if Control == 1 || Control == 2 % front -
    
    x_check_max = x + Length_torso;
    x_check_min = x;
 
    %c = x_check_max; % check x-coord.
 
    if x_check_min < 0
        x_check_min = 0;
    end
    if x_check_max > size(z_heights,1)
        x_check_max = size(z_heights,1);
    end
    %c = x_check_min;
    x_node_max = 1 + ceil(x_check_max/grid_step);
    x_node_min = 1 + ceil(x_check_min/grid_step);
    y_node = 1 + round(y / grid_step);
 
    %c = x_node_max;
 
    for id = x_node_max:-1:x_node_min
        
        if id == 1
            inclination_infront = (z_heights(id+1, y_node) - z_heights(id,y_node)) / grid_step;
            inclination_behind = inclination_infront;
        elseif id == size(z_heights,1)
            inclination_behind = (z_heights(id, y_node) - z_heights(id-1,y_node)) / grid_step;
            inclination_infront = inclination_behind;
        else
            inclination_infront = (z_heights(id+1, y_node) - z_heights(id,y_node)) / grid_step;
            inclination_behind = (z_heights(id, y_node) - z_heights(id-1,y_node)) / grid_step;
        end
        
        %c = inclination_behind;
        %c = inclination_infront;
        c = inclination_behind - inclination_infront;
        if abs(inclination_behind - inclination_infront) <= 1e-6
            Sh_temp_front = 0;
            speed_of_change = 0;
        elseif inclination_infront <= inclination_behind
            Sh_temp_front = -0;%.1 * tan(inclination_infront-inclination_behind) * 360 /(2 * pi);
            speed_of_change = 0;
            break
        else
            Sh_temp_front = -tan(inclination_infront-inclination_behind) * 360 /(2 * pi);
            speed_of_change = 1;
            break
        end
    end
    
    %c = Sh_temp_front;
    %x_check = x + Length_torso/2;
    %x_node = 1 + ceil(x_check / grid_step);
    %y_node = 1 + round(y / grid_step);
 
    %inclination_infront = (z_heights(x_node+1, y_node) - z_heights(x_node,y_node)) / grid_step;
    %inclination_behind = (z_heights(x_node, y_node) - z_heights(x_node-1,y_node)) / grid_step;
 
    %if inclination_behind == inclination_infront
    %    Sh_temp_front = 0;
    %else
    %    Sh_temp_front = -tan(inclination_infront);
    %end
 
elseif Control == 3 % back
    
    x_check_max = x;
    x_check_min = x - Length_torso;
 
    %c = x_check_max; % check x-coord.
 
    if x_check_min < 0
        x_check_min = 0;
    end
    if x_check_max > size(z_heights,1)
        x_check_max = size(z_heights,1);
    end
 
    x_node_max = 1 + floor(x_check_max/grid_step);
    x_node_min = 1 + floor(x_check_min/grid_step);
    y_node = 1 + round(y / grid_step);
 
    %c = x_node_max;
 
    for id = x_node_min:1:x_node_max
        
        if id == 1
            inclination_infront = (z_heights(id+1, y_node) - z_heights(id,y_node)) / grid_step;
            inclination_behind = inclination_infront;
        elseif id == size(z_heights,1)
            inclination_behind = (z_heights(id, y_node) - z_heights(id-1,y_node)) / grid_step;
            inclination_infront = inclination_behind;
        else
            inclination_infront = (z_heights(id+1, y_node) - z_heights(id,y_node)) / grid_step;
            inclination_behind = (z_heights(id, y_node) - z_heights(id-1,y_node)) / grid_step;
        end
        
        %c = inclination_infront;
 
        if abs(inclination_behind - inclination_infront) <= 1e-6
            Sh_temp_back = 0;
        elseif inclination_infront <= inclination_behind
            Sh_temp_back = -0;%.1 * tan(inclination_behind-inclination_infront) * 360 /(2 * pi);
            break           
        else
            Sh_temp_back = -tan(inclination_behind-inclination_infront) * 360 /(2 * pi);            
            break
        end
    end
 
elseif Control == 4 % left +    % needed to test
 
    y_check_max = y + 2 * Width_torso;
    y_check_min = y;
 
    %c = x_check_max; % check x-coord.
 
    if y_check_min < 0
        y_check_min = 0;
    end
    if y_check_max > size(z_heights,2) 
        y_check_max = size(z_heights,2);
    end
   
    y_node_max = 1 + ceil(y_check_max/grid_step);
    y_node_min = 1 + ceil(y_check_min/grid_step);
    x_node = 1 + round(x / grid_step);
    
    for id = y_node_max:-1:y_node_min
        
        if id == 1
            inclination_infront = (z_heights(x_node, id + 1) - z_heights(x_node,id)) / grid_step;
            inclination_behind = inclination_infront;
        elseif id == size(z_heights,2)
            inclination_behind = (z_heights(x_node, id) - z_heights(x_node,id-1)) / grid_step;
            inclination_infront = inclination_behind;
        else
            inclination_infront = (z_heights(x_node, id + 1) - z_heights(x_node,id)) / grid_step;
            inclination_behind = (z_heights(x_node, id) - z_heights(x_node,id-1)) / grid_step;
        end
        
        %c = inclination_infront;
 
        if abs(inclination_infront - inclination_behind) <= 1e-6
            Si_temp_left = 0;
           
        else
            Si_temp_left = tan(inclination_infront) * 360 /(2 * pi);
            
            break
        end
    end
   
elseif Control == 5 % right
 
 
    y_check_max = y;
    y_check_min = y - 2 * Width_torso;
 
    %c = x_check_max; % check x-coord.
 
    if y_check_min < 0
        y_check_min = 0;
    end
    if y_check_max > size(z_heights,2) 
        y_check_max = size(z_heights,2);
    end
   
    y_node_max = 1 + floor(y_check_max/grid_step);
    y_node_min = 1 + floor(y_check_min/grid_step);
    x_node = 1 + round(x / grid_step);
    
    for id = y_node_min:1:y_node_max
        
        if id == 1
            inclination_infront = (z_heights(x_node, id + 1) - z_heights(x_node,id)) / grid_step;
            inclination_behind = inclination_infront;
        elseif id == size(z_heights,2)
            inclination_behind = (z_heights(x_node, id) - z_heights(x_node,id-1)) / grid_step;
            inclination_infront = inclination_behind;
        else
            inclination_infront = (z_heights(x_node, id + 1) - z_heights(x_node,id)) / grid_step;
            inclination_behind = (z_heights(x_node, id) - z_heights(x_node,id-1)) / grid_step;
        end
        
        %c = inclination_infront;
 
        if abs(inclination_infront - inclination_behind) <= 1e-6
            Si_temp_right = 0;
           
        else
            if inclination_infront <= inclination_behind
                Si_temp_right = 0;
            else
                Si_temp_right = tan(inclination_infront) * 360 /(2 * pi);
            end
            break
        end
    end    
end
 
if abs(Sh_temp_front - Sh_front_angle_before) <= 1
    Sh_front_angle = 0;   
elseif speed_of_change == 1
    Sh_front_angle = Sh_front_angle_before + (Sh_temp_front - Sh_front_angle_before) / 10;
else
    Sh_front_angle = Sh_front_angle_before + (Sh_temp_front - Sh_front_angle_before) / 15;
end
 
if abs(Sh_temp_back - Sh_back_angle_before) <= 1
    Sh_back_angle = 0;   
elseif speed_of_change == 1
    Sh_back_angle = Sh_back_angle_before + (Sh_temp_back - Sh_back_angle_before) / 10;
else
    Sh_back_angle = Sh_back_angle_before + (Sh_temp_back - Sh_back_angle_before) / 15;
end
 
if abs(Si_temp_left - Si_left_angle_before) <= 1
    Si_left_angle = 0;    
elseif speed_of_change == 1
    Si_left_angle = Si_left_angle_before + (Si_temp_left - Si_left_angle_before) / 10;
else
    Si_left_angle = Si_left_angle_before + (Si_temp_left - Si_left_angle_before) / 15;
end
 
if abs(Si_temp_right - Si_right_angle_before) <= 1
    Si_right_angle = 0;    
elseif speed_of_change == 1
    Si_right_angle = Si_right_angle_before + (Si_temp_right - Si_right_angle_before) / 10;
else
    Si_right_angle = Si_right_angle_before + (Si_temp_right - Si_right_angle_before) / 15;
end
end