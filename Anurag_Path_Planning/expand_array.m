% To return an expanded array
function exp_array = expand_array(node_x, node_y, gn_current, xTarget, yTarget, CLOSED, MAX_X, MAX_Y, MAP)
    exp_array = [];
    exp_count = 1;
    c2 = size(CLOSED, 1);%Number of elements in CLOSED including the zeros

    % 8-connected neighborhood (including diagonals)
    for k = 1:-1:-1
        for j = 1:-1:-1
            % Skip the current node itself
            if (k ~= 0 || j ~= 0)

                s_x = node_x + k;
                s_y = node_y + j;
    
                % - Boundary check -
                if (s_x < 1 || s_x > MAX_X || s_y < 1 || s_y > MAX_Y)
                    continue;
                end
    
                % - Obstacle check -
                if MAP(s_x, s_y) == -1
                    continue; % to skip obstacles
                end
                
                % sanity check
                flag = 1;
                for c1 = 1:c2
                    if(s_x == CLOSED(c1, 1) && s_y == CLOSED(c1, 2))
                        flag = 0;
                        break;
                    end
                end
    
                % - Closed list check -
                if (flag == 1)
                    gn = gn_current + distance(node_x, node_y, s_x, s_y);
                    hn = distance(s_x, s_y, xTarget, yTarget);
                    fn = gn + hn;
                    exp_array(exp_count,:) = [s_x, s_y, gn, hn, fn];
                    exp_count = exp_count + 1;
                end
            end
        end
    end
end