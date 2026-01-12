% function to Populate the OPEN list
function new_row = insert_open(xval,yval,parent_xval,parent_yval,hn,gn,fn)
    new_row = zeros(1, 8);
    new_row(1) = 1;
    new_row(2) = xval;
    new_row(3) = yval;
    new_row(4) = parent_xval;
    new_row(5) = parent_yval;
    new_row(6) = hn;
    new_row(7) = gn;
    new_row(8) = fn;
end