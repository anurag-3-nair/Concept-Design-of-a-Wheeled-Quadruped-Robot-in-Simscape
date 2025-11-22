function n_index = node_index(OPEN, xval, yval)

    % this function returns the index of the location of a node in the list
    if isempty(OPEN)
        n_index = -1;
        return;
    end

    idx = find(OPEN(:, 2) == xval & OPEN(:, 3) == yval, 1);

    if isempty(idx)
        n_index = -1;
    else
        n_index = idx;
    end
end