function cost = objectiveFunc_dummy(fis)
    % Evaluate the output of the FIS for some dummy input
    output = evalfis(fis, [0.2 0.2]);  % input1 and input2 normalized
    cost = abs(output - 1);            % dummy cost: want output near 1
end
