function [state, options, optchanged] = guardarFitness(options, state, flag)
    global BESTS
    optchanged = false;
    if isequal(flag, 'iter')
        BESTS(end+1) = state.Best(end);
    end
end