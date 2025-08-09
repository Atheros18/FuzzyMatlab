function [state, options, optchanged] = guardarFitness(options, state, flag)
    global BESTS
    optchanged = false;

    if isequal(flag, 'iter')
        BESTS(end+1) = min(state.Score);  % más directo
    end

    if isequal(flag, 'done')
        save('poblacion_final.mat', 'state');
    end
end