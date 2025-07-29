function r = ref_pos_func(t)
    global ref_pos_list

    duration = 5;  % duración de cada tramo
    idx = floor(t / duration) + 1;
    idx = min(idx, length(ref_pos_list));

    r = ref_pos_list(idx);
end