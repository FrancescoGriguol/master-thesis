function [Tp_on, Tp_off, Tp, Tnp, S_on, S_off, numEventsInWindow] = buildTimeSurfaces(events, t, resolution, tau, delta, Nmax, Nmin)
% Calcola le Time Surfaces:
% - Tp_on: Time Surface con polarità ON
% - Tp_off: Time Surface con polarità OFF
% - Tp: Time Surface combinata con polarità (come nel paper)
% - Tnp: Time Surface normalizzata senza polarità

    width = resolution(1);
    height = resolution(2);

    % Finestra temporale
    timeMin = t - delta;
    idx = find(events.timeStamp >= timeMin & events.timeStamp <= t);

    % Controllo numero eventi
    numEventsInWindow = length(idx);
    if numEventsInWindow > Nmax
        idx = idx(end - Nmax + 1:end);
    elseif numEventsInWindow < Nmin
        idx_all = find(events.timeStamp <= t);
        if length(idx_all) >= Nmin
            idx = idx_all(end - Nmin + 1:end);
        else
            idx = idx_all;
        end
    end

    % Estrai eventi
    x = double(events.x(idx)) + 1;
    y = double(events.y(idx)) + 1;
    times = double(events.timeStamp(idx));
    p = 2 * double(events.polarity(idx)) - 1; % {-1, +1}

    % SAE separati
    S_on = zeros(height, width);
    S_off = zeros(height, width);
    S_combined = zeros(height, width);
    P_combined = zeros(height, width); % polarità dell'ultimo evento

    for i = 1:length(times)
        if p(i) == 1
            S_on(y(i), x(i)) = times(i);
        else
            S_off(y(i), x(i)) = times(i);
        end
        % Aggiorna superficie combinata
        S_combined(y(i), x(i)) = times(i);
        P_combined(y(i), x(i)) = p(i);
    end
    
    % Decadimento esponenziale
    T_on = exp(-(t - S_on) / tau);
    T_on(S_on == 0) = 0;

    T_off = exp(-(t - S_off) / tau);
    T_off(S_off == 0) = 0;

    % Time Surfaces con polarità separata
    Tp_on = T_on;
    Tp_off = -T_off;

    % Time Surface combinata con polarità 
    T_comb = exp(-(t - S_combined) / tau);
    T_comb(S_combined == 0) = 0;
    Tp = P_combined .* T_comb;

    % Time Surface normalizzata senza polarità
    T_np_raw = T_on + T_off;
    maxT = max(T_np_raw(:));
    minT = min(T_np_raw(:));
    if maxT > minT
        % Tnp = (T_np_raw - minT) / (maxT - minT);
        Tnp = uint8(255 * (T_np_raw - minT) / (maxT - minT));
    else
        % Tnp = zeros(size(T_np_raw));
        Tnp = zeros(size(T_np_raw),'uint8');
    end
end

