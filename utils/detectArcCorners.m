function corners = detectArcCorners(T_on, T_off, S_on, S_off, radius, numPoints, N, threshold, minDist)
%DETECTCORNERSPOLARIZEDARC Corner detection separata su T_on e T_off
%   T_on, T_off: time surfaces separate
%   radius: raggio del cerchio
%   numPoints: numero punti sul cerchio
%   N: pixel consecutivi minimi nel test ARC
%   threshold: soglia temporale (in secondi)
%   minDist: distanza minima tra corner

    offsets = getCircleOffsets(radius, numPoints);

    corners_on  = arcCorners(T_on, S_on, offsets, N, threshold, 1);
    corners_off = arcCorners(T_off, S_off, offsets, N, threshold, -1);

    % Unisci corner e applica filtraggio per distanza minima
    allCorners = [corners_on; corners_off];
    if ~isempty(allCorners)
        % allCorners = unique(round(allCorners), 'rows'); % [x, y, pol, time]
        % Applichiamo round SOLO alle coordinate spaziali
        allCorners(:,1:2) = round(allCorners(:,1:2));
        allCorners = unique(allCorners, 'rows');
        corners = selectMinDistance(allCorners, minDist);
    else
        corners = [];
    end
end


function corners = arcCorners(T, S, offsets, N, threshold, polarity)
    % arcCorners rileva corner su superficie T con timestamp in S
    % Restituisce corner come [x, y, polarity, timestamp]
    corners = [];  % inizializza vuoto
    % Scansiona tutti i pixel con T > 0
    [rows, cols] = find(T);  % rows = y, cols = x
    for i = 1:length(rows)
        y = rows(i);
        x = cols(i);
        % Esegui test solo se valore SAE esistente
        if S(y, x) == 0
            continue
        end
        if arcTest(T, y, x, offsets, N, threshold)
            timestamp = S(y, x);
            corners(end+1, :) = [x, y, polarity, timestamp]; 
        end
    end
end


function selected = selectMinDistance(corners, minDist)
    if isempty(corners)
        selected = [];
        return;
    end
    numCorners = size(corners,1);
    selected = zeros(numCorners, 4);
    selected(1,:) = corners(1,:);
    numSelected = 1;
    for i = 2:numCorners
        dists = sqrt(sum((selected(1:numSelected,1:2) - corners(i,1:2)).^2, 2));
        if all(dists >= minDist)
            numSelected = numSelected + 1;
            selected(numSelected,:) = corners(i,:);
        end
    end
    selected = selected(1:numSelected, :);
end