function offsets = getCircleOffsets(radius, numPoints)
%GETCIRCLEOFFSETS Restituisce offset su un cerchio di raggio arbitrario.
%   offsets = getCircleOffsets(radius, numPoints)
%   radius: raggio del cerchio (pixel)
%   numPoints: numero di punti equispaziati da campionare (es. 16)

    arguments
        radius (1,1) double {mustBePositive, mustBeInteger}
        numPoints (1,1) double {mustBePositive, mustBeInteger}
    end

    theta = linspace(0, 2*pi, numPoints + 1);
    theta(end) = []; % rimuove duplicato 2*pi

    dx = round(radius * cos(theta));
    dy = round(radius * sin(theta));

    % offsets = [dy', dx'];  
    offsets = unique([dy', dx'], 'rows'); % [row_offset, col_offset]
end
