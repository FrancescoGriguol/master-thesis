function isCorner = arcTest(T, centerY, centerX, offsets, N, threshold)
%ARCTEST Esegue il test ARC su un singolo pixel, tenendo conto della polarità.
%   T: Time surface (con polarità, valori positivi e negativi)
%   centerY, centerX: coordinate del pixel centrale
%   offsets: [dy, dx] (N x 2)
%   N: numero minimo di pixel consecutivi
%   threshold: soglia temporale in secondi (delta_t)

    isCorner = false;
    numPoints = size(offsets, 1);

    if centerY < 1 || centerX < 1 || ...
       centerY > size(T, 1) || centerX > size(T, 2)
        return;
    end

    centerTime = T(centerY, centerX);
    if centerTime == 0
        return;
    end

    centerPolarity = sign(centerTime);
    arcPattern = false(numPoints, 1);

    for i = 1:numPoints
        y = centerY + offsets(i,1);
        x = centerX + offsets(i,2);
        if y >= 1 && y <= size(T,1) && x >= 1 && x <= size(T,2)
            t_i = T(y, x);
            if sign(t_i) == centerPolarity  % stessa polarità
                arcPattern(i) = abs(t_i - centerTime) <= threshold;
            else
                arcPattern(i) = false;
            end
        else
            arcPattern(i) = false;
        end
    end

    % Verifica N pixel consecutivi (wrap-around)
    arcPattern = [arcPattern; arcPattern(1:N-1)];
    counts = conv(double(arcPattern), ones(N,1), 'valid');
    if any(counts == N)
        isCorner = true;
    end
end
