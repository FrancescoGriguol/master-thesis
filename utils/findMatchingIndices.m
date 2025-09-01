function idxMatch = findMatchingIndices(allPoints, targetPoints, threshold)
% Trova gli indici in allPoints che corrispondono a targetPoints
% allPoints: Nx2 matrice (es. trackedFeatures{39}(:,:,1))
% targetPoints: Mx2 matrice (es. inliers1)
% threshold: distanza massima per considerare due punti corrispondenti

if nargin < 3
    threshold = 1.0;  % default: 1 pixel
end

idxMatch = zeros(size(targetPoints,1),1);  % preallocazione

for i = 1:size(targetPoints,1)
    diffs = allPoints - targetPoints(i,:);  % differenza tra punti
    dists = sqrt(sum(diffs.^2, 2));         % distanza euclidea
    [minDist, idx] = min(dists);
    if minDist < threshold
        idxMatch(i) = idx;
    else
        idxMatch(i) = NaN;  % nessuna corrispondenza trovata
    end
end

% Rimuovi eventuali NaN
idxMatch = idxMatch(~isnan(idxMatch));
end
