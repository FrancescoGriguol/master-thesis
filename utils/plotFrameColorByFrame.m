function plotFrameColorByFrame(T, label, scale, baseColor)
    % T: 4x4 transform
    % label: nome del frame
    % scale: lunghezza degli assi
    % baseColor: colore RGB del frame, usato per tintare gli assi

    if nargin < 3, scale = 0.1; end
    if nargin < 4, baseColor = [0 0 0]; end

    origin = T(1:3,4);
    R = T(1:3,1:3);
    axes = {'X', 'Y', 'Z'};
    axisDirs = R * scale * eye(3);

    hold on;
    for i = 1:3
        color = baseColor;
        color(i) = min(1, color(i) + 0.5);  % esalta canale dell'asse
        q = quiver3(origin(1), origin(2), origin(3), ...
                    axisDirs(1,i), axisDirs(2,i), axisDirs(3,i), ...
                    0, 'Color', color, 'LineWidth', 1.8);
        % Etichetta asse
        text(origin(1) + axisDirs(1,i), ...
             origin(2) + axisDirs(2,i), ...
             origin(3) + axisDirs(3,i), ...
             [' ', label, '-', axes{i}], ...
             'Color', color, 'FontSize', 9, 'FontWeight', 'bold');
    end
end
