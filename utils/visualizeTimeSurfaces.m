function visualizeTimeSurfaces(Tp_on, Tp_off, Tp, Tnp)

    % --- Normalized Time Surface ---
    fig1 = figure('Name', 'Normalized Time Surface', 'Color', 'w');
    ax1 = axes('Parent', fig1);  % assegna un axes esplicito
    imshow(Tnp, [0, 255], 'Parent', ax1);
    axis(ax1, 'image');
    ax1.XColor = 'none';
    ax1.YColor = 'none';
    colormap(ax1, gray);  % colormap associata all'axes corrente
    cb = colorbar(ax1);   % assegna la colorbar all'axes
    cb.Label.String = 'Normalized Intensity (0 = old, 255 = recent)';
    title(ax1, 'Normalized Time Surface (Tnp)', 'FontWeight', 'bold');

    % --- Time Surface ON (pixel verdi) ---
    fig2 = figure('Name', 'Time Surface ON', 'Color', 'w');
    ax2 = axes('Parent', fig2);
    mask_on = Tp_on > 0;
    img_on = zeros([size(Tp_on), 3]);
    img_on(:,:,2) = mask_on;  % Verde
    image(ax2, img_on);
    axis(ax2, 'image');
    ax2.XColor = 'none';
    ax2.YColor = 'none';
    title(ax2, 'ON Events (Verde)', 'FontWeight', 'bold');

    % --- Time Surface OFF (pixel rossi) ---
    fig3 = figure('Name', 'Time Surface OFF', 'Color', 'w');
    ax3 = axes('Parent', fig3);
    mask_off = Tp_off < 0;
    img_off = zeros([size(Tp_off), 3]);
    img_off(:,:,1) = mask_off;  % Rosso
    image(ax3, img_off);
    axis(ax3, 'image');
    ax3.XColor = 'none';
    ax3.YColor = 'none';
    title(ax3, 'OFF Events (Rosso)', 'FontWeight', 'bold');

    % --- Tp: Verde (ON) + Rosso (OFF) ---
    fig4 = figure('Name', 'Combined Time Surface Tp', 'Color', 'w');
    ax4 = axes('Parent', fig4);
    mask_on = Tp > 0;
    mask_off = Tp < 0;
    img_comb = zeros([size(Tp), 3]);
    img_comb(:,:,2) = mask_on;  % Verde
    img_comb(:,:,1) = mask_off; % Rosso
    image(ax4, img_comb);
    axis(ax4, 'image');
    ax4.XColor = 'none';
    ax4.YColor = 'none';
    title(ax4, 'Time Surface Tp: Verde = ON, Rosso = OFF', 'FontWeight', 'bold');

end
