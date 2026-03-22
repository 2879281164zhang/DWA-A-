clear; clc; close all;

%% ===================== Global Style =====================
set(groot, 'defaultAxesFontName', 'Times New Roman');
set(groot, 'defaultTextFontName', 'Times New Roman');
set(groot, 'defaultAxesFontSize', 12);
set(groot, 'defaultTextFontSize', 12);

%% ===================== Parameters =====================
cfg.inputResultFile      = 'fuxian_result.mat';

cfg.outputMP4            = 'route_inset_animation_optimized.mp4';
cfg.outputGIF            = 'route_inset_animation_optimized.gif';
cfg.outputObsMat         = 'generated_target_ships_optimized.mat';

cfg.saveMP4              = true;
cfg.saveGIF              = false;
cfg.saveObsMat           = true;

cfg.durationSec          = 60;
cfg.fps                  = 12;
cfg.nFrames              = cfg.durationSec * cfg.fps;

cfg.leftAxesPos          = [0.05 0.10 0.53 0.80];
cfg.rightAxesPos         = [0.64 0.20 0.28 0.50];

cfg.nominalColor         = [0.60 0.60 0.60];
cfg.actualColor          = [0.00 0.15 1.00];
cfg.zoomRectColor        = [0.88 0.20 0.20];
cfg.ownShipColor         = [0.88 0.12 0.12];
cfg.fishingTrailColor    = [0.95 0.10 0.10];

cfg.nominalWidth         = 0.9;
cfg.actualWidthMain      = 2.0;
cfg.actualWidthZoom      = 1.8;
cfg.zoomRectWidth        = 1.1;
cfg.shipTrailWidthMain   = 1.0;
cfg.shipTrailWidthZoom   = 0.9;

cfg.startGoalMarkerSize  = 12;
cfg.labelFontSize        = 10;
cfg.expLabelFontSize     = 11;

cfg.showGrid             = false;
cfg.showDWA              = false;
cfg.showShipTrails       = true;

cfg.zoomLonFraction      = 0.12;
cfg.zoomLatFraction      = 0.12;
cfg.zoomAheadFrac        = 0.020;

cfg.safeBufferRadius     = 4;
cfg.routeBufferRadius    = 9;
cfg.snapRadius           = 20;

cfg.ownShipLengthFracLon    = 0.0030;
cfg.ownShipWidthFracLat     = 0.0018;
cfg.targetShipLengthFracLon = 0.0026;
cfg.targetShipWidthFracLat  = 0.0016;
cfg.fishingBoatScale        = 0.75;

cfg.nominalDenseN        = 440;

cfg.headFrac             = 0.28;
cfg.fishFrac             = 0.66;
cfg.overFrac             = 0.68;

cfg.headAvoidAmpFrac     = 0.0072;
cfg.fishAvoidAmpFrac     = 0.0078;
cfg.overAvoidAmpFrac     = 0.0120;

cfg.headAvoidSign        = -1;
cfg.fishAvoidSign        = -1;
cfg.overAvoidSign        = -1;

cfg.headPrePts           = 2;
cfg.fishPrePts           = 3;
cfg.overPrePts           = 10;

cfg.headRejoinMin        = 24;
cfg.headRejoinMax        = 50;
cfg.fishRejoinMin        = 22;
cfg.fishRejoinMax        = 48;
cfg.overRejoinMin        = 52;
cfg.overRejoinMax        = 120;

cfg.leadingFixCount      = 72;
cfg.routeSmoothWindow    = 7;
cfg.routeSmoothPasses    = 1;

cfg.shipColors = [
    0.90 0.40 0.10;
    0.49 0.18 0.56;
    0.82 0.72 0.15
];

%% ===================== Load Result =====================
if exist(cfg.inputResultFile, 'file') ~= 2
    error('Cannot find %s.', cfg.inputResultFile);
end

S = load(cfg.inputResultFile);

if ~isfield(S, 'G')
    error('Variable "G" not found in %s.', cfg.inputResultFile);
end
if ~isfield(S, 'pathLat') || ~isfield(S, 'pathLon')
    error('Variables "pathLat" and "pathLon" not found in %s.', cfg.inputResultFile);
end

G = logical(S.G);
pathLatRaw = S.pathLat(:);
pathLonRaw = S.pathLon(:);

if numel(pathLatRaw) < 2 || numel(pathLonRaw) < 2
    error('Path data is too short.');
end

if isfield(S, 'mapInfo')
    mapInfo = S.mapInfo;
else
    mapInfo = struct();
end

if ~isfield(mapInfo, 'latRange')
    mapInfo.latRange = [35.9690, 36.0614];
end
if ~isfield(mapInfo, 'lonRange')
    mapInfo.lonRange = [120.2090, 120.4070];
end

latRange = mapInfo.latRange;
lonRange = mapInfo.lonRange;

lonW = diff(lonRange);
latH = diff(latRange);
latMean = mean(latRange);
lonScale = cosd(latMean);

%% ===================== Masks =====================
freeMask = ~G;

dangerMask = conv2(double(G), ones(2 * cfg.safeBufferRadius + 1), 'same') > 0;
safeMask = freeMask & ~dangerMask;

routeDangerMask = conv2(double(G), ones(2 * cfg.routeBufferRadius + 1), 'same') > 0;
routeMask = freeMask & ~routeDangerMask;

if ~any(routeMask(:))
    warning('routeMask is empty. Fallback to safeMask.');
    routeMask = safeMask;
end

%% ===================== Rebuild Nominal Route =====================
[pathLonNomDense, pathLatNomDense] = buildNominalRouteLocal( ...
    pathLonRaw, pathLatRaw, routeMask, safeMask, latRange, lonRange, cfg.nominalDenseN, cfg.snapRadius);

[pathLonNomDense, pathLatNomDense] = suppressLeadingBacktrackLocal( ...
    pathLonNomDense, pathLatNomDense, routeMask, safeMask, latRange, lonRange, cfg.leadingFixCount);

[pathLonNomDense, pathLatNomDense] = stabilizeRouteLocal( ...
    pathLonNomDense, pathLatNomDense, routeMask, safeMask, latRange, lonRange, ...
    cfg.routeSmoothWindow, cfg.routeSmoothPasses);

[pathLonNomDense, pathLatNomDense] = fixHarborExitLocal( ...
    pathLonNomDense, pathLatNomDense, routeMask, safeMask, latRange, lonRange, round(0.11 * cfg.nominalDenseN));

%% ===================== Build Actual Route =====================
[pathLonActDense, pathLatActDense, routeInfo] = buildActualRouteLocal( ...
    pathLonNomDense, pathLatNomDense, routeMask, safeMask, freeMask, latRange, lonRange, cfg);

[pathLonActDense, pathLatActDense] = suppressLeadingBacktrackLocal( ...
    pathLonActDense, pathLatActDense, routeMask, safeMask, latRange, lonRange, cfg.leadingFixCount);

[pathLonActDense, pathLatActDense] = stabilizeRouteLocal( ...
    pathLonActDense, pathLatActDense, routeMask, safeMask, latRange, lonRange, ...
    cfg.routeSmoothWindow, cfg.routeSmoothPasses);

[pathLonActDense, pathLatActDense] = fixHarborExitLocal( ...
    pathLonActDense, pathLatActDense, routeMask, safeMask, latRange, lonRange, round(0.11 * numel(pathLonActDense)));

[pathLonActDense, pathLatActDense] = resamplePolylineByCountLinearLocal( ...
    pathLonActDense, pathLatActDense, max(cfg.nominalDenseN, 520));

[pathLonActDense, pathLatActDense] = stabilizeRouteLocal( ...
    pathLonActDense, pathLatActDense, routeMask, safeMask, latRange, lonRange, 5, 1);

[pathLonActDense, pathLatActDense] = snapOnlyInvalidPointsToMaskLocal( ...
    pathLonActDense, pathLatActDense, routeMask, safeMask, latRange, lonRange, cfg.snapRadius);

%% ===================== Resample for Animation =====================
[pathLonNom, pathLatNom] = resamplePolylineByCountLinearLocal( ...
    pathLonNomDense, pathLatNomDense, cfg.nFrames);

[pathLonAct, pathLatAct] = resamplePolylineByCountLinearLocal( ...
    pathLonActDense, pathLatActDense, cfg.nFrames);

[pathLonAct, pathLatAct] = snapOnlyInvalidPointsToMaskLocal( ...
    pathLonAct, pathLatAct, routeMask, safeMask, latRange, lonRange, cfg.snapRadius);

%% ===================== Create Target Ships =====================
ships = createScenarioShipsLocal( ...
    pathLonNomDense, pathLatNomDense, routeMask, safeMask, freeMask, latRange, lonRange, cfg);

if cfg.saveObsMat
    save(cfg.outputObsMat, 'ships', ...
        'pathLonNomDense', 'pathLatNomDense', ...
        'pathLonActDense', 'pathLatActDense', 'routeInfo');
end

nShips = numel(ships);

%% ===================== Figure and Axes =====================
fig = figure('Color', 'w', ...
    'Name', 'Inset Route Animation Optimized', ...
    'Units', 'pixels', ...
    'Position', [60 60 1500 820]);

axMain = axes('Parent', fig, 'Position', cfg.leftAxesPos);
axZoom = axes('Parent', fig, 'Position', cfg.rightAxesPos);

drawBinaryMapGeoLocal(axMain, G, latRange, lonRange, cfg.showGrid);
drawBinaryMapGeoLocal(axZoom, G, latRange, lonRange, cfg.showGrid);

hold(axMain, 'on');
hold(axZoom, 'on');

%% ===================== Static Graphics =====================
plot(axMain, pathLonNom, pathLatNom, ':', ...
    'Color', cfg.nominalColor, 'LineWidth', cfg.nominalWidth);

plot(axZoom, pathLonNom, pathLatNom, ':', ...
    'Color', cfg.nominalColor, 'LineWidth', cfg.nominalWidth);

plot(axMain, pathLonNom(1), pathLatNom(1), 'gp', ...
    'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g', ...
    'MarkerSize', cfg.startGoalMarkerSize);

plot(axMain, pathLonNom(end), pathLatNom(end), 'rp', ...
    'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', ...
    'MarkerSize', cfg.startGoalMarkerSize);

plot(axZoom, pathLonNom(1), pathLatNom(1), 'gp', ...
    'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g', ...
    'MarkerSize', cfg.startGoalMarkerSize);

plot(axZoom, pathLonNom(end), pathLatNom(end), 'rp', ...
    'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', ...
    'MarkerSize', cfg.startGoalMarkerSize);

%% ===================== Moving Zoom Box =====================
boxW = cfg.zoomLonFraction * lonW;
boxH = cfg.zoomLatFraction * latH;

initBox = [pathLonAct(1) - 0.5 * boxW, pathLatAct(1) - 0.5 * boxH, boxW, boxH];

hRectMain = rectangle(axMain, 'Position', initBox, ...
    'EdgeColor', cfg.zoomRectColor, ...
    'LineWidth', cfg.zoomRectWidth, ...
    'LineStyle', '--');

xlim(axZoom, [initBox(1), initBox(1) + initBox(3)]);
ylim(axZoom, [initBox(2), initBox(2) + initBox(4)]);

%% ===================== Own Ship History =====================
hTrailMain = plot(axMain, nan, nan, '-', ...
    'Color', cfg.actualColor, 'LineWidth', cfg.actualWidthMain);

hTrailZoom = plot(axZoom, nan, nan, '-', ...
    'Color', cfg.actualColor, 'LineWidth', cfg.actualWidthZoom);

%% ===================== Own Ship Shape =====================
Lown = cfg.ownShipLengthFracLon * lonW;
Wown = cfg.ownShipWidthFracLat * latH;

psiOwn0 = headingFromPathLocal(pathLonAct, pathLatAct, 1);
[vxOwn0, vyOwn0] = fusiformVerticesLocal(pathLonAct(1), pathLatAct(1), psiOwn0, Lown, Wown);

hOwnShipMain = patch(axMain, vxOwn0, vyOwn0, cfg.ownShipColor, ...
    'EdgeColor', cfg.ownShipColor, 'LineWidth', 0.7);

hOwnShipZoom = patch(axZoom, vxOwn0, vyOwn0, cfg.ownShipColor, ...
    'EdgeColor', cfg.ownShipColor, 'LineWidth', 0.7);

%% ===================== Own Ship Labels =====================
hOwnTextMain = text(axMain, pathLonAct(1), pathLatAct(1), 'Experimental ship', ...
    'FontName', 'Times New Roman', ...
    'FontSize', cfg.expLabelFontSize, ...
    'Color', 'k', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'bottom', 'Clipping', 'on');

hOwnTextZoom = text(axZoom, pathLonAct(1), pathLatAct(1), 'Experimental ship', ...
    'FontName', 'Times New Roman', ...
    'FontSize', cfg.expLabelFontSize, ...
    'Color', 'k', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'bottom', 'Clipping', 'on');

%% ===================== Target Ship Shapes =====================
Ltar = cfg.targetShipLengthFracLon * lonW;
Wtar = cfg.targetShipWidthFracLat * latH;

hShipMain = gobjects(nShips, 1);
hShipZoom = gobjects(nShips, 1);

for i = 1:nShips
    [vx, vy] = fusiformVerticesLocal(ships(i).lon(1), ships(i).lat(1), ships(i).heading(1), ...
        Ltar * ships(i).sizeScale, Wtar * ships(i).sizeScale);

    hShipMain(i) = patch(axMain, vx, vy, ships(i).color, ...
        'EdgeColor', ships(i).color, 'LineWidth', 0.7);

    hShipZoom(i) = patch(axZoom, vx, vy, ships(i).color, ...
        'EdgeColor', ships(i).color, 'LineWidth', 0.7);
end

%% ===================== Target Ship Trails =====================
hShipTrailMain = gobjects(nShips, 1);
hShipTrailZoom = gobjects(nShips, 1);

if cfg.showShipTrails
    for i = 1:nShips
        if strcmpi(ships(i).name, 'Fishing boat')
            trailColor = cfg.fishingTrailColor;
        else
            trailColor = ships(i).color;
        end

        hShipTrailMain(i) = plot(axMain, nan, nan, '-', ...
            'Color', trailColor, 'LineWidth', cfg.shipTrailWidthMain);

        hShipTrailZoom(i) = plot(axZoom, nan, nan, '-', ...
            'Color', trailColor, 'LineWidth', cfg.shipTrailWidthZoom);
    end
end

%% ===================== Ship Name Labels =====================
hShipTextMain = gobjects(nShips, 1);
for i = 1:nShips
    [dxLab, dyLab] = getShipLabelOffsetLocal(i, lonW, latH);
    hShipTextMain(i) = text(axMain, ships(i).lon(1) + dxLab, ships(i).lat(1) + dyLab, ships(i).name, ...
        'FontName', 'Times New Roman', ...
        'FontSize', cfg.labelFontSize, ...
        'Color', 'k', ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'Clipping', 'on');
end

hShipTextZoom = text(axZoom, ships(1).lon(1), ships(1).lat(1), ships(1).name, ...
    'FontName', 'Times New Roman', ...
    'FontSize', cfg.labelFontSize, ...
    'Color', 'k', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', 'Clipping', 'on');

%% ===================== Video / GIF Writers =====================
if cfg.saveMP4
    v = VideoWriter(cfg.outputMP4, 'MPEG-4');
    v.FrameRate = cfg.fps;
    open(v);
else
    v = [];
end

gifInitialized = false;

%% ===================== Animation Loop =====================
for k = 1:cfg.nFrames
    set(hTrailMain, 'XData', pathLonAct(1:k), 'YData', pathLatAct(1:k));
    set(hTrailZoom, 'XData', pathLonAct(1:k), 'YData', pathLatAct(1:k));

    psiOwn = headingFromPathLocal(pathLonAct, pathLatAct, k);
    [vxOwn, vyOwn] = fusiformVerticesLocal(pathLonAct(k), pathLatAct(k), psiOwn, Lown, Wown);
    set(hOwnShipMain, 'XData', vxOwn, 'YData', vyOwn);
    set(hOwnShipZoom, 'XData', vxOwn, 'YData', vyOwn);

    [tNow, ~] = tangentNormalFromPathLocal(pathLonAct, pathLatAct, k, lonScale);
    xCenter = pathLonAct(k) + cfg.zoomAheadFrac * lonW * tNow(1);
    yCenter = pathLatAct(k) + cfg.zoomAheadFrac * lonW * tNow(2);

    x0 = xCenter - 0.5 * boxW;
    y0 = yCenter - 0.5 * boxH;

    x0 = max(lonRange(1), min(x0, lonRange(2) - boxW));
    y0 = max(latRange(1), min(y0, latRange(2) - boxH));

    set(hRectMain, 'Position', [x0, y0, boxW, boxH]);
    xlim(axZoom, [x0, x0 + boxW]);
    ylim(axZoom, [y0, y0 + boxH]);

    set(hOwnTextMain, 'Position', [pathLonAct(k) - 0.025 * lonW, pathLatAct(k) + 0.028 * latH, 0]);
    set(hOwnTextZoom, 'Position', [x0 + 0.36 * boxW, y0 + 0.68 * boxH, 0]);

    nearestShipIdx = 1;
    nearestDist = inf;

    for i = 1:nShips
        [vx, vy] = fusiformVerticesLocal(ships(i).lon(k), ships(i).lat(k), ships(i).heading(k), ...
            Ltar * ships(i).sizeScale, Wtar * ships(i).sizeScale);

        set(hShipMain(i), 'XData', vx, 'YData', vy);
        set(hShipZoom(i), 'XData', vx, 'YData', vy);

        if cfg.showShipTrails
            set(hShipTrailMain(i), 'XData', ships(i).lon(1:k), 'YData', ships(i).lat(1:k));
            set(hShipTrailZoom(i), 'XData', ships(i).lon(1:k), 'YData', ships(i).lat(1:k));
        end

        d = hypot((ships(i).lon(k) - pathLonAct(k)) * lonScale, (ships(i).lat(k) - pathLatAct(k)));
        if d < nearestDist
            nearestDist = d;
            nearestShipIdx = i;
        end

        [dxLab, dyLab] = getShipLabelOffsetLocal(i, lonW, latH);
        set(hShipTextMain(i), 'Position', [ships(i).lon(k) + dxLab, ships(i).lat(k) + dyLab, 0]);
    end

    [dxLabZ, dyLabZ] = getShipLabelOffsetLocal(nearestShipIdx, boxW, boxH);
    set(hShipTextZoom, ...
        'String', ships(nearestShipIdx).name, ...
        'Position', [ships(nearestShipIdx).lon(k) + 0.12 * dxLabZ, ships(nearestShipIdx).lat(k) + 0.12 * dyLabZ, 0]);

    drawnow;

    frame = getframe(fig);

    if cfg.saveMP4
        writeVideo(v, frame);
    end

    if cfg.saveGIF
        [im, cm] = rgb2ind(frame2im(frame), 256);
        if ~gifInitialized
            imwrite(im, cm, cfg.outputGIF, 'gif', 'Loopcount', inf, 'DelayTime', 1 / cfg.fps);
            gifInitialized = true;
        else
            imwrite(im, cm, cfg.outputGIF, 'gif', 'WriteMode', 'append', 'DelayTime', 1 / cfg.fps);
        end
    end
end

if cfg.saveMP4
    close(v);
end

fprintf('Animation finished.\n');
if cfg.saveMP4
    fprintf('MP4 saved: %s\n', cfg.outputMP4);
end
if cfg.saveGIF
    fprintf('GIF saved: %s\n', cfg.outputGIF);
end
if cfg.saveObsMat
    fprintf('Target ship trajectories saved: %s\n', cfg.outputObsMat);
end

%% ===================== Local Functions =====================

function [pathLonNom, pathLatNom] = buildNominalRouteLocal(pathLonRaw, pathLatRaw, primaryMask, secondaryMask, latRange, lonRange, nDense, snapRadius)
pathLonRaw = pathLonRaw(:);
pathLatRaw = pathLatRaw(:);

valid = isfinite(pathLonRaw) & isfinite(pathLatRaw);
pathLonRaw = pathLonRaw(valid);
pathLatRaw = pathLatRaw(valid);

if numel(pathLonRaw) < 2
    error('Raw nominal path is too short after removing invalid points.');
end

nRaw = numel(pathLonRaw);
anchorIdx = unique(round(linspace(1, nRaw, 7)));

anchorLon = pathLonRaw(anchorIdx);
anchorLat = pathLatRaw(anchorIdx);

[anchorLon, anchorLat] = snapOnlyInvalidPointsToMaskLocal( ...
    anchorLon, anchorLat, primaryMask, secondaryMask, latRange, lonRange, snapRadius);

P = [];
for i = 1:(numel(anchorLon) - 1)
    ctrlGeo = [anchorLon(i), anchorLat(i); anchorLon(i + 1), anchorLat(i + 1)];
    segGeo = buildSegmentViaWaypointsLocal(ctrlGeo, primaryMask, secondaryMask, latRange, lonRange);

    if isempty(segGeo)
        lonTmp = linspace(ctrlGeo(1, 1), ctrlGeo(2, 1), 80)';
        latTmp = linspace(ctrlGeo(1, 2), ctrlGeo(2, 2), 80)';
        [lonTmp, latTmp] = snapOnlyInvalidPointsToMaskLocal( ...
            lonTmp, latTmp, primaryMask, secondaryMask, latRange, lonRange, snapRadius);
        segGeo = [lonTmp, latTmp];
    end

    if i > 1 && ~isempty(segGeo)
        segGeo(1, :) = [];
    end
    P = [P; segGeo]; %#ok<AGROW>
end

P = uniqueConsecutiveRowsLocal(P);
[pathLonNom, pathLatNom] = resamplePolylineByCountLinearLocal(P(:, 1), P(:, 2), nDense);

[pathLonNom, pathLatNom] = snapOnlyInvalidPointsToMaskLocal( ...
    pathLonNom, pathLatNom, primaryMask, secondaryMask, latRange, lonRange, snapRadius);
end

function [pathLonAct, pathLatAct, info] = buildActualRouteLocal(pathLonNom, pathLatNom, primaryMask, secondaryMask, freeMask, latRange, lonRange, cfg)
n = numel(pathLonNom);

kHead = round(cfg.headFrac * n);
kOver = round(cfg.overFrac * n);
kFish = round(cfg.fishFrac * n);

%% ---------- 1) Head-on outside channel near harbor exit ----------
kHeadPre = max(2, kHead - cfg.headPrePts);
[headWp1, headWp2] = buildHeadAvoidWaypointsLocal( ...
    pathLonNom, pathLatNom, kHead, primaryMask, secondaryMask, freeMask, latRange, lonRange, cfg);

kHeadPost = chooseAdaptiveRejoinIndexLocal( ...
    pathLonNom, pathLatNom, ...
    max(kHeadPre + 6, kHead + cfg.headRejoinMin), ...
    min(n, kHead + cfg.headRejoinMax), ...
    headWp2, primaryMask, secondaryMask, latRange, lonRange);

%% ---------- 2) Start overtaking target ship 3 after entering channel ----------
kOverPre = max(kHeadPost + 2, kOver - cfg.overPrePts);
[overWp1, overWp2, overWp3] = buildOvertakeWaypointsLocal( ...
    pathLonNom, pathLatNom, kOver, primaryMask, secondaryMask, freeMask, latRange, lonRange, cfg);

kOverPost = chooseAdaptiveRejoinIndexLocal( ...
    pathLonNom, pathLatNom, ...
    max(kOverPre + 10, kOver + cfg.overRejoinMin), ...
    min(n, kOver + cfg.overRejoinMax), ...
    overWp3, primaryMask, secondaryMask, latRange, lonRange);

%% ---------- 3) Later encounter fishing boat and pass astern ----------
kFishPre = max(kOverPost + 2, kFish - cfg.fishPrePts);
[fishWp1, fishWp2] = buildFishingAvoidWaypointsLocal( ...
    pathLonNom, pathLatNom, kFish, primaryMask, secondaryMask, freeMask, latRange, lonRange, cfg);

kFishPost = chooseAdaptiveRejoinIndexLocal( ...
    pathLonNom, pathLatNom, ...
    max(kFishPre + 8, kFish + cfg.fishRejoinMin), ...
    min(n, kFish + cfg.fishRejoinMax), ...
    fishWp2, primaryMask, secondaryMask, latRange, lonRange);

%% ---------- Assemble route ----------
seg1_nom = [pathLonNom(1:kHeadPre), pathLatNom(1:kHeadPre)];

seg1_dyn = buildSegmentViaWaypointsLocal( ...
    [pathLonNom(kHeadPre), pathLatNom(kHeadPre); ...
     headWp1(1), headWp1(2); ...
     headWp2(1), headWp2(2); ...
     pathLonNom(kHeadPost), pathLatNom(kHeadPost)], ...
    primaryMask, secondaryMask, latRange, lonRange);

seg2_nom = [pathLonNom(kHeadPost:kOverPre), pathLatNom(kHeadPost:kOverPre)];

seg2_dyn = buildSegmentViaWaypointsLocal( ...
    [pathLonNom(kOverPre), pathLatNom(kOverPre); ...
     overWp1(1), overWp1(2); ...
     overWp2(1), overWp2(2); ...
     overWp3(1), overWp3(2); ...
     pathLonNom(kOverPost), pathLatNom(kOverPost)], ...
    primaryMask, secondaryMask, latRange, lonRange);

seg3_nom = [pathLonNom(kOverPost:kFishPre), pathLatNom(kOverPost:kFishPre)];

seg3_dyn = buildSegmentViaWaypointsLocal( ...
    [pathLonNom(kFishPre), pathLatNom(kFishPre); ...
     fishWp1(1), fishWp1(2); ...
     fishWp2(1), fishWp2(2); ...
     pathLonNom(kFishPost), pathLatNom(kFishPost)], ...
    primaryMask, secondaryMask, latRange, lonRange);

seg4_nom = [pathLonNom(kFishPost:end), pathLatNom(kFishPost:end)];

P = [
    seg1_nom;
    seg1_dyn(2:end, :);
    seg2_nom(2:end, :);
    seg2_dyn(2:end, :);
    seg3_nom(2:end, :);
    seg3_dyn(2:end, :);
    seg4_nom(2:end, :)
];

P = uniqueConsecutiveRowsLocal(P);
[pathLonAct, pathLatAct] = deal(P(:, 1), P(:, 2));

info.kHead = kHead;
info.kOver = kOver;
info.kFish = kFish;
info.kHeadPost = kHeadPost;
info.kOverPost = kOverPost;
info.kFishPost = kFishPost;
info.headWp1 = headWp1;
info.headWp2 = headWp2;
info.overWp1 = overWp1;
info.overWp2 = overWp2;
info.overWp3 = overWp3;
info.fishWp1 = fishWp1;
info.fishWp2 = fishWp2;
end
function segGeo = buildSegmentViaWaypointsLocal(ctrlGeo, primaryMask, secondaryMask, latRange, lonRange)
segRC = [];

for i = 1:(size(ctrlGeo, 1) - 1)
    pA = ctrlGeo(i, :);
    pB = ctrlGeo(i + 1, :);

    [rc, usedMask] = findRouteBetweenGeoLocal(pA, pB, primaryMask, secondaryMask, latRange, lonRange, 65);

    if isempty(rc)
        lonTmp = linspace(pA(1), pB(1), 80)';
        latTmp = linspace(pA(2), pB(2), 80)';
        [lonTmp, latTmp] = snapTrajectoryToMaskLocal( ...
            lonTmp, latTmp, primaryMask, secondaryMask, latRange, lonRange, 8);
        [rTmp, cTmp] = geoPathToRCPathLocal(lonTmp, latTmp, size(primaryMask), latRange, lonRange);
        rc = uniqueConsecutiveRowsLocal([rTmp, cTmp]);
        usedMask = secondaryMask;
    end

    rc = smoothPathGreedyLOSLocal(rc, usedMask);

    if i > 1 && ~isempty(rc)
        rc(1, :) = [];
    end
    segRC = [segRC; rc]; %#ok<AGROW>
end

[latSeg, lonSeg] = rcPathToGeoLocal(segRC, size(primaryMask), latRange, lonRange);
segGeo = [lonSeg, latSeg];
segGeo = uniqueConsecutiveRowsLocal(segGeo);
end

function [rc, usedMask] = findRouteBetweenGeoLocal(pA, pB, primaryMask, secondaryMask, latRange, lonRange, padding)
rc = astarBetweenGeoLocal(pA, pB, primaryMask, latRange, lonRange, padding);
if ~isempty(rc)
    usedMask = primaryMask;
    return;
end

rc = astarBetweenGeoLocal(pA, pB, secondaryMask, latRange, lonRange, padding);
if ~isempty(rc)
    usedMask = secondaryMask;
    return;
end

usedMask = secondaryMask;
end

function ships = createScenarioShipsLocal(pathLonNom, pathLatNom, routeMask, safeMask, freeMask, latRange, lonRange, cfg)
n = numel(pathLonNom);

kHead = round(cfg.headFrac * n);
kOver = round(cfg.overFrac * n);
kFish = round(cfg.fishFrac * n);

ships = repmat(struct( ...
    'name', '', ...
    'lon', [], ...
    'lat', [], ...
    'heading', [], ...
    'color', zeros(1, 3), ...
    'sizeScale', 1), 3, 1);

ships(1) = createHeadOnShipLocal( ...
    'Head-on ship', pathLonNom, pathLatNom, kHead, ...
    safeMask, freeMask, latRange, lonRange, ...
    cfg.shipColors(1, :), 0.95, cfg.nFrames);

ships(2) = createFishingBoatLocal( ...
    'Fishing boat', pathLonNom, pathLatNom, kFish, ...
    safeMask, freeMask, latRange, lonRange, ...
    cfg.shipColors(2, :), cfg.fishingBoatScale, cfg.nFrames);

ships(3) = createTargetShip3Local( ...
    'Target ship 3', pathLonNom, pathLatNom, kOver, ...
    routeMask, safeMask, latRange, lonRange, ...
    cfg.shipColors(3, :), 0.90, cfg.nFrames);
end
function ship = createShipOnReferenceLocal(name, refLon, refLat, idxStart, idxEnd, ...
    frameStart, frameEnd, offsetAmp, primaryMask, secondaryMask, latRange, lonRange, ...
    color, sizeScale, nFrames)

[lon, lat, hd] = buildReferenceTrajectoryLocal( ...
    refLon, refLat, idxStart, idxEnd, frameStart, frameEnd, ...
    offsetAmp, primaryMask, secondaryMask, latRange, lonRange, nFrames);

ship = struct( ...
    'name', name, ...
    'lon', lon, ...
    'lat', lat, ...
    'heading', hd, ...
    'color', color, ...
    'sizeScale', sizeScale);
end

function ship = createShipOnReferenceOffsetLocal(name, refLon, refLat, idxStart, idxEnd, ...
    frameStart, frameEnd, offsetAmp, primaryMask, secondaryMask, latRange, lonRange, ...
    color, sizeScale, nFrames)

[lon, lat, hd] = buildReferenceTrajectoryLocal( ...
    refLon, refLat, idxStart, idxEnd, frameStart, frameEnd, ...
    offsetAmp, primaryMask, secondaryMask, latRange, lonRange, nFrames);

ship = struct( ...
    'name', name, ...
    'lon', lon, ...
    'lat', lat, ...
    'heading', hd, ...
    'color', color, ...
    'sizeScale', sizeScale);
end

function [lon, lat, hd] = buildReferenceTrajectoryLocal(refLon, refLat, idxStart, idxEnd, ...
    frameStart, frameEnd, offsetAmp, primaryMask, secondaryMask, latRange, lonRange, nFrames)

idxStart = max(1, min(numel(refLon), idxStart));
idxEnd = max(1, min(numel(refLon), idxEnd));

if idxStart <= idxEnd
    lonSeg = refLon(idxStart:idxEnd);
    latSeg = refLat(idxStart:idxEnd);
else
    lonSeg = refLon(idxStart:-1:idxEnd);
    latSeg = refLat(idxStart:-1:idxEnd);
end

if abs(offsetAmp) > 0
    latMean = mean(latRange);
    lonScale = cosd(latMean);
    nSeg = numel(lonSeg);

    lonOff = zeros(nSeg, 1);
    latOff = zeros(nSeg, 1);

    for i = 1:nSeg
        [~, nVec] = tangentNormalFromPathLocal(lonSeg, latSeg, i, lonScale);
        lonOff(i) = lonSeg(i) + offsetAmp * nVec(1);
        latOff(i) = latSeg(i) + offsetAmp * nVec(2);
    end

    lonSeg = lonOff;
    latSeg = latOff;
end

[lonSeg, latSeg] = snapOnlyInvalidPointsToMaskLocal( ...
    lonSeg, latSeg, primaryMask, secondaryMask, latRange, lonRange, 8);

trackLen = max(2, frameEnd - frameStart + 1);
[lonTrack, latTrack] = resamplePolylineByCountLinearLocal(lonSeg, latSeg, trackLen);

lon = nan(nFrames, 1);
lat = nan(nFrames, 1);

lon(1:frameStart) = lonTrack(1);
lat(1:frameStart) = latTrack(1);

lon(frameStart:frameEnd) = lonTrack;
lat(frameStart:frameEnd) = latTrack;

lon(frameEnd:end) = lonTrack(end);
lat(frameEnd:end) = latTrack(end);

hd = computeHeadingSeriesLocal(lon, lat);
end

function ship = createFishingBoatLocal(name, refLon, refLat, kCross, primaryMask, secondaryMask, latRange, lonRange, color, sizeScale, nFrames)
latMean = mean(latRange);
lonScale = cosd(latMean);
mainScale = max(diff(lonRange), diff(latRange));

[tVec, nVec] = tangentNormalFromPathLocal(refLon, refLat, kCross, lonScale);
base = [refLon(kCross), refLat(kCross)];

% 渔船：较晚从右舷（下侧）向左舷（上侧）直线穿越
pStart = base - 0.016 * mainScale * nVec + 0.004 * mainScale * tVec;
pEnd   = base + 0.014 * mainScale * nVec - 0.003 * mainScale * tVec;

lonPath = linspace(pStart(1), pEnd(1), 120)';
latPath = linspace(pStart(2), pEnd(2), 120)';

[lonPath, latPath] = snapOnlyInvalidPointsToMaskLocal( ...
    lonPath, latPath, primaryMask, secondaryMask, latRange, lonRange, 6);

[lon, lat] = placeTrackInFramesLocal( ...
    lonPath, latPath, ...
    round(0.56 * nFrames), round(0.86 * nFrames), nFrames);

ship = struct( ...
    'name', name, ...
    'lon', lon, ...
    'lat', lat, ...
    'heading', computeHeadingSeriesLocal(lon, lat), ...
    'color', color, ...
    'sizeScale', sizeScale);
end
function ship = createHeadOnShipLocal(name, refLon, refLat, kMeet, primaryMask, secondaryMask, latRange, lonRange, color, sizeScale, nFrames)
latMean = mean(latRange);
lonScale = cosd(latMean);
mainScale = max(diff(lonRange), diff(latRange));

[tVec, nVec] = tangentNormalFromPathLocal(refLon, refLat, kMeet, lonScale);
base = [refLon(kMeet), refLat(kMeet)];

% 对遇船位于航道外上侧，在本船出港池后相遇
pStart = base + 0.020 * mainScale * tVec + 0.018 * mainScale * nVec;
pEnd   = base - 0.010 * mainScale * tVec + 0.008 * mainScale * nVec;

lonPath = linspace(pStart(1), pEnd(1), 90)';
latPath = linspace(pStart(2), pEnd(2), 90)';

[lonPath, latPath] = snapOnlyInvalidPointsToMaskLocal( ...
    lonPath, latPath, primaryMask, secondaryMask, latRange, lonRange, 8);

[lon, lat] = placeTrackInFramesLocal( ...
    lonPath, latPath, ...
    round(0.12 * nFrames), round(0.34 * nFrames), nFrames);

ship = struct( ...
    'name', name, ...
    'lon', lon, ...
    'lat', lat, ...
    'heading', computeHeadingSeriesLocal(lon, lat), ...
    'color', color, ...
    'sizeScale', sizeScale);
end

function ship = createTargetShip3Local(name, refLon, refLat, kOver, primaryMask, secondaryMask, latRange, lonRange, color, sizeScale, nFrames)
idxStart = max(1, kOver - 2);
idxEnd   = min(numel(refLon), kOver + 20);

[lon, lat, hd] = buildReferenceTrajectoryLocal( ...
    refLon, refLat, idxStart, idxEnd, 1, round(0.97 * nFrames), ...
    0.0, primaryMask, secondaryMask, latRange, lonRange, nFrames);

ship = struct( ...
    'name', name, ...
    'lon', lon, ...
    'lat', lat, ...
    'heading', hd, ...
    'color', color, ...
    'sizeScale', sizeScale);
end

function hd = computeHeadingSeriesLocal(lon, lat)
n = numel(lon);
hd = zeros(n, 1);
for k = 1:n
    hd(k) = headingFromPathLocal(lon, lat, k);
end
hd = smoothHeadingLocal(hd);
end

function [lon, lat] = placeTrackInFramesLocal(lonPath, latPath, frameStart, frameEnd, nFrames)
frameStart = max(1, min(nFrames, frameStart));
frameEnd = max(frameStart, min(nFrames, frameEnd));

trackLen = max(2, frameEnd - frameStart + 1);
[lonTrack, latTrack] = resamplePolylineByCountLinearLocal(lonPath, latPath, trackLen);

lon = nan(nFrames, 1);
lat = nan(nFrames, 1);

lon(1:frameStart) = lonTrack(1);
lat(1:frameStart) = latTrack(1);

lon(frameStart:frameEnd) = lonTrack;
lat(frameStart:frameEnd) = latTrack;

lon(frameEnd:end) = lonTrack(end);
lat(frameEnd:end) = latTrack(end);
end

function [wp1, wp2] = buildHeadAvoidWaypointsLocal(pathLon, pathLat, kHead, primaryMask, secondaryMask, freeMask, latRange, lonRange, cfg)
% 对遇：靠近会遇时才大幅右舷避让
mainScale = max(diff(lonRange), diff(latRange));
latMean = mean(latRange);
lonScale = cosd(latMean);

k1 = max(2, kHead + 3);
k2 = min(numel(pathLon), kHead + 18);

[tVec1, nVec1] = tangentNormalFromPathLocal(pathLon, pathLat, k1, lonScale);
[tVec2, nVec2] = tangentNormalFromPathLocal(pathLon, pathLat, k2, lonScale);

base1 = [pathLon(k1), pathLat(k1)];
base2 = [pathLon(k2), pathLat(k2)];

cand1 = base1 + 0.0035 * mainScale * tVec1 + 0.0115 * cfg.headAvoidSign * nVec1;
cand2 = base2 + 0.0110 * mainScale * tVec2 + 0.0090 * cfg.headAvoidSign * nVec2;

wp1 = snapGeoPointToMasksLocal(cand1, primaryMask, secondaryMask, freeMask, latRange, lonRange, cfg.snapRadius);
wp2 = snapGeoPointToMasksLocal(cand2, primaryMask, secondaryMask, freeMask, latRange, lonRange, cfg.snapRadius);
end
function [lonOut, latOut] = fixHarborExitLocal(lonIn, latIn, primaryMask, secondaryMask, latRange, lonRange, idxAnchor)
% 出港池段按“竖直出港 + 斜向接入航道”的两段式路径重建，强力消除出口抖动
lonOut = lonIn(:);
latOut = latIn(:);

n = numel(lonOut);
if n < 10
    return;
end

idxAnchor = max(10, min(n - 2, idxAnchor));
p1 = [lonOut(1), latOut(1)];
p3 = [lonOut(idxAnchor), latOut(idxAnchor)];

iElbow = max(4, round(0.58 * idxAnchor));
lonCol = mean(lonOut(1:max(3, round(0.12 * idxAnchor))));
elbow = [lonCol, latOut(iElbow)];
elbow = snapGeoPointToMasksLocal(elbow, primaryMask, secondaryMask, secondaryMask, latRange, lonRange, 10);

if segmentFullyInMaskLocal(p1, elbow, secondaryMask, latRange, lonRange) && ...
   segmentFullyInMaskLocal(elbow, p3, secondaryMask, latRange, lonRange)

    n1 = max(4, round(0.60 * idxAnchor));
    n2 = idxAnchor - n1 + 1;

    lonSeg1 = linspace(p1(1), elbow(1), n1).';
    latSeg1 = linspace(p1(2), elbow(2), n1).';
    lonSeg2 = linspace(elbow(1), p3(1), n2).';
    latSeg2 = linspace(elbow(2), p3(2), n2).';

    lonSeg = [lonSeg1; lonSeg2(2:end)];
    latSeg = [latSeg1; latSeg2(2:end)];
    [lonSeg, latSeg] = resamplePolylineByCountLinearLocal(lonSeg, latSeg, idxAnchor);
else
    rc = astarBetweenGeoLocal(p1, p3, secondaryMask, latRange, lonRange, 45);
    if isempty(rc)
        rc = astarBetweenGeoLocal(p1, p3, primaryMask, latRange, lonRange, 45);
    end
    if isempty(rc) || size(rc, 1) < 2
        return;
    end
    rc = smoothPathGreedyLOSLocal(rc, secondaryMask);
    [latSeg, lonSeg] = rcPathToGeoLocal(rc, size(primaryMask), latRange, lonRange);
    [lonSeg, latSeg] = resamplePolylineByCountLinearLocal(lonSeg, latSeg, idxAnchor);
end

[lonSeg, latSeg] = snapOnlyInvalidPointsToMaskLocal( ...
    lonSeg, latSeg, secondaryMask, primaryMask, latRange, lonRange, 10);

lonOut(1:idxAnchor) = lonSeg;
latOut(1:idxAnchor) = latSeg;
end
function [wp1, wp2] = buildFishingAvoidWaypointsLocal(pathLon, pathLat, kFish, primaryMask, secondaryMask, freeMask, latRange, lonRange, cfg)
% 明确从渔船尾后通过：先略向后保守，再向右舷大幅外偏
mainScale = max(diff(lonRange), diff(latRange));
latMean = mean(latRange);
lonScale = cosd(latMean);

k1 = max(2, kFish + 2);
k2 = min(numel(pathLon), kFish + 18);

[tVec1, nVec1] = tangentNormalFromPathLocal(pathLon, pathLat, k1, lonScale);
[tVec2, nVec2] = tangentNormalFromPathLocal(pathLon, pathLat, k2, lonScale);

base1 = [pathLon(k1), pathLat(k1)];
base2 = [pathLon(k2), pathLat(k2)];

cand1 = base1 - 0.0120 * mainScale * tVec1 + 0.0105 * cfg.fishAvoidSign * nVec1;
cand2 = base2 + 0.0040 * mainScale * tVec2 + 0.0075 * cfg.fishAvoidSign * nVec2;

wp1 = snapGeoPointToMasksLocal(cand1, primaryMask, secondaryMask, freeMask, latRange, lonRange, cfg.snapRadius);
wp2 = snapGeoPointToMasksLocal(cand2, primaryMask, secondaryMask, freeMask, latRange, lonRange, cfg.snapRadius);
end
function [wp1, wp2, wp3] = buildOvertakeWaypointsLocal(pathLon, pathLat, kOver, primaryMask, secondaryMask, freeMask, latRange, lonRange, cfg)
% 追越：进入航道后即明显右舷外偏，保持较长距离后回原航线
mainScale = max(diff(lonRange), diff(latRange));
n = numel(pathLon);
latMean = mean(latRange);
lonScale = cosd(latMean);

k1 = max(2, kOver - 4);
k2 = min(n, kOver + 20);
k3 = min(n, kOver + 64);

[tVec1, nVec1] = tangentNormalFromPathLocal(pathLon, pathLat, k1, lonScale);
[tVec2, nVec2] = tangentNormalFromPathLocal(pathLon, pathLat, k2, lonScale);
[tVec3, nVec3] = tangentNormalFromPathLocal(pathLon, pathLat, k3, lonScale);

base1 = [pathLon(k1), pathLat(k1)];
base2 = [pathLon(k2), pathLat(k2)];
base3 = [pathLon(k3), pathLat(k3)];

cand1 = base1 + 0.0030 * mainScale * tVec1 + 0.0100 * cfg.overAvoidSign * nVec1;
cand2 = base2 + 0.0180 * mainScale * tVec2 + 0.0145 * cfg.overAvoidSign * nVec2;
cand3 = base3 + 0.0200 * mainScale * tVec3 + 0.0060 * cfg.overAvoidSign * nVec3;

wp1 = snapGeoPointToMasksLocal(cand1, primaryMask, secondaryMask, freeMask, latRange, lonRange, cfg.snapRadius);
wp2 = snapGeoPointToMasksLocal(cand2, primaryMask, secondaryMask, freeMask, latRange, lonRange, cfg.snapRadius);
wp3 = snapGeoPointToMasksLocal(cand3, primaryMask, secondaryMask, freeMask, latRange, lonRange, cfg.snapRadius);
end
function wp = chooseOffsetWaypointAdaptiveLocal(pathLon, pathLat, k, preferredSign, ampList, tangList, primaryMask, secondaryMask, freeMask, latRange, lonRange, snapRadius)
latMean = mean(latRange);
lonScale = cosd(latMean);

[tVec, nVec] = tangentNormalFromPathLocal(pathLon, pathLat, k, lonScale);
base = [pathLon(k), pathLat(k)];

bestScore = inf;
wp = base;

for ia = 1:numel(ampList)
    amp = ampList(ia);

    for it = 1:numel(tangList)
        tangShift = tangList(it);

        cand = zeros(6, 2);
        cand(1, :) = base + tangShift * tVec + preferredSign * amp * nVec;
        cand(2, :) = base + 0.75 * tangShift * tVec + preferredSign * 0.85 * amp * nVec;
        cand(3, :) = base + 0.50 * tangShift * tVec + preferredSign * 0.65 * amp * nVec;
        cand(4, :) = base + 0.35 * tangShift * tVec + preferredSign * 0.45 * amp * nVec;
        cand(5, :) = base + 0.25 * tangShift * tVec;
        cand(6, :) = base;

        for i = 1:size(cand, 1)
            p = cand(i, :);
            [r0, c0] = geoToRCIndexLocal(p(1), p(2), size(primaryMask), latRange, lonRange);

            [r1, c1, ok1] = snapRCToAllowedLocal(primaryMask, r0, c0, snapRadius);
            usedPrimary = ok1;

            if ~ok1
                [r1, c1, ok1] = snapRCToAllowedLocal(secondaryMask, r0, c0, snapRadius);
            end
            if ~ok1
                [r1, c1, ok1] = snapRCToAllowedLocal(freeMask, r0, c0, snapRadius);
            end
            if ~ok1
                continue;
            end

            [lat1, lon1] = rc2geoLocal([r1, c1], size(primaryMask), latRange, lonRange);
            p2 = [lon1, lat1];

            sideMetric = dot([(p2(1) - base(1)) * lonScale, p2(2) - base(2)], [nVec(1) * lonScale, nVec(2)]);
            wrongSidePenalty = 8 * double(preferredSign * sideMetric < -1e-10);

            freeScore = localFreedomScoreLocal(r1, c1, primaryMask, 5);

            score = 10 * double(~usedPrimary) + 2.5 * ia + 0.8 * it + wrongSidePenalty - 0.03 * freeScore;

            if score < bestScore
                bestScore = score;
                wp = p2;
            end
        end
    end
end
end

function idxBest = chooseAdaptiveRejoinIndexLocal(pathLon, pathLat, idxMin, idxMax, fromPoint, primaryMask, secondaryMask, latRange, lonRange)
n = numel(pathLon);
idxMin = max(2, min(n, idxMin));
idxMax = max(idxMin, min(n, idxMax));

latMean = mean(latRange);
suffixLen = suffixPathLengthLocal(pathLon, pathLat, latMean);

bestCost = inf;
idxBest = idxMin;

for j = idxMin:2:idxMax
    goal = [pathLon(j), pathLat(j)];

    if hasSafeLineOfSightGeoLocal(fromPoint, goal, primaryMask, latRange, lonRange)
        segCost = hypot((goal(1) - fromPoint(1)) * cosd(latMean), goal(2) - fromPoint(2));
    else
        rc = astarBetweenGeoLocal(fromPoint, goal, primaryMask, latRange, lonRange, 55);
        if isempty(rc)
            rc = astarBetweenGeoLocal(fromPoint, goal, secondaryMask, latRange, lonRange, 55);
        end
        if isempty(rc)
            continue;
        end

        [latSeg, lonSeg] = rcPathToGeoLocal(rc, size(primaryMask), latRange, lonRange);
        segCost = polylineLengthMetricLocal(lonSeg, latSeg, latMean);
    end

    totalCost = segCost + suffixLen(j);

    if totalCost < bestCost
        bestCost = totalCost;
        idxBest = j;
    end
end
end

function suffixLen = suffixPathLengthLocal(lon, lat, latMean)
n = numel(lon);
suffixLen = zeros(n, 1);

for i = (n - 1):-1:1
    suffixLen(i) = suffixLen(i + 1) + hypot((lon(i + 1) - lon(i)) * cosd(latMean), lat(i + 1) - lat(i));
end
end

function L = polylineLengthMetricLocal(lon, lat, latMean)
lon = lon(:);
lat = lat(:);

if numel(lon) < 2
    L = 0;
    return;
end

dx = diff(lon) * cosd(latMean);
dy = diff(lat);
L = sum(hypot(dx, dy));
end

function score = localFreedomScoreLocal(r, c, mask, rad)
[nRows, nCols] = size(mask);
rmin = max(1, r - rad);
rmax = min(nRows, r + rad);
cmin = max(1, c - rad);
cmax = min(nCols, c + rad);
blk = mask(rmin:rmax, cmin:cmax);
score = sum(blk(:));
end

function pathRC = astarBetweenGeoLocal(pStart, pGoal, freeMask, latRange, lonRange, padding)
[r1, c1] = geoToRCIndexLocal(pStart(1), pStart(2), size(freeMask), latRange, lonRange);
[r2, c2] = geoToRCIndexLocal(pGoal(1), pGoal(2), size(freeMask), latRange, lonRange);

[r1, c1, ok1] = snapRCToAllowedLocal(freeMask, r1, c1, 15);
[r2, c2, ok2] = snapRCToAllowedLocal(freeMask, r2, c2, 15);

if ~ok1 || ~ok2
    pathRC = [];
    return;
end

[nRows, nCols] = size(freeMask);

rmin = max(1, min(r1, r2) - padding);
rmax = min(nRows, max(r1, r2) + padding);
cmin = max(1, min(c1, c2) - padding);
cmax = min(nCols, max(c1, c2) + padding);

localMask = freeMask(rmin:rmax, cmin:cmax);
sLocal = [r1 - rmin + 1, c1 - cmin + 1];
gLocal = [r2 - rmin + 1, c2 - cmin + 1];

pathLocal = astarGridLocal(localMask, sLocal, gLocal);
if isempty(pathLocal)
    pathRC = [];
    return;
end

pathRC = pathLocal;
pathRC(:, 1) = pathRC(:, 1) + rmin - 1;
pathRC(:, 2) = pathRC(:, 2) + cmin - 1;
end

function pathRC = astarGridLocal(freeMask, startRC, goalRC)
[nRows, nCols] = size(freeMask);
N = nRows * nCols;

sIdx = sub2ind([nRows, nCols], startRC(1), startRC(2));
gIdx = sub2ind([nRows, nCols], goalRC(1), goalRC(2));

gScore = inf(N, 1);
parent = zeros(N, 1, 'uint32');
state = zeros(N, 1, 'uint8');

heapNode = zeros(N, 1, 'uint32');
heapF = inf(N, 1);
heapPos = zeros(N, 1, 'uint32');
heapSize = uint32(0);

gScore(sIdx) = 0;
h0 = octileHeuristicLocal(startRC(1), startRC(2), goalRC(1), goalRC(2));
[heapNode, heapF, heapPos, heapSize] = heapPushLocal(heapNode, heapF, heapPos, heapSize, uint32(sIdx), h0);
state(sIdx) = 1;

dr = int32([-1 0 1 1 1 0 -1 -1]);
dc = int32([-1 -1 -1 0 1 1 1 0]);
stepCost = [sqrt(2), 1, sqrt(2), 1, sqrt(2), 1, sqrt(2), 1];

found = false;

while heapSize > 0
    [current, heapNode, heapF, heapPos, heapSize] = heapPopLocal(heapNode, heapF, heapPos, heapSize);
    curIdx = double(current);

    if state(curIdx) == 2
        continue;
    end
    state(curIdx) = 2;

    if curIdx == gIdx
        found = true;
        break;
    end

    r = mod(curIdx - 1, nRows) + 1;
    c = floor((curIdx - 1) / nRows) + 1;

    for k = 1:8
        rr = r + double(dr(k));
        cc = c + double(dc(k));

        if rr < 1 || rr > nRows || cc < 1 || cc > nCols
            continue;
        end
        if ~freeMask(rr, cc)
            continue;
        end

        if mod(k, 2) == 1
            if ~freeMask(r, cc) || ~freeMask(rr, c)
                continue;
            end
        end

        neiIdx = sub2ind([nRows, nCols], rr, cc);
        if state(neiIdx) == 2
            continue;
        end

        tentativeG = gScore(curIdx) + stepCost(k);

        if tentativeG < gScore(neiIdx)
            gScore(neiIdx) = tentativeG;
            parent(neiIdx) = uint32(curIdx);

            h = octileHeuristicLocal(rr, cc, goalRC(1), goalRC(2));
            f = tentativeG + 1.01 * h;

            if state(neiIdx) == 0
                [heapNode, heapF, heapPos, heapSize] = heapPushLocal( ...
                    heapNode, heapF, heapPos, heapSize, uint32(neiIdx), f);
                state(neiIdx) = 1;
            else
                [heapNode, heapF, heapPos] = heapDecreaseLocal( ...
                    heapNode, heapF, heapPos, uint32(neiIdx), f);
            end
        end
    end
end

if ~found
    pathRC = [];
    return;
end

idxPath = gIdx;
u = gIdx;

while u ~= sIdx
    u = double(parent(u));
    if u == 0
        pathRC = [];
        return;
    end
    idxPath = [u; idxPath]; %#ok<AGROW>
end

r = mod(idxPath - 1, nRows) + 1;
c = floor((idxPath - 1) / nRows) + 1;
pathRC = [r, c];
end

function h = octileHeuristicLocal(r, c, rg, cg)
dx = abs(c - cg);
dy = abs(r - rg);
h = (sqrt(2) - 1) * min(dx, dy) + max(dx, dy);
end

function [heapNode, heapF, heapPos, heapSize] = heapPushLocal(heapNode, heapF, heapPos, heapSize, node, f)
heapSize = heapSize + 1;
i = double(heapSize);
heapNode(i) = node;
heapF(i) = f;
heapPos(double(node)) = uint32(i);

while i > 1
    p = floor(i / 2);
    if heapF(p) <= heapF(i)
        break;
    end
    [heapNode, heapF, heapPos] = heapSwapLocal(heapNode, heapF, heapPos, i, p);
    i = p;
end
end

function [node, heapNode, heapF, heapPos, heapSize] = heapPopLocal(heapNode, heapF, heapPos, heapSize)
node = heapNode(1);
heapPos(double(node)) = 0;

if heapSize == 1
    heapSize = uint32(0);
    return;
end

heapNode(1) = heapNode(heapSize);
heapF(1) = heapF(heapSize);
heapPos(double(heapNode(1))) = 1;
heapSize = heapSize - 1;

i = 1;
while true
    l = 2 * i;
    r = l + 1;
    smallest = i;

    if l <= heapSize && heapF(l) < heapF(smallest)
        smallest = l;
    end
    if r <= heapSize && heapF(r) < heapF(smallest)
        smallest = r;
    end
    if smallest == i
        break;
    end
    [heapNode, heapF, heapPos] = heapSwapLocal(heapNode, heapF, heapPos, i, smallest);
    i = smallest;
end
end

function [heapNode, heapF, heapPos] = heapDecreaseLocal(heapNode, heapF, heapPos, node, newF)
i = double(heapPos(double(node)));
if i == 0 || newF >= heapF(i)
    return;
end

heapF(i) = newF;

while i > 1
    p = floor(i / 2);
    if heapF(p) <= heapF(i)
        break;
    end
    [heapNode, heapF, heapPos] = heapSwapLocal(heapNode, heapF, heapPos, i, p);
    i = p;
end
end

function [heapNode, heapF, heapPos] = heapSwapLocal(heapNode, heapF, heapPos, i, j)
tmpNode = heapNode(i);
tmpF = heapF(i);

heapNode(i) = heapNode(j);
heapF(i) = heapF(j);

heapNode(j) = tmpNode;
heapF(j) = tmpF;

heapPos(double(heapNode(i))) = uint32(i);
heapPos(double(heapNode(j))) = uint32(j);
end

function pathRC2 = smoothPathGreedyLOSLocal(pathRC, freeMask)
if size(pathRC, 1) <= 2
    pathRC2 = pathRC;
    return;
end

pathRC2 = pathRC(1, :);
i = 1;
n = size(pathRC, 1);

while i < n
    linked = false;

    for j = n:-1:(i + 1)
        if hasSafeLineOfSightLocal(pathRC(i, :), pathRC(j, :), freeMask)
            pathRC2 = [pathRC2; pathRC(j, :)]; %#ok<AGROW>
            i = j;
            linked = true;
            break;
        end
    end

    if ~linked
        pathRC2 = [pathRC2; pathRC(i + 1, :)]; %#ok<AGROW>
        i = i + 1;
    end
end
end

function tf = hasSafeLineOfSightLocal(rc1, rc2, freeMask)
[nRows, nCols] = size(freeMask);

r1 = rc1(1);
c1 = rc1(2);
r2 = rc2(1);
c2 = rc2(2);

nStep = max(abs(r2 - r1), abs(c2 - c1)) * 10 + 1;
rs = linspace(r1, r2, nStep);
cs = linspace(c1, c2, nStep);

tf = true;

for k = 1:nStep
    rr = rs(k);
    cc = cs(k);

    rFloor = floor(rr);
    rCeil = ceil(rr);
    cFloor = floor(cc);
    cCeil = ceil(cc);

    rSet = unique([rFloor, rCeil]);
    cSet = unique([cFloor, cCeil]);

    for ir = 1:numel(rSet)
        for ic = 1:numel(cSet)
            r0 = max(1, min(nRows, rSet(ir)));
            c0 = max(1, min(nCols, cSet(ic)));

            if ~freeMask(r0, c0)
                tf = false;
                return;
            end
        end
    end
end
end

function tf = hasSafeLineOfSightGeoLocal(p1, p2, freeMask, latRange, lonRange)
nStep = 120;
lon = linspace(p1(1), p2(1), nStep);
lat = linspace(p1(2), p2(2), nStep);

tf = true;
for i = 1:nStep
    if ~pointInMaskLocal(lon(i), lat(i), freeMask, latRange, lonRange)
        tf = false;
        return;
    end
end
end

function [lonOut, latOut] = suppressLeadingBacktrackLocal(lonIn, latIn, primaryMask, secondaryMask, latRange, lonRange, nLead)
lonOut = lonIn(:);
latOut = latIn(:);

n = numel(lonOut);
if n < 8
    return;
end

nLead = min([nLead, n - 2, n]);
if nLead < 6
    return;
end

[rLead, cLead] = geoPathToRCPathLocal(lonOut(1:nLead), latOut(1:nLead), size(primaryMask), latRange, lonRange);
rcLead = uniqueConsecutiveRowsLocal([rLead, cLead]);

rcLead2 = smoothPathGreedyLOSLocal(rcLead, primaryMask);
if isempty(rcLead2) || size(rcLead2, 1) < 2
    rcLead2 = smoothPathGreedyLOSLocal(rcLead, secondaryMask);
end

if isempty(rcLead2) || size(rcLead2, 1) < 2
    return;
end

[latTmp, lonTmp] = rcPathToGeoLocal(rcLead2, size(primaryMask), latRange, lonRange);
[lonTmp, latTmp] = resamplePolylineByCountLinearLocal(lonTmp, latTmp, nLead);

[lonTmp, latTmp] = snapOnlyInvalidPointsToMaskLocal( ...
    lonTmp, latTmp, primaryMask, secondaryMask, latRange, lonRange, 10);

lonOut(1:nLead) = lonTmp;
latOut(1:nLead) = latTmp;
end

function [lonOut, latOut] = stabilizeRouteLocal(lonIn, latIn, primaryMask, secondaryMask, latRange, lonRange, win, nPass)
% 保守稳定化：去短折点 + LOS 简化，不做大幅移动平均，避免“平滑后再吸附”造成新抖动
nTarget = numel(lonIn);
lonOut = lonIn(:);
latOut = latIn(:);

for ip = 1:nPass
    [lonOut, latOut] = dezigzagRouteLocal(lonOut, latOut, latRange);

    [r0, c0] = geoPathToRCPathLocal(lonOut, latOut, size(primaryMask), latRange, lonRange);
    rc = uniqueConsecutiveRowsLocal([r0, c0]);

    rc2 = smoothPathGreedyLOSLocal(rc, primaryMask);
    if isempty(rc2) || size(rc2, 1) < 2
        rc2 = smoothPathGreedyLOSLocal(rc, secondaryMask);
    end

    if ~isempty(rc2) && size(rc2, 1) >= 2
        [latTmp, lonTmp] = rcPathToGeoLocal(rc2, size(primaryMask), latRange, lonRange);
        [lonOut, latOut] = resamplePolylineByCountLinearLocal(lonTmp, latTmp, nTarget);
    else
        [lonOut, latOut] = resamplePolylineByCountLinearLocal(lonOut, latOut, nTarget);
    end

    [lonOut, latOut] = snapOnlyInvalidPointsToMaskLocal( ...
        lonOut, latOut, primaryMask, secondaryMask, latRange, lonRange, 8);
end
end


function [lon2, lat2] = dezigzagRouteLocal(lon, lat, latRange)
lon = lon(:);
lat = lat(:);

n = numel(lon);
if n <= 3
    lon2 = lon;
    lat2 = lat;
    return;
end

latMean = mean(latRange);
segLen = hypot(diff(lon) * cosd(latMean), diff(lat));

if isempty(segLen)
    lon2 = lon;
    lat2 = lat;
    return;
end

shortTh = max(1e-6, 2.0 * median(segLen));
keep = true(n, 1);

for i = 2:(n - 1)
    v1 = [(lon(i) - lon(i - 1)) * cosd(latMean), lat(i) - lat(i - 1)];
    v2 = [(lon(i + 1) - lon(i)) * cosd(latMean), lat(i + 1) - lat(i)];

    n1 = norm(v1);
    n2 = norm(v2);

    if n1 < 1e-12 || n2 < 1e-12
        keep(i) = false;
        continue;
    end

    c = dot(v1, v2) / (n1 * n2);

    if c < -0.20 && min(n1, n2) < shortTh
        keep(i) = false;
    end
end

lon2 = lon(keep);
lat2 = lat(keep);

if numel(lon2) < 2
    lon2 = lon;
    lat2 = lat;
end
end

function [rPath, cPath] = geoPathToRCPathLocal(lon, lat, sz, latRange, lonRange)
n = numel(lon);
rPath = zeros(n, 1);
cPath = zeros(n, 1);

for i = 1:n
    [rPath(i), cPath(i)] = geoToRCIndexLocal(lon(i), lat(i), sz, latRange, lonRange);
end
end

function M = uniqueConsecutiveRowsLocal(M)
if isempty(M)
    return;
end

keep = [true; any(diff(M, 1, 1) ~= 0, 2)];
M = M(keep, :);
end

function valid = pointsInMaskLocal(lon, lat, mask, latRange, lonRange)
valid = false(numel(lon), 1);
sz = size(mask);

for k = 1:numel(lon)
    [r, c] = geoToRCIndexLocal(lon(k), lat(k), sz, latRange, lonRange);
    valid(k) = mask(r, c);
end
end

function ok = pointInMaskLocal(lon, lat, mask, latRange, lonRange)
[r, c] = geoToRCIndexLocal(lon, lat, size(mask), latRange, lonRange);
ok = mask(r, c);
end

function [pStart, pEnd] = findValidStraightCrossingLocal(base, tVec, nVec, primaryMask, secondaryMask, latRange, lonRange, mainScale)
spanList = mainScale * [0.010 0.012 0.014 0.016];
shiftList = mainScale * [-0.002 0 0.002 0.004];

for ispan = 1:numel(spanList)
    span = spanList(ispan);
    for ishift = 1:numel(shiftList)
        sh = shiftList(ishift);
        p1 = base - span * nVec + sh * tVec;
        p2 = base + span * nVec - 0.004 * mainScale * tVec + sh * tVec;

        if segmentFullyInMaskLocal(p1, p2, primaryMask, latRange, lonRange) || ...
           segmentFullyInMaskLocal(p1, p2, secondaryMask, latRange, lonRange)
            pStart = p1;
            pEnd = p2;
            return;
        end
    end
end

pStart = base - 0.012 * mainScale * nVec;
pEnd   = base + 0.012 * mainScale * nVec - 0.004 * mainScale * tVec;
end

function tf = segmentFullyInMaskLocal(p1, p2, mask, latRange, lonRange)
nStep = 150;
lon = linspace(p1(1), p2(1), nStep);
lat = linspace(p1(2), p2(2), nStep);

tf = true;
for i = 1:nStep
    if ~pointInMaskLocal(lon(i), lat(i), mask, latRange, lonRange)
        tf = false;
        return;
    end
end
end

function [ok, p] = lineIntersection2DLocal(p1, v1, p2, v2)
A = [v1(:), -v2(:)];
b = (p2(:) - p1(:));

if rank(A) < 2
    ok = false;
    p = [nan, nan];
    return;
end

x = A \ b;
p = (p1(:) + x(1) * v1(:)).';
ok = all(isfinite(p));
end

function pOut = snapGeoPointToMasksLocal(pIn, primaryMask, secondaryMask, tertiaryMask, latRange, lonRange, snapRadius)
[r0, c0] = geoToRCIndexLocal(pIn(1), pIn(2), size(primaryMask), latRange, lonRange);

[r1, c1, ok] = snapRCToAllowedLocal(primaryMask, r0, c0, snapRadius);
if ~ok
    [r1, c1, ok] = snapRCToAllowedLocal(secondaryMask, r0, c0, snapRadius);
end
if ~ok
    [r1, c1, ok] = snapRCToAllowedLocal(tertiaryMask, r0, c0, snapRadius);
end

if ok
    [lat1, lon1] = rc2geoLocal([r1, c1], size(primaryMask), latRange, lonRange);
    pOut = [lon1, lat1];
else
    pOut = pIn;
end
end

function [lonOut, latOut] = snapTrajectoryToMaskLocal(lonIn, latIn, primaryMask, secondaryMask, latRange, lonRange, snapRadius)
lonOut = lonIn(:);
latOut = latIn(:);

for k = 1:numel(lonOut)
    [r0, c0] = geoToRCIndexLocal(lonOut(k), latOut(k), size(primaryMask), latRange, lonRange);

    [r1, c1, ok1] = snapRCToAllowedLocal(primaryMask, r0, c0, snapRadius);
    if ok1
        [latOut(k), lonOut(k)] = rc2geoLocal([r1, c1], size(primaryMask), latRange, lonRange);
    else
        [r2, c2, ok2] = snapRCToAllowedLocal(secondaryMask, r0, c0, snapRadius);
        if ok2
            [latOut(k), lonOut(k)] = rc2geoLocal([r2, c2], size(primaryMask), latRange, lonRange);
        end
    end
end
end

function [lonOut, latOut] = snapOnlyInvalidPointsToMaskLocal(lonIn, latIn, primaryMask, secondaryMask, latRange, lonRange, snapRadius)
lonOut = lonIn(:);
latOut = latIn(:);

valid = pointsInMaskLocal(lonOut, latOut, primaryMask, latRange, lonRange);
badIdx = find(~valid);

for ii = 1:numel(badIdx)
    k = badIdx(ii);
    [r0, c0] = geoToRCIndexLocal(lonOut(k), latOut(k), size(primaryMask), latRange, lonRange);

    [r1, c1, ok1] = snapRCToAllowedLocal(primaryMask, r0, c0, snapRadius);
    if ~ok1
        [r1, c1, ok1] = snapRCToAllowedLocal(secondaryMask, r0, c0, snapRadius);
    end

    if ok1
        [latOut(k), lonOut(k)] = rc2geoLocal([r1, c1], size(primaryMask), latRange, lonRange);
    end
end
end

function [rBest, cBest, ok] = snapRCToAllowedLocal(mask, r0, c0, maxRadius)
[nRows, nCols] = size(mask);

r0 = max(1, min(nRows, r0));
c0 = max(1, min(nCols, c0));

if mask(r0, c0)
    rBest = r0;
    cBest = c0;
    ok = true;
    return;
end

bestD = inf;
rBest = r0;
cBest = c0;
ok = false;

for rad = 1:maxRadius
    rmin = max(1, r0 - rad);
    rmax = min(nRows, r0 + rad);
    cmin = max(1, c0 - rad);
    cmax = min(nCols, c0 + rad);

    found = false;
    for r = rmin:rmax
        for c = cmin:cmax
            if mask(r, c)
                d = hypot(r - r0, c - c0);
                if d < bestD
                    bestD = d;
                    rBest = r;
                    cBest = c;
                    found = true;
                    ok = true;
                end
            end
        end
    end

    if found
        return;
    end
end
end

function [r, c] = geoToRCIndexLocal(lon, lat, sz, latRange, lonRange)
nRows = sz(1);
nCols = sz(2);

c = round((lon - lonRange(1)) / (lonRange(2) - lonRange(1)) * (nCols - 1) + 1);
r = round((latRange(2) - lat) / (latRange(2) - latRange(1)) * (nRows - 1) + 1);

r = max(1, min(nRows, r));
c = max(1, min(nCols, c));
end

function [lat, lon] = rc2geoLocal(rc, sz, latRange, lonRange)
nRows = sz(1);
nCols = sz(2);

r = rc(1);
c = rc(2);

lon = lonRange(1) + (c - 0.5) / nCols * (lonRange(2) - lonRange(1));
lat = latRange(2) - (r - 0.5) / nRows * (latRange(2) - latRange(1));
end

function [latPath, lonPath] = rcPathToGeoLocal(pathRC, sz, latRange, lonRange)
n = size(pathRC, 1);
latPath = zeros(n, 1);
lonPath = zeros(n, 1);

for i = 1:n
    [latPath(i), lonPath(i)] = rc2geoLocal(pathRC(i, :), sz, latRange, lonRange);
end
end

function [tVec, nVec] = tangentNormalFromPathLocal(lon, lat, k, lonScale)
n = numel(lon);
k1 = max(1, k - 6);
k2 = min(n, k + 6);

vMetric = [(lon(k2) - lon(k1)) * lonScale, (lat(k2) - lat(k1))];
nv = norm(vMetric);

if nv < eps
    vMetric = [1, 0];
    nv = 1;
end

tMetric = vMetric / nv;
nMetric = [-tMetric(2), tMetric(1)];

tVec = [tMetric(1) / lonScale, tMetric(2)];
nVec = [nMetric(1) / lonScale, nMetric(2)];

tNorm = hypot(tVec(1) * lonScale, tVec(2));
nNorm = hypot(nVec(1) * lonScale, nVec(2));

tVec = tVec / tNorm;
nVec = nVec / nNorm;
end

function psi = headingFromPathLocal(lon, lat, k)
n = numel(lon);
k1 = max(1, k - 2);
k2 = min(n, k + 2);
psi = atan2(lat(k2) - lat(k1), lon(k2) - lon(k1));
end

function hd = smoothHeadingLocal(hd)
hd = unwrap(hd);
hd = smoothdata(hd, 'movmean', 7);
end

function [vx, vy] = fusiformVerticesLocal(lon, lat, psi, L, W)
body = [ ...
     0.50 * L,  0.00 * W; ...
     0.25 * L,  0.22 * W; ...
     0.00 * L,  0.32 * W; ...
    -0.28 * L,  0.22 * W; ...
    -0.50 * L,  0.00 * W; ...
    -0.28 * L, -0.22 * W; ...
     0.00 * L, -0.32 * W; ...
     0.25 * L, -0.22 * W];

R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
rotBody = (R * body')';

vx = rotBody(:, 1) + lon;
vy = rotBody(:, 2) + lat;
end

function [dxLab, dyLab] = getShipLabelOffsetLocal(i, lonW, latH)
switch i
    case 1
        dxLab = -0.008 * lonW;
        dyLab = 0.018 * latH;
    case 2
        dxLab = 0.012 * lonW;
        dyLab = 0.020 * latH;
    case 3
        dxLab = 0.015 * lonW;
        dyLab = -0.020 * latH;
    otherwise
        dxLab = 0.012 * lonW;
        dyLab = 0.015 * latH;
end
end

function drawBinaryMapGeoLocal(ax, G, latRange, lonRange, drawGrid)
[nRows, nCols] = size(G);

imagesc(ax, [lonRange(1), lonRange(2)], [latRange(1), latRange(2)], flipud(double(~G)));
set(ax, 'YDir', 'normal');
colormap(ax, gray(256));
caxis(ax, [0 1]);
hold(ax, 'on');
box(ax, 'on');

if drawGrid
    lonLines = linspace(lonRange(1), lonRange(2), nCols);
    latLines = linspace(latRange(1), latRange(2), nRows);

    for i = 1:numel(lonLines)
        plot(ax, [lonLines(i), lonLines(i)], [latRange(1), latRange(2)], '-', ...
            'Color', [0.85 0.85 0.85], 'LineWidth', 0.15);
    end

    for i = 1:numel(latLines)
        plot(ax, [lonRange(1), lonRange(2)], [latLines(i), latLines(i)], '-', ...
            'Color', [0.85 0.85 0.85], 'LineWidth', 0.15);
    end
end

xt = linspace(lonRange(1), lonRange(2), 4);
yt = linspace(latRange(1), latRange(2), 4);

set(ax, 'XTick', xt, 'YTick', yt);
set(ax, 'XTickLabel', compose('%.4f', xt));
set(ax, 'YTickLabel', compose('%.4f', yt));
set(ax, 'FontName', 'Times New Roman');

xlabel(ax, 'Longitude/(°)', 'FontName', 'Times New Roman');
ylabel(ax, 'Latitude/(°)', 'FontName', 'Times New Roman');

axis(ax, 'image');
axis(ax, 'tight');
end

function [lonNew, latNew] = resamplePolylineByCountLinearLocal(lon, lat, nOut)
lon = lon(:);
lat = lat(:);

if numel(lon) ~= numel(lat)
    error('lon and lat must have same length.');
end

valid = isfinite(lon) & isfinite(lat);
lon = lon(valid);
lat = lat(valid);

if isempty(lon)
    error('Input path is empty after removing invalid points.');
end

if numel(lon) == 1
    lonNew = repmat(lon(1), nOut, 1);
    latNew = repmat(lat(1), nOut, 1);
    return;
end

keep = [true; abs(diff(lon)) > eps(max(abs(lon)) + 1) | abs(diff(lat)) > eps(max(abs(lat)) + 1)];
lon = lon(keep);
lat = lat(keep);

if numel(lon) == 1
    lonNew = repmat(lon(1), nOut, 1);
    latNew = repmat(lat(1), nOut, 1);
    return;
end

d = hypot(diff(lon), diff(lat));
s = [0; cumsum(d)];

[sUnique, ia] = unique(s, 'stable');
lon = lon(ia);
lat = lat(ia);

if numel(sUnique) == 1 || sUnique(end) <= 0
    lonNew = repmat(lon(1), nOut, 1);
    latNew = repmat(lat(1), nOut, 1);
    return;
end

sNew = linspace(0, sUnique(end), nOut)';
lonNew = interp1(sUnique, lon, sNew, 'linear');
latNew = interp1(sUnique, lat, sNew, 'linear');
end