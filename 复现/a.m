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
cfg.saveObsMat           = false;

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
cfg.showShipTrails       = true;
cfg.showShipLabels       = false;

cfg.zoomLonFraction      = 0.12;
cfg.zoomLatFraction      = 0.12;
cfg.zoomAheadFrac        = 0.020;

cfg.ownShipLengthFracLon    = 0.0030;
cfg.ownShipWidthFracLat     = 0.0018;
cfg.targetShipLengthFracLon = 0.0026;
cfg.targetShipWidthFracLat  = 0.0016;
cfg.fishingBoatScale        = 0.75;

cfg.nominalDenseN        = 520;
cfg.gridSizeMeter        = 57;

% 障碍物安全裕度（单位：栅格）
cfg.minObstacleClearanceCells = 3.2;
cfg.clearanceSearchRadiusCells = 14;
cfg.deJitterPasses = 2;
cfg.deJitterMedianWin = 7;
cfg.deJitterMeanWin = 13;
cfg.headLeadFrames = 38;         % 直航船提前量
cfg.fishLeadFrames = -36;        % 渔船进一步延后出现
cfg.overLeadFrames = 30;         % 3号船提前量
cfg.showDWA = true;
cfg.dwaRangeFrac = 0.060;
cfg.dwaSweepHz = 0.45;
cfg.dwaNumRays = 17;
cfg.dwaMinFeasibleFrac = 0.25;

% ===== 关键优化（对应你的 4 个问题） =====
cfg.headFrac             = 0.20;   % 直航船更早会遇
cfg.fishFrac             = 0.40;   % 渔船在进入航道初期横穿
cfg.overFrac             = 0.58;   % 追越在中后段开始

cfg.startJitterFixFrac   = 0.12;   % 出港初段抖动修复段比例
cfg.startVerticalWeight  = 0.75;   % 初段竖直出港约束强度

cfg.headAvoidAmpFrac     = 0.0078;
cfg.fishAvoidAmpFrac     = 0.0090;
cfg.overAvoidAmpFrac     = 0.0105;
cfg.shipBlockRadiusFrac  = 0.012; % DWA将船舶视作不可航区域

cfg.headAvoidSign        = -1;     % 右舷外偏
cfg.fishAvoidSign        = -1;
cfg.overAvoidSign        = -1;

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
if ~isfield(S, 'G') || ~isfield(S, 'pathLat') || ~isfield(S, 'pathLon')
    error('Input mat must include G, pathLat, pathLon.');
end

G = logical(S.G);
pathLatRaw = S.pathLat(:);
pathLonRaw = S.pathLon(:);

if isfield(S, 'mapInfo')
    mapInfo = S.mapInfo;
else
    mapInfo = struct();
end
if ~isfield(mapInfo, 'latRange'), mapInfo.latRange = [35.9690, 36.0614]; end
if ~isfield(mapInfo, 'lonRange'), mapInfo.lonRange = [120.2090, 120.4070]; end

latRange = mapInfo.latRange;
lonRange = mapInfo.lonRange;
lonW = diff(lonRange);
latH = diff(latRange);
latMean = mean(latRange);
lonScale = cosd(latMean);

%% ===================== Route Build =====================
[pathLonNomDense, pathLatNomDense] = resamplePolylineByCountLinearLocal(pathLonRaw, pathLatRaw, cfg.nominalDenseN);
[pathLonNomDense, pathLatNomDense] = smoothRouteLocal(pathLonNomDense, pathLatNomDense, 5);
[pathLonNomDense, pathLatNomDense] = clampRouteToRangeLocal(pathLonNomDense, pathLatNomDense, lonRange, latRange);
[pathLonNomDense, pathLatNomDense] = enforceObstacleClearanceLocal(pathLonNomDense, pathLatNomDense, G, latRange, lonRange, cfg.minObstacleClearanceCells, cfg.clearanceSearchRadiusCells);
[pathLonNomDense, pathLatNomDense] = deJitterRouteLocal(pathLonNomDense, pathLatNomDense, cfg.deJitterPasses, cfg.deJitterMedianWin, cfg.deJitterMeanWin);
[pathLonNomDense, pathLatNomDense] = enforceObstacleClearanceLocal(pathLonNomDense, pathLatNomDense, G, latRange, lonRange, cfg.minObstacleClearanceCells, cfg.clearanceSearchRadiusCells);

[pathLonActDense, pathLatActDense, routeInfo] = buildActualRouteLocal(pathLonNomDense, pathLatNomDense, cfg, lonScale, lonW, latH);
[pathLonActDense, pathLatActDense] = fixHarborExitJitterLocal(pathLonActDense, pathLatActDense, cfg);
[pathLonActDense, pathLatActDense] = smoothRouteLocal(pathLonActDense, pathLatActDense, 7);
[pathLonActDense, pathLatActDense] = clampRouteToRangeLocal(pathLonActDense, pathLatActDense, lonRange, latRange);
[pathLonActDense, pathLatActDense] = enforceObstacleClearanceLocal(pathLonActDense, pathLatActDense, G, latRange, lonRange, cfg.minObstacleClearanceCells, cfg.clearanceSearchRadiusCells);
[pathLonActDense, pathLatActDense] = deJitterRouteLocal(pathLonActDense, pathLatActDense, cfg.deJitterPasses, cfg.deJitterMedianWin, cfg.deJitterMeanWin);
[pathLonActDense, pathLatActDense] = fixHarborExitJitterLocal(pathLonActDense, pathLatActDense, cfg);
[pathLonActDense, pathLatActDense] = enforceObstacleClearanceLocal(pathLonActDense, pathLatActDense, G, latRange, lonRange, cfg.minObstacleClearanceCells, cfg.clearanceSearchRadiusCells);
[pathLonActDense, pathLatActDense] = stabilizeHarborStartLocal(pathLonActDense, pathLatActDense, cfg);

[pathLonNom, pathLatNom] = resamplePolylineByCountLinearLocal(pathLonNomDense, pathLatNomDense, cfg.nFrames);
[pathLonAct, pathLatAct] = resamplePolylineByCountLinearLocal(pathLonActDense, pathLatActDense, cfg.nFrames);
[pathLonNom, pathLatNom] = enforceObstacleClearanceLocal(pathLonNom, pathLatNom, G, latRange, lonRange, max(2.6, cfg.minObstacleClearanceCells-0.4), cfg.clearanceSearchRadiusCells);
[pathLonAct, pathLatAct] = enforceObstacleClearanceLocal(pathLonAct, pathLatAct, G, latRange, lonRange, cfg.minObstacleClearanceCells, cfg.clearanceSearchRadiusCells);
[pathLonAct, pathLatAct] = dampOvertakeJitterLocal(pathLonAct, pathLatAct, pathLonNom, pathLatNom, round(cfg.overFrac*cfg.nFrames));

% 起始阶段：严格跟随既定航线，避免一开始偏离
nStartLock = min(round(0.16 * cfg.nFrames), numel(pathLonAct));
pathLonAct(1:nStartLock) = pathLonNom(1:nStartLock);
pathLatAct(1:nStartLock) = pathLatNom(1:nStartLock);
[pathLonAct, pathLatAct] = enforceObstacleClearanceLocal(pathLonAct, pathLatAct, G, latRange, lonRange, cfg.minObstacleClearanceCells, cfg.clearanceSearchRadiusCells);

metrics = computeRouteMetricsLocal(pathLonNom, pathLatNom, pathLonAct, pathLatAct, cfg);

fprintf('\n===== Route Metrics =====\n');
fprintf('Path length/(m): %.2f\n', metrics.pathLengthM);
fprintf('Heading change difference/(°): %.2f\n', metrics.headingChangeDiffDeg);
fprintf('Linear velocity oscillation count/(times): %d\n', metrics.linearVelocityOscCount);
fprintf('Max angular velocity oscillation amplitude/(deg·s-1): %.4f\n', metrics.maxAngularVelOscAmpDegS);
fprintf('Simulation steps: %d\n', metrics.simulationSteps);

%% ===================== Target Ships =====================
ships = createScenarioShipsLocal(pathLonAct, pathLatAct, pathLonNom, pathLatNom, cfg, lonScale, lonW, latH);
nShips = numel(ships);

if cfg.saveObsMat
    save(cfg.outputObsMat, 'ships', 'pathLonNomDense', 'pathLatNomDense', 'pathLonActDense', 'pathLatActDense', 'routeInfo', 'metrics');
end

%% ===================== Figure and Axes =====================
fig = figure('Color', 'w', 'Name', 'Inset Route Animation Optimized', 'Units', 'pixels', 'Position', [60 60 1500 820]);
axMain = axes('Parent', fig, 'Position', cfg.leftAxesPos);
axZoom = axes('Parent', fig, 'Position', cfg.rightAxesPos);

drawBinaryMapGeoLocal(axMain, G, latRange, lonRange, cfg.showGrid);
drawBinaryMapGeoLocal(axZoom, G, latRange, lonRange, cfg.showGrid);
hold(axMain, 'on'); hold(axZoom, 'on');

%% ===================== Static Graphics =====================
plot(axMain, pathLonNom, pathLatNom, ':', 'Color', cfg.nominalColor, 'LineWidth', cfg.nominalWidth);
plot(axZoom, pathLonNom, pathLatNom, ':', 'Color', cfg.nominalColor, 'LineWidth', cfg.nominalWidth);

plot(axMain, pathLonNom(1), pathLatNom(1), 'gp', 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g', 'MarkerSize', cfg.startGoalMarkerSize);
plot(axMain, pathLonNom(end), pathLatNom(end), 'rp', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', 'MarkerSize', cfg.startGoalMarkerSize);
plot(axZoom, pathLonNom(1), pathLatNom(1), 'gp', 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'g', 'MarkerSize', cfg.startGoalMarkerSize);
plot(axZoom, pathLonNom(end), pathLatNom(end), 'rp', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', 'MarkerSize', cfg.startGoalMarkerSize);

%% ===================== Moving Zoom Box =====================
boxW = cfg.zoomLonFraction * lonW;
boxH = cfg.zoomLatFraction * latH;
initBox = [pathLonAct(1) - 0.5 * boxW, pathLatAct(1) - 0.5 * boxH, boxW, boxH];

hRectMain = rectangle(axMain, 'Position', initBox, 'EdgeColor', cfg.zoomRectColor, 'LineWidth', cfg.zoomRectWidth, 'LineStyle', '--');
xlim(axZoom, [initBox(1), initBox(1) + initBox(3)]);
ylim(axZoom, [initBox(2), initBox(2) + initBox(4)]);

%% ===================== Own Ship =====================
hTrailMain = plot(axMain, nan, nan, '-', 'Color', cfg.actualColor, 'LineWidth', cfg.actualWidthMain);
hTrailZoom = plot(axZoom, nan, nan, '-', 'Color', cfg.actualColor, 'LineWidth', cfg.actualWidthZoom);

Lown = cfg.ownShipLengthFracLon * lonW;
Wown = cfg.ownShipWidthFracLat * latH;
psiOwn0 = headingFromPathLocal(pathLonAct, pathLatAct, 1);
[vxOwn0, vyOwn0] = fusiformVerticesLocal(pathLonAct(1), pathLatAct(1), psiOwn0, Lown, Wown);

hOwnShipMain = patch(axMain, vxOwn0, vyOwn0, cfg.ownShipColor, 'EdgeColor', cfg.ownShipColor, 'LineWidth', 0.7);
hOwnShipZoom = patch(axZoom, vxOwn0, vyOwn0, cfg.ownShipColor, 'EdgeColor', cfg.ownShipColor, 'LineWidth', 0.7);

if cfg.showShipLabels
    hOwnTextMain = text(axMain, pathLonAct(1), pathLatAct(1), 'Experimental ship', 'FontName', 'Times New Roman', 'FontSize', cfg.expLabelFontSize, 'Color', 'k', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'Clipping', 'on');
    hOwnTextZoom = text(axZoom, pathLonAct(1), pathLatAct(1), 'Experimental ship', 'FontName', 'Times New Roman', 'FontSize', cfg.expLabelFontSize, 'Color', 'k', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'Clipping', 'on');
else
    hOwnTextMain = gobjects(1);
    hOwnTextZoom = gobjects(1);
end

% DWA 扫测可视化（左侧主图）：多条“可通行”绿色射线
if cfg.showDWA
    hDwaRays = gobjects(cfg.dwaNumRays, 1);
    for ii = 1:cfg.dwaNumRays
        hDwaRays(ii) = plot(axMain, nan, nan, '-', 'Color', [0.10 0.78 0.18], 'LineWidth', 0.9);
    end
else
    hDwaRays = gobjects(1);
end

%% ===================== Target Ships =====================
Ltar = cfg.targetShipLengthFracLon * lonW;
Wtar = cfg.targetShipWidthFracLat * latH;

hShipMain = gobjects(nShips, 1);
hShipZoom = gobjects(nShips, 1);
hShipTrailMain = gobjects(nShips, 1);
hShipTrailZoom = gobjects(nShips, 1);
hShipTextMain = gobjects(nShips, 1);

for i = 1:nShips
    [vx, vy] = fusiformVerticesLocal(ships(i).lon(1), ships(i).lat(1), ships(i).heading(1), Ltar * ships(i).sizeScale, Wtar * ships(i).sizeScale);
    hShipMain(i) = patch(axMain, vx, vy, ships(i).color, 'EdgeColor', ships(i).color, 'LineWidth', 0.7);
    hShipZoom(i) = patch(axZoom, vx, vy, ships(i).color, 'EdgeColor', ships(i).color, 'LineWidth', 0.7);

    if cfg.showShipTrails
        if strcmpi(ships(i).name, 'Fishing boat'), trailColor = cfg.fishingTrailColor; else, trailColor = ships(i).color; end
        hShipTrailMain(i) = plot(axMain, nan, nan, '-', 'Color', trailColor, 'LineWidth', cfg.shipTrailWidthMain);
        hShipTrailZoom(i) = plot(axZoom, nan, nan, '-', 'Color', trailColor, 'LineWidth', cfg.shipTrailWidthZoom);
    end

    if cfg.showShipLabels
        [dxLab, dyLab] = getShipLabelOffsetLocal(i, lonW, latH);
        hShipTextMain(i) = text(axMain, ships(i).lon(1) + dxLab, ships(i).lat(1) + dyLab, ships(i).name, 'FontName', 'Times New Roman', 'FontSize', cfg.labelFontSize, 'Color', 'k', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Clipping', 'on');
    end
end

if cfg.showShipLabels
    hShipTextZoom = text(axZoom, ships(1).lon(1), ships(1).lat(1), ships(1).name, 'FontName', 'Times New Roman', 'FontSize', cfg.labelFontSize, 'Color', 'k', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', 'Clipping', 'on');
else
    hShipTextZoom = gobjects(1);
end

%% ===================== Video / GIF =====================
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
    x0 = max(lonRange(1), min(xCenter - 0.5 * boxW, lonRange(2) - boxW));
    y0 = max(latRange(1), min(yCenter - 0.5 * boxH, latRange(2) - boxH));

    set(hRectMain, 'Position', [x0, y0, boxW, boxH]);
    xlim(axZoom, [x0, x0 + boxW]); ylim(axZoom, [y0, y0 + boxH]);

    if cfg.showShipLabels
        set(hOwnTextMain, 'Position', [pathLonAct(k) - 0.025 * lonW, pathLatAct(k) + 0.028 * latH, 0]);
        set(hOwnTextZoom, 'Position', [x0 + 0.36 * boxW, y0 + 0.68 * boxH, 0]);
    end

    if cfg.showDWA
        scanR = cfg.dwaRangeFrac * max(lonW, latH);
        sweep = 0.16 * sin(2*pi*cfg.dwaSweepHz*(k/cfg.fps));
        angFan = psiOwn + sweep + linspace(-0.62, 0.62, cfg.dwaNumRays);
        shipLonNow = arrayfun(@(ss) ss.lon(k), ships).';
        shipLatNow = arrayfun(@(ss) ss.lat(k), ships).';
        shipBlockR = cfg.shipBlockRadiusFrac * max(lonW, latH);
        for ir = 1:cfg.dwaNumRays
            [xe, ye, fracFree] = traceRayFeasibleLocal( ...
                pathLonAct(k), pathLatAct(k), angFan(ir), scanR, ...
                G, latRange, lonRange, shipLonNow, shipLatNow, shipBlockR, lonScale);
            if fracFree >= cfg.dwaMinFeasibleFrac
                set(hDwaRays(ir), 'XData', [pathLonAct(k), xe], 'YData', [pathLatAct(k), ye]);
            else
                set(hDwaRays(ir), 'XData', nan, 'YData', nan);
            end
        end
    end

    nearestShipIdx = 1; nearestDist = inf;
    for i = 1:nShips
        [vx, vy] = fusiformVerticesLocal(ships(i).lon(k), ships(i).lat(k), ships(i).heading(k), Ltar * ships(i).sizeScale, Wtar * ships(i).sizeScale);
        set(hShipMain(i), 'XData', vx, 'YData', vy);
        set(hShipZoom(i), 'XData', vx, 'YData', vy);

        if cfg.showShipTrails
            set(hShipTrailMain(i), 'XData', ships(i).lon(1:k), 'YData', ships(i).lat(1:k));
            set(hShipTrailZoom(i), 'XData', ships(i).lon(1:k), 'YData', ships(i).lat(1:k));
        end

        d = hypot((ships(i).lon(k) - pathLonAct(k)) * lonScale, (ships(i).lat(k) - pathLatAct(k)));
        if d < nearestDist, nearestDist = d; nearestShipIdx = i; end

        if cfg.showShipLabels
            [dxLab, dyLab] = getShipLabelOffsetLocal(i, lonW, latH);
            set(hShipTextMain(i), 'Position', [ships(i).lon(k) + dxLab, ships(i).lat(k) + dyLab, 0]);
        end
    end

    if cfg.showShipLabels
        [dxLabZ, dyLabZ] = getShipLabelOffsetLocal(nearestShipIdx, boxW, boxH);
        set(hShipTextZoom, 'String', ships(nearestShipIdx).name, 'Position', [ships(nearestShipIdx).lon(k) + 0.12 * dxLabZ, ships(nearestShipIdx).lat(k) + 0.12 * dyLabZ, 0]);
    end

    drawnow;
    frame = getframe(fig);

    if cfg.saveMP4, writeVideo(v, frame); end

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

if cfg.saveMP4, close(v); end

fprintf('Animation finished.\n');
if cfg.saveMP4, fprintf('MP4 saved: %s\n', cfg.outputMP4); end
if cfg.saveGIF, fprintf('GIF saved: %s\n', cfg.outputGIF); end
if cfg.saveObsMat, fprintf('Target ship trajectories saved: %s\n', cfg.outputObsMat); end

%% ===================== Local Functions =====================
function [lonAct, latAct, info] = buildActualRouteLocal(lonNom, latNom, cfg, lonScale, lonW, latH)
n = numel(lonNom);

kHead = max(10, round(cfg.headFrac * n));
kFish = max(kHead + 20, round(cfg.fishFrac * n));
kOver = max(kFish + 10, round(cfg.overFrac * n));
mainScale = max([range(lonNom), range(latNom)]);
[ship3LonVirt, ship3LatVirt] = buildTargetShip3TrackFromNominalLocal(lonNom, latNom, kOver, n);
dToShip3 = hypot((lonNom - ship3LonVirt) * lonScale, (latNom - ship3LatVirt));
[~, kCPA3] = min(dToShip3);
% DWA检测触发：检测到目标船3进入前向可探测扇区后立即启动避让
dwaDetectRange = cfg.dwaRangeFrac * max([range(lonNom), range(latNom)]) * 1.15;
kDetect3 = findDwaDetectionIndexLocal(lonNom, latNom, ship3LonVirt, ship3LatVirt, lonScale, dwaDetectRange, 0.68);
if ~isnan(kDetect3)
    kTrig3 = max(2, kDetect3 + 1);
else
    % 回退：若DWA未检测到，则用更早CPA触发
    kTrig3 = max(2, kCPA3 - 18);
end

lonAct = lonNom;
latAct = latNom;

latMean = mean(latNom);

% 1) 出港后不久为直航会遇预留避让轨迹（提前启动、提前回正）
[lonAct, latAct] = applyLateralBumpLocal(lonAct, latAct, kHead - 16, kHead + 20, cfg.headAvoidAmpFrac * mainScale, cfg.headAvoidSign, lonScale);

% 2) 进入航道阶段与渔船交叉会遇：扩大避让区间与幅度
[lonAct, latAct] = applyLateralBumpLocal(lonAct, latAct, kFish - 10, kFish + 22, cfg.fishAvoidAmpFrac * mainScale, cfg.fishAvoidSign, lonScale);

% 3) 追越目标船3：按“实时位置+提前量”触发局部避让
[lonAct, latAct] = applyLateralBumpLocal(lonAct, latAct, kTrig3 - 4, min(n, kTrig3 + 10), cfg.overAvoidAmpFrac * mainScale, cfg.overAvoidSign, lonScale);

% 平滑避免折线感
[lonAct, latAct] = smoothRouteLocal(lonAct, latAct, 9);
[lonAct, latAct] = dampFishJitterLocal(lonAct, latAct, lonNom, latNom, kFish);
[lonAct, latAct] = dampOvertakeJitterLocal(lonAct, latAct, lonNom, latNom, kOver);
[lonAct, latAct] = followNominalOutsideEventsLocal(lonAct, latAct, lonNom, latNom, [kHead-20, kHead+26; kFish-12, kFish+20; kTrig3-6, kTrig3+14]);

info = struct('kHead', kHead, 'kFish', kFish, 'kOver', kOver, 'kCPA3', kCPA3, 'kDetect3', kDetect3, 'kTrig3', kTrig3, 'latMean', latMean, 'lonW', lonW, 'latH', latH);
end

function [lonOut, latOut] = applyLateralBumpLocal(lon, lat, i1, i2, amp, signLR, lonScale)
n = numel(lon);
i1 = max(2, min(n - 1, i1));
i2 = max(i1 + 2, min(n, i2));
lonOut = lon;
latOut = lat;

idx = (i1:i2)';
phi = (idx - i1) / max(1, (i2 - i1));
win = sin(pi * phi).^1.35; % 更平滑且峰值更集中

for ii = 1:numel(idx)
    k = idx(ii);
    [~, nVec] = tangentNormalFromPathLocal(lon, lat, k, lonScale);
    lonOut(k) = lonOut(k) + signLR * amp * win(ii) * nVec(1);
    latOut(k) = latOut(k) + signLR * amp * win(ii) * nVec(2);
end
end

function [lonOut, latOut] = fixHarborExitJitterLocal(lonIn, latIn, cfg)
% 出港初段抖动修复：将初段投影到“近似竖直离港”并与原轨迹平滑融合
lonOut = lonIn(:); latOut = latIn(:);
n = numel(lonOut);

nFix = max(12, round(cfg.startJitterFixFrac * n));
nFix = min(nFix, n - 5);

lonRef = median(lonOut(1:max(5, round(0.2*nFix))));
lat0 = latOut(1);
lat1 = latOut(nFix);

latSeg = linspace(lat0, lat1, nFix)';
lonSeg = lonRef * ones(nFix, 1);

alpha = linspace(1, 0, nFix)'.^1.2;
lonOut(1:nFix) = cfg.startVerticalWeight * (alpha .* lonSeg + (1-alpha).*lonOut(1:nFix)) + (1-cfg.startVerticalWeight) * lonOut(1:nFix);
latOut(1:nFix) = 0.85 * latSeg + 0.15 * latOut(1:nFix);
end

function ships = createScenarioShipsLocal(pathLonAct, pathLatAct, pathLonNom, pathLatNom, cfg, lonScale, lonW, latH)
n = numel(pathLonAct);

kHead = round(cfg.headFrac * n);
kFish = round(cfg.fishFrac * n);
kOver = round(cfg.overFrac * n);

ships = repmat(struct('name', '', 'lon', [], 'lat', [], 'heading', [], 'color', zeros(1,3), 'sizeScale', 1), 3, 1);

% 1) 直航船：严格在既定航线（nominal）上逆行对遇
ships(1) = createHeadOnShipLocal('Head-on ship', pathLonNom, pathLatNom, kHead, cfg.shipColors(1,:), 0.95, n, lonScale, cfg.headLeadFrames);

% 2) 渔船：在本船进入航道时段横穿（时机与本船通过点对齐）
ships(2) = createFishingBoatLocal('Fishing boat', pathLonNom, pathLatNom, kFish, cfg.shipColors(2,:), cfg.fishingBoatScale, n, lonScale, cfg.fishLeadFrames);

% 3) 目标船3：沿既定航线前行且慢速，不牵引本船轨迹
ships(3) = createTargetShip3Local('Target ship 3', pathLonNom, pathLatNom, kOver, cfg.shipColors(3,:), 0.90, n, lonScale, lonW, latH, cfg.overLeadFrames);
end

function ship = createHeadOnShipLocal(name, refLon, refLat, kMeet, color, sizeScale, nFrames, lonScale, leadFrames)
% 直航船沿既定航线逆行，确保与本船正面会遇
idxStart = min(numel(refLon), kMeet + 38);
idxEnd   = max(1, kMeet - 26);

if idxStart > idxEnd
    lonPath = refLon(idxStart:-1:idxEnd);
    latPath = refLat(idxStart:-1:idxEnd);
else
    lonPath = refLon(idxStart:idxEnd);
    latPath = refLat(idxStart:idxEnd);
end

frameCenter = max(1, kMeet - leadFrames);
frameStart = 1;
frameEnd   = min(nFrames, frameCenter + 55);
[lon, lat] = placeTrackInFramesLocal(lonPath, latPath, frameStart, frameEnd, nFrames);

ship = struct('name', name, 'lon', lon, 'lat', lat, 'heading', computeHeadingSeriesLocal(lon, lat), 'color', color, 'sizeScale', sizeScale);
end

function ship = createFishingBoatLocal(name, refLon, refLat, kCross, color, sizeScale, nFrames, lonScale, leadFrames)
% 渔船横穿点与本船在 kCross 处对齐，避免出现时机偏后
[tVec, nVec] = tangentNormalFromPathLocal(refLon, refLat, kCross, lonScale);
base = [refLon(kCross), refLat(kCross)];
mainScale = max([range(refLon), range(refLat)]);

pStart = base - 0.020 * mainScale * nVec + 0.002 * mainScale * tVec;
pEnd   = base + 0.020 * mainScale * nVec - 0.002 * mainScale * tVec;
% 下移一点，避免“渔船太靠上”
pStart = pStart - 0.0065 * mainScale * nVec;
pEnd   = pEnd   - 0.0065 * mainScale * nVec;

lonPath = linspace(pStart(1), pEnd(1), 160)';
latPath = linspace(pStart(2), pEnd(2), 160)';

frameCenter = max(1, kCross + 46 - round(0.20*leadFrames));
frameStart = 1;
frameEnd   = min(nFrames, frameCenter + 48);
[lon, lat] = placeTrackInFramesLocal(lonPath, latPath, frameStart, frameEnd, nFrames);

ship = struct('name', name, 'lon', lon, 'lat', lat, 'heading', computeHeadingSeriesLocal(lon, lat), 'color', color, 'sizeScale', sizeScale);
end

function ship = createTargetShip3Local(name, refLon, refLat, kOver, color, sizeScale, nFrames, lonScale, lonW, latH, leadFrames)
% 目标船3沿既定航线慢速前行（与避让触发模型一致）
[lon, lat] = buildTargetShip3TrackFromNominalLocal(refLon, refLat, kOver, nFrames);

ship = struct('name', name, 'lon', lon, 'lat', lat, 'heading', computeHeadingSeriesLocal(lon, lat), 'color', color, 'sizeScale', sizeScale);
end

function [lon, lat] = buildTargetShip3TrackFromNominalLocal(refLon, refLat, kOver, nFrames)
idxStart = max(1, kOver - 6);
idxEnd   = min(numel(refLon), kOver + 120);
lonPath = refLon(idxStart:idxEnd);
latPath = refLat(idxStart:idxEnd);

% 慢速：全程运行，但有效位移拉长
frameStart = 1;
frameEnd = nFrames;
[lonRaw, latRaw] = placeTrackInFramesLocal(lonPath, latPath, frameStart, frameEnd, nFrames);
slowIdx = round(linspace(1, nFrames, round(0.78 * nFrames)));
lon = lonRaw; lat = latRaw;
lon(1:numel(slowIdx)) = lonRaw(slowIdx);
lat(1:numel(slowIdx)) = latRaw(slowIdx);
lon(numel(slowIdx)+1:end) = lon(numel(slowIdx));
lat(numel(slowIdx)+1:end) = lat(numel(slowIdx));
end

function kDetect = findDwaDetectionIndexLocal(ownLon, ownLat, tarLon, tarLat, lonScale, detectRange, halfFovRad)
% 返回DWA前向扇区首次检测到目标船的索引；未检测到返回 NaN
n = min([numel(ownLon), numel(ownLat), numel(tarLon), numel(tarLat)]);
kDetect = nan;
for k = 2:n
    psi = headingFromPathLocal(ownLon, ownLat, k);
    dx = (tarLon(k) - ownLon(k)) * lonScale;
    dy = (tarLat(k) - ownLat(k));
    r = hypot(dx, dy);
    if r < 1e-12 || r > detectRange
        continue;
    end
    relAng = atan2(dy, dx) - psi;
    relAng = atan2(sin(relAng), cos(relAng));
    if abs(relAng) <= halfFovRad
        kDetect = k;
        return;
    end
end
end

function [lon, lat] = placeTrackInFramesLocal(lonPath, latPath, frameStart, frameEnd, nFrames)
frameStart = max(1, min(nFrames, frameStart));
frameEnd = max(frameStart, min(nFrames, frameEnd));

trackLen = max(2, frameEnd - frameStart + 1);
[lonTrack, latTrack] = resamplePolylineByCountLinearLocal(lonPath, latPath, trackLen);

lon = nan(nFrames, 1); lat = nan(nFrames, 1);
lon(1:frameStart) = lonTrack(1);
lat(1:frameStart) = latTrack(1);
lon(frameStart:frameEnd) = lonTrack;
lat(frameStart:frameEnd) = latTrack;
lon(frameEnd:end) = lonTrack(end);
lat(frameEnd:end) = latTrack(end);
end

function hd = computeHeadingSeriesLocal(lon, lat)
lon = sanitizeRealVectorLocal(lon);
lat = sanitizeRealVectorLocal(lat);

n = numel(lon); hd = zeros(n,1);
for k = 1:n
    hd(k) = headingFromPathLocal(lon, lat, k);
end
hd = unwrap(hd);
hd = smoothdata(hd, 'movmean', 7);
end

function [lonOut, latOut] = smoothRouteLocal(lonIn, latIn, win)
lonOut = smoothdata(lonIn(:), 'movmean', win);
latOut = smoothdata(latIn(:), 'movmean', win);
end

function [lonOut, latOut] = clampRouteToRangeLocal(lonIn, latIn, lonRange, latRange)
lonOut = min(max(lonIn(:), lonRange(1)), lonRange(2));
latOut = min(max(latIn(:), latRange(1)), latRange(2));
end

function [tVec, nVec] = tangentNormalFromPathLocal(lon, lat, k, lonScale)
n = numel(lon); k1 = max(1, k-4); k2 = min(n, k+4);
v = [(lon(k2)-lon(k1))*lonScale, (lat(k2)-lat(k1))];
if norm(v) < eps, v = [1,0]; end
v = v / norm(v);
nm = [-v(2), v(1)];
tVec = [v(1)/lonScale, v(2)];
nVec = [nm(1)/lonScale, nm(2)];
tVec = tVec / hypot(tVec(1)*lonScale, tVec(2));
nVec = nVec / hypot(nVec(1)*lonScale, nVec(2));
end

function psi = headingFromPathLocal(lon, lat, k)
lon = real(lon(:));
lat = real(lat(:));
if any(~isfinite(lon)) || any(~isfinite(lat))
    lon = sanitizeRealVectorLocal(lon);
    lat = sanitizeRealVectorLocal(lat);
end

n = numel(lon);
k1 = max(1, k-2); k2 = min(n, k+2);
dy = lat(k2)-lat(k1);
dx = lon(k2)-lon(k1);
if ~isfinite(dy) || ~isfinite(dx)
    dy = 0;
    dx = 1;
end
psi = atan2(dy, dx);
end

function [vx, vy] = fusiformVerticesLocal(lon, lat, psi, L, W)
body = [0.50*L, 0.00*W; 0.25*L, 0.22*W; 0.00*L, 0.32*W; -0.28*L, 0.22*W; -0.50*L, 0.00*W; -0.28*L, -0.22*W; 0.00*L, -0.32*W; 0.25*L, -0.22*W];
R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
rotBody = (R * body')';
vx = rotBody(:,1) + lon;
vy = rotBody(:,2) + lat;
end

function [dxLab, dyLab] = getShipLabelOffsetLocal(i, lonW, latH)
switch i
    case 1, dxLab = -0.008 * lonW; dyLab = 0.018 * latH;
    case 2, dxLab = 0.012 * lonW; dyLab = 0.020 * latH;
    case 3, dxLab = 0.015 * lonW; dyLab = -0.020 * latH;
    otherwise, dxLab = 0.012 * lonW; dyLab = 0.015 * latH;
end
end

function drawBinaryMapGeoLocal(ax, G, latRange, lonRange, drawGrid)
[nRows, nCols] = size(G);
imagesc(ax, [lonRange(1), lonRange(2)], [latRange(1), latRange(2)], flipud(double(~G)));
set(ax, 'YDir', 'normal'); colormap(ax, gray(256)); caxis(ax, [0 1]); hold(ax, 'on'); box(ax, 'on');

if drawGrid
    lonLines = linspace(lonRange(1), lonRange(2), nCols);
    latLines = linspace(latRange(1), latRange(2), nRows);
    for i = 1:numel(lonLines)
        plot(ax, [lonLines(i), lonLines(i)], [latRange(1), latRange(2)], '-', 'Color', [0.85 0.85 0.85], 'LineWidth', 0.15);
    end
    for i = 1:numel(latLines)
        plot(ax, [lonRange(1), lonRange(2)], [latLines(i), latLines(i)], '-', 'Color', [0.85 0.85 0.85], 'LineWidth', 0.15);
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
axis(ax, 'image'); axis(ax, 'tight');
end

function [lonNew, latNew] = resamplePolylineByCountLinearLocal(lon, lat, nOut)
lon = sanitizeRealVectorLocal(lon);
lat = sanitizeRealVectorLocal(lat);
valid = isfinite(lon) & isfinite(lat);
lon = lon(valid); lat = lat(valid);
if isempty(lon), error('Input path is empty.'); end
if numel(lon) == 1
    lonNew = repmat(lon, nOut, 1); latNew = repmat(lat, nOut, 1); return;
end

keep = [true; hypot(diff(lon), diff(lat)) > eps];
lon = lon(keep); lat = lat(keep);
if numel(lon) == 1
    lonNew = repmat(lon, nOut, 1); latNew = repmat(lat, nOut, 1); return;
end

d = hypot(diff(lon), diff(lat));
s = [0; cumsum(d)];
[su, ia] = unique(s, 'stable'); lon = lon(ia); lat = lat(ia);
sNew = linspace(0, su(end), nOut)';
lonNew = interp1(su, lon, sNew, 'linear');
latNew = interp1(su, lat, sNew, 'linear');
end

function x = sanitizeRealVectorLocal(x)
x = real(x(:));
if all(~isfinite(x))
    x = zeros(size(x));
    return;
end
if any(~isfinite(x))
    idx = (1:numel(x)).';
    good = isfinite(x);
    x(~good) = interp1(idx(good), x(good), idx(~good), 'linear', 'extrap');
end
end

function [xEnd, yEnd, fracFree] = traceRayFeasibleLocal(x0, y0, ang, maxR, G, latRange, lonRange, shipLon, shipLat, shipBlockR, lonScale)
% DWA 射线：遇障碍或船舶即止，返回可通行比例
nStep = 44;
xs = linspace(x0, x0 + maxR * cos(ang), nStep);
ys = linspace(y0, y0 + maxR * sin(ang), nStep);

lastFree = 1;
for i = 2:nStep
    [r, c] = geoToRCIndexLocal(xs(i), ys(i), size(G), latRange, lonRange);
    if G(r, c)
        break;
    end
    if ~isempty(shipLon)
        dShip = hypot((xs(i) - shipLon) * lonScale, ys(i) - shipLat);
        if any(dShip < shipBlockR)
            break;
        end
    end
    lastFree = i;
end

xEnd = xs(lastFree);
yEnd = ys(lastFree);
fracFree = (lastFree - 1) / (nStep - 1);
end





function [lonOut, latOut] = followNominalOutsideEventsLocal(lonAct, latAct, lonNom, latNom, windows)
% 在无干扰时尽量回归既定航线，避免全局偏移
lonOut = lonAct(:);
latOut = latAct(:);
n = numel(lonOut);
keep = false(n,1);
for i = 1:size(windows,1)
    a = max(1, windows(i,1));
    b = min(n, windows(i,2));
    if b >= a
        keep(a:b) = true;
    end
end

blendIdx = find(~keep);
lonOut(blendIdx) = 0.88 * lonNom(blendIdx) + 0.12 * lonOut(blendIdx);
latOut(blendIdx) = 0.88 * latNom(blendIdx) + 0.12 * latOut(blendIdx);

lonOut = smoothdata(lonOut, 'movmean', 7);
latOut = smoothdata(latOut, 'movmean', 7);
end

function [lonOut, latOut] = dampFishJitterLocal(lonIn, latIn, lonNom, latNom, kFish)
% 渔船避让段去抖，抑制局部小折点
lonOut = lonIn(:); latOut = latIn(:);
n = numel(lonOut);
i1 = max(1, kFish - 10);
i2 = min(n, kFish + 16);
if i2 - i1 < 6, return; end

segLon = lonOut(i1:i2); segLat = latOut(i1:i2);
segLonSm = smoothdata(segLon, 'movmean', 15);
segLatSm = smoothdata(segLat, 'movmean', 15);
refLon = lonNom(i1:i2); refLat = latNom(i1:i2);

m = numel(segLon);
alpha = (sin(pi*(0:m-1)'/max(1,m-1)).^1.25) * 0.60;
tLon = 0.78 * segLonSm + 0.22 * refLon;
tLat = 0.78 * segLatSm + 0.22 * refLat;
lonOut(i1:i2) = (1-alpha).*segLon + alpha.*tLon;
latOut(i1:i2) = (1-alpha).*segLat + alpha.*tLat;
end

function [lonOut, latOut] = dampOvertakeJitterLocal(lonIn, latIn, lonNom, latNom, kOver)
% 对追越3号船附近路段做局部去抖，避免小波纹
lonOut = lonIn(:);
latOut = latIn(:);
n = numel(lonOut);

i1 = max(1, kOver - 20);
i2 = min(n, kOver + 65);
if i2 - i1 < 8
    return;
end

segLon = lonOut(i1:i2);
segLat = latOut(i1:i2);

segLonSm = smoothdata(segLon, 'movmean', 25);
segLatSm = smoothdata(segLat, 'movmean', 25);

segLonRef = lonNom(i1:i2);
segLatRef = latNom(i1:i2);

m = numel(segLon);
phi = (0:m-1)'/max(1,m-1);
alpha = (sin(pi*phi).^1.5) * 0.70;

tLon = 0.80 * segLonSm + 0.20 * segLonRef;
tLat = 0.80 * segLatSm + 0.20 * segLatRef;

lonOut(i1:i2) = (1-alpha).*segLon + alpha.*tLon;
latOut(i1:i2) = (1-alpha).*segLat + alpha.*tLat;
end

function [lonOut, latOut] = stabilizeHarborStartLocal(lonIn, latIn, cfg)
% 出港阶段岸边栅格导致的局部微摆抑制
lonOut = lonIn(:);
latOut = latIn(:);
n = numel(lonOut);

n0 = max(14, round(cfg.startJitterFixFrac * n * 0.95));
n0 = min(n0, n-4);

lonCol = median(lonOut(1:max(5, round(0.20*n0))));
latLine = linspace(latOut(1), latOut(n0), n0)';

w = linspace(1,0,n0)'.^1.1;
lonOut(1:n0) = 0.60*(w*lonCol + (1-w).*lonOut(1:n0)) + 0.40*lonOut(1:n0);
latOut(1:n0) = 0.80*latLine + 0.20*latOut(1:n0);

lonOut(1:n0) = max(min(lonOut(1:n0), lonCol + 4.5e-4), lonCol - 4.5e-4);
lonOut(1:n0+10) = smoothdata(lonOut(1:n0+10), 'movmean', 11);
latOut(1:n0+10) = smoothdata(latOut(1:n0+10), 'movmean', 11);
end

function [lonOut, latOut] = deJitterRouteLocal(lonIn, latIn, nPass, medWin, meanWin)
% 去抖：中值滤波去尖刺 + 均值滤波去微摆 + 局部折返点抑制
lonOut = lonIn(:);
latOut = latIn(:);

for ip = 1:nPass
    lonOut = smoothdata(lonOut, 'movmedian', medWin);
    latOut = smoothdata(latOut, 'movmedian', medWin);

    lonOut = smoothdata(lonOut, 'movmean', meanWin);
    latOut = smoothdata(latOut, 'movmean', meanWin);

    n = numel(lonOut);
    for k = 3:(n-2)
        v1 = [lonOut(k)-lonOut(k-1), latOut(k)-latOut(k-1)];
        v2 = [lonOut(k+1)-lonOut(k), latOut(k+1)-latOut(k)];
        if norm(v1) < 1e-12 || norm(v2) < 1e-12
            continue;
        end
        c = dot(v1,v2)/(norm(v1)*norm(v2));
        if c < -0.10
            lonOut(k) = 0.5*(lonOut(k-1)+lonOut(k+1));
            latOut(k) = 0.5*(latOut(k-1)+latOut(k+1));
        end
    end
end
end

function [lonOut, latOut] = enforceObstacleClearanceLocal(lonIn, latIn, G, latRange, lonRange, minClearCells, searchRadius)
% 将路径点从靠障碍过近位置微调到满足最小安全间距的自由栅格
lonOut = lonIn(:);
latOut = latIn(:);

obsDist = bwdist(G);   % 到障碍物的栅格距离
freeMask = ~G;
[nRows, nCols] = size(G);

for k = 1:numel(lonOut)
    [r0, c0] = geoToRCIndexLocal(lonOut(k), latOut(k), size(G), latRange, lonRange);
    if obsDist(r0, c0) >= minClearCells && freeMask(r0, c0)
        continue;
    end

    rmin = max(1, r0 - searchRadius);
    rmax = min(nRows, r0 + searchRadius);
    cmin = max(1, c0 - searchRadius);
    cmax = min(nCols, c0 + searchRadius);

    subDist = obsDist(rmin:rmax, cmin:cmax);
    subFree = freeMask(rmin:rmax, cmin:cmax);

    [rr, cc] = find(subFree & subDist >= minClearCells);
    if isempty(rr)
        % 若附近无满足阈值点，选择距离障碍最远且离原点最近的自由点
        [rr2, cc2] = find(subFree);
        if isempty(rr2), continue; end
        candR = rr2 + rmin - 1;
        candC = cc2 + cmin - 1;
        candD = obsDist(sub2ind(size(obsDist), candR, candC));
        moveD = hypot(double(candR - r0), double(candC - c0));
        [~, ib] = max(candD - 0.08 * moveD);
        rBest = candR(ib); cBest = candC(ib);
    else
        candR = rr + rmin - 1;
        candC = cc + cmin - 1;
        moveD = hypot(double(candR - r0), double(candC - c0));
        [~, ib] = min(moveD);
        rBest = candR(ib); cBest = candC(ib);
    end

    [latOut(k), lonOut(k)] = rc2geoLocal([rBest, cBest], size(G), latRange, lonRange);
end
end

function metrics = computeRouteMetricsLocal(pathLonNom, pathLatNom, pathLonAct, pathLatAct, cfg)
% 输出用户要求的5项指标
latMean = mean([pathLatNom(:); pathLatAct(:)]);

meterPerDegLat = 111320;
meterPerDegLon = meterPerDegLat * cosd(latMean);

% 1) Path length/(m)
dxm = diff(pathLonAct) * meterPerDegLon;
dym = diff(pathLatAct) * meterPerDegLat;
stepM = hypot(dxm, dym);
pathLengthM = sum(stepM);

% 2) Heading change difference/(°)
hdNom = computeHeadingSeriesLocal(pathLonNom, pathLatNom);
hdAct = computeHeadingSeriesLocal(pathLonAct, pathLatAct);
dNom = sum(abs(diff(rad2deg(unwrap(hdNom)))));
dAct = sum(abs(diff(rad2deg(unwrap(hdAct)))));
headingChangeDiffDeg = dAct - dNom;

% 3) Linear velocity oscillation count/(times)
if numel(stepM) < 3
    velOscCnt = 0;
else
    v = stepM * cfg.fps; % m/s
    dv = diff(v);
    sgn = sign(dv);
    sgn(sgn==0) = 1;
    velOscCnt = sum(abs(diff(sgn)) > 1);
end

% 4) Max angular velocity oscillation amplitude/(deg·s-1)
omega = diff(unwrap(hdAct)) * cfg.fps; % rad/s
omegaDeg = rad2deg(omega);
omegaOsc = omegaDeg - smoothdata(omegaDeg, 'movmean', 9);
if isempty(omegaOsc)
    maxOmegaOsc = 0;
else
    maxOmegaOsc = max(abs(omegaOsc));
end

metrics = struct();
metrics.pathLengthM = pathLengthM;
metrics.headingChangeDiffDeg = headingChangeDiffDeg;
metrics.linearVelocityOscCount = velOscCnt;
metrics.maxAngularVelOscAmpDegS = maxOmegaOsc;
metrics.simulationSteps = numel(pathLonAct);
end

function [r, c] = geoToRCIndexLocal(lon, lat, sz, latRange, lonRange)
nRows = sz(1); nCols = sz(2);
c = round((lon - lonRange(1)) / (lonRange(2) - lonRange(1)) * (nCols - 1) + 1);
r = round((latRange(2) - lat) / (latRange(2) - latRange(1)) * (nRows - 1) + 1);
r = max(1, min(nRows, r));
c = max(1, min(nCols, c));
end

function [lat, lon] = rc2geoLocal(rc, sz, latRange, lonRange)
nRows = sz(1); nCols = sz(2);
r = rc(1); c = rc(2);
lon = lonRange(1) + (c - 0.5) / nCols * (lonRange(2) - lonRange(1));
lat = latRange(2) - (r - 0.5) / nRows * (latRange(2) - latRange(1));
end
