clear; clc; close all;

%% ===================== Global Style =====================
set(groot, 'defaultAxesFontName', 'Times New Roman');
set(groot, 'defaultTextFontName', 'Times New Roman');
set(groot, 'defaultAxesFontSize', 12);
set(groot, 'defaultTextFontSize', 12);

%% ===================== Parameters =====================
cfg.inputResultFile      = 'fuxian_result.mat';

cfg.outputMP4            = 'route_inset_animation_with_obstacles.mp4';
cfg.outputGIF            = 'route_inset_animation_with_obstacles.gif';
cfg.outputObsMat         = 'generated_target_ships.mat';

cfg.saveMP4              = true;
cfg.saveGIF              = false;
cfg.saveObsMat           = true;

cfg.durationSec          = 60;
cfg.fps                  = 12;
cfg.nFrames              = cfg.durationSec * cfg.fps;

cfg.leftAxesPos          = [0.05 0.10 0.53 0.80];
cfg.rightAxesPos         = [0.64 0.20 0.28 0.50];

cfg.nominalColor         = [0.55 0.55 0.55];
cfg.actualColor          = [0 0 1];
cfg.zoomRectColor        = [0.88 0.20 0.20];
cfg.ownShipColor         = [0.85 0.15 0.15];
cfg.dwaCandidateColor    = [0.10 0.65 0.10];
cfg.dwaSelectedColor     = [1.00 0.10 0.10];

cfg.nominalWidth         = 0.9;
cfg.actualWidthMain      = 2.0;
cfg.actualWidthZoom      = 1.8;
cfg.zoomRectWidth        = 1.1;

cfg.startGoalMarkerSize  = 12;
cfg.labelFontSize        = 10;
cfg.expLabelFontSize     = 11;

cfg.showGrid             = false;
cfg.showDWA              = true;

cfg.zoomLonFraction      = 0.12;
cfg.zoomLatFraction      = 0.12;
cfg.zoomAheadFrac        = 0.020;

cfg.safeBufferRadius     = 4;
cfg.snapRadius           = 12;

cfg.ownShipLengthFracLon    = 0.0030;
cfg.ownShipWidthFracLat     = 0.0018;
cfg.targetShipLengthFracLon = 0.0026;
cfg.targetShipWidthFracLat  = 0.0016;
cfg.fishingBoatScale        = 0.75;

cfg.dwaNumCandidates     = 11;
cfg.dwaAngleDeg          = 34;
cfg.dwaLookaheadFrac     = 0.010;
cfg.dwaStepCount         = 14;

cfg.nominalDenseN        = 420;

% Encounter positions along nominal path
cfg.headFrac             = 0.20;   % 对遇
cfg.fishFrac             = 0.58;   % 渔船交叉相遇
cfg.overFrac             = 0.80;   % 追越

% Avoidance amplitudes
cfg.headAvoidAmpFrac     = 0.0035;
cfg.fishAvoidAmpFrac     = 0.0065;
cfg.overAvoidAmpFrac     = 0.0120;

% COLREG-like preferred side:
% right/starboard = -1 * left-normal
cfg.headAvoidSign        = -1;
cfg.fishAvoidSign        = -1;
cfg.overAvoidSign        = -1;

cfg.shipColors = [
    0.90 0.40 0.10;   % Head-on ship
    0.49 0.18 0.56;   % Fishing boat
    0.82 0.72 0.15    % Target ship 3 (overtaken vessel)
];

%% ===================== Load Result =====================
if exist(cfg.inputResultFile, 'file') ~= 2
    error('Cannot find %s.', cfg.inputResultFile);
end

S = load(cfg.inputResultFile);

if ~isfield(S, 'G')
    error('Variable "G" not found in fuxian_result.mat.');
end
if ~isfield(S, 'pathLat') || ~isfield(S, 'pathLon')
    error('Variables "pathLat" and "pathLon" not found in fuxian_result.mat.');
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

%% ===================== Safe Water Masks =====================
freeMask = ~G;
dangerMask = conv2(double(G), ones(2*cfg.safeBufferRadius+1), 'same') > 0;
safeMask = freeMask & ~dangerMask;

%% ===================== Static Nominal Route =====================
% 这里虚线就是静态环境下的原始全局航线，不参与动态避碰修改
[pathLonNomDense, pathLatNomDense] = resamplePolylineByCountLinearLocal( ...
    pathLonRaw, pathLatRaw, cfg.nominalDenseN);

[pathLonNomDense, pathLatNomDense] = snapOnlyInvalidPointsToMaskLocal( ...
    pathLonNomDense, pathLatNomDense, safeMask, freeMask, latRange, lonRange, cfg.snapRadius);

%% ===================== Dynamic Actual Route =====================
[pathLonActDense, pathLatActDense, routeInfo] = buildActualRouteLocal( ...
    pathLonNomDense, pathLatNomDense, safeMask, freeMask, latRange, lonRange, cfg);

%% ===================== Resample for Animation =====================
[pathLonNom, pathLatNom] = resamplePolylineByCountLinearLocal( ...
    pathLonNomDense, pathLatNomDense, cfg.nFrames);

[pathLonAct, pathLatAct] = resamplePolylineByCountLinearLocal( ...
    pathLonActDense, pathLatActDense, cfg.nFrames);

[pathLonAct, pathLatAct] = snapOnlyInvalidPointsToMaskLocal( ...
    pathLonAct, pathLatAct, safeMask, freeMask, latRange, lonRange, cfg.snapRadius);

%% ===================== Create Target Ships =====================
ships = createScenarioShipsLocal( ...
    pathLonNomDense, pathLatNomDense, safeMask, freeMask, latRange, lonRange, cfg);

if cfg.saveObsMat
    save(cfg.outputObsMat, 'ships', ...
        'pathLonNomDense', 'pathLatNomDense', ...
        'pathLonActDense', 'pathLatActDense', 'routeInfo');
end

nShips = numel(ships);

%% ===================== Figure and Axes =====================
fig = figure('Color', 'w', ...
    'Name', 'Inset Route Animation with Obstacles', ...
    'Units', 'pixels', ...
    'Position', [60 60 1500 820]);

axMain = axes('Parent', fig, 'Position', cfg.leftAxesPos);
axZoom = axes('Parent', fig, 'Position', cfg.rightAxesPos);

drawBinaryMapGeoLocal(axMain, G, latRange, lonRange, cfg.showGrid);
drawBinaryMapGeoLocal(axZoom, G, latRange, lonRange, cfg.showGrid);

hold(axMain, 'on');
hold(axZoom, 'on');

%% ===================== Static Graphics =====================
% 虚线：静态全局规划
plot(axMain, pathLonNom, pathLatNom, ':', ...
    'Color', cfg.nominalColor, 'LineWidth', cfg.nominalWidth);

plot(axZoom, pathLonNom, pathLatNom, ':', ...
    'Color', cfg.nominalColor, 'LineWidth', cfg.nominalWidth);

% 起终点
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

initBox = [pathLonAct(1)-0.5*boxW, pathLatAct(1)-0.5*boxH, boxW, boxH];

hRectMain = rectangle(axMain, 'Position', initBox, ...
    'EdgeColor', cfg.zoomRectColor, ...
    'LineWidth', cfg.zoomRectWidth, ...
    'LineStyle', '--');

xlim(axZoom, [initBox(1), initBox(1)+initBox(3)]);
ylim(axZoom, [initBox(2), initBox(2)+initBox(4)]);

%% ===================== Actual Route History =====================
% 实际轨迹：只会随着本船航行逐步留下
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
    'VerticalAlignment', 'bottom');

hOwnTextZoom = text(axZoom, pathLonAct(1), pathLatAct(1), 'Experimental ship', ...
    'FontName', 'Times New Roman', ...
    'FontSize', cfg.expLabelFontSize, ...
    'Color', 'k', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'bottom');

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

%% ===================== Ship Name Labels =====================
hShipTextMain = gobjects(nShips,1);
for i = 1:nShips
    [dxLab, dyLab] = getShipLabelOffsetLocal(i, lonW, latH);
    hShipTextMain(i) = text(axMain, ships(i).lon(1)+dxLab, ships(i).lat(1)+dyLab, ships(i).name, ...
        'FontName', 'Times New Roman', ...
        'FontSize', cfg.labelFontSize, ...
        'Color', 'k', ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle');
end

hShipTextZoom = text(axZoom, ships(1).lon(1), ships(1).lat(1), ships(1).name, ...
    'FontName', 'Times New Roman', ...
    'FontSize', cfg.labelFontSize, ...
    'Color', 'k', ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle');

%% ===================== DWA Candidate Trajectories =====================
if cfg.showDWA
    hDWACand = gobjects(cfg.dwaNumCandidates,1);
    for i = 1:cfg.dwaNumCandidates
        hDWACand(i) = plot(axMain, nan, nan, '-', ...
            'Color', cfg.dwaCandidateColor, 'LineWidth', 0.7);
    end

    hDWASelected = plot(axMain, nan, nan, '-', ...
        'Color', cfg.dwaSelectedColor, 'LineWidth', 1.4);
else
    hDWACand = gobjects(0);
    hDWASelected = gobjects(1);
end

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
    % 实际轨迹逐步显示
    set(hTrailMain, 'XData', pathLonAct(1:k), 'YData', pathLatAct(1:k));
    set(hTrailZoom, 'XData', pathLonAct(1:k), 'YData', pathLatAct(1:k));

    % 本船形状
    psiOwn = headingFromPathLocal(pathLonAct, pathLatAct, k);
    [vxOwn, vyOwn] = fusiformVerticesLocal(pathLonAct(k), pathLatAct(k), psiOwn, Lown, Wown);
    set(hOwnShipMain, 'XData', vxOwn, 'YData', vyOwn);
    set(hOwnShipZoom, 'XData', vxOwn, 'YData', vyOwn);

    % 跟随放大框
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

    % 本船标签
    set(hOwnTextMain, 'Position', [pathLonAct(k) - 0.025*lonW, pathLatAct(k) + 0.028*latH, 0]);
    set(hOwnTextZoom, 'Position', [x0 + 0.36*boxW, y0 + 0.68*boxH, 0]);

    % 目标船
    nearestShipIdx = 1;
    nearestDist = inf;

    for i = 1:nShips
        [vx, vy] = fusiformVerticesLocal(ships(i).lon(k), ships(i).lat(k), ships(i).heading(k), ...
            Ltar * ships(i).sizeScale, Wtar * ships(i).sizeScale);

        set(hShipMain(i), 'XData', vx, 'YData', vy);
        set(hShipZoom(i), 'XData', vx, 'YData', vy);

        d = hypot((ships(i).lon(k) - pathLonAct(k))*lonScale, (ships(i).lat(k) - pathLatAct(k)));
        if d < nearestDist
            nearestDist = d;
            nearestShipIdx = i;
        end

        [dxLab, dyLab] = getShipLabelOffsetLocal(i, lonW, latH);
        set(hShipTextMain(i), 'Position', [ships(i).lon(k)+dxLab, ships(i).lat(k)+dyLab, 0]);
    end

    [dxLabZ, dyLabZ] = getShipLabelOffsetLocal(nearestShipIdx, boxW, boxH);
    set(hShipTextZoom, ...
        'String', ships(nearestShipIdx).name, ...
        'Position', [ships(nearestShipIdx).lon(k)+0.12*dxLabZ, ships(nearestShipIdx).lat(k)+0.12*dyLabZ, 0]);

    % DWA 检索示意（始终显示）
    if cfg.showDWA
        [candX, candY, selX, selY] = generateDWACandidatesSafeLocal( ...
            pathLonAct, pathLatAct, k, safeMask, latRange, lonRange, cfg);

        for ii = 1:cfg.dwaNumCandidates
            set(hDWACand(ii), 'XData', candX{ii}, 'YData', candY{ii});
        end
        set(hDWASelected, 'XData', selX, 'YData', selY);
    end

    drawnow;

    frame = getframe(fig);

    if cfg.saveMP4
        writeVideo(v, frame);
    end

    if cfg.saveGIF
        [im, cm] = rgb2ind(frame2im(frame), 256);
        if ~gifInitialized
            imwrite(im, cm, cfg.outputGIF, 'gif', 'Loopcount', inf, 'DelayTime', 1/cfg.fps);
            gifInitialized = true;
        else
            imwrite(im, cm, cfg.outputGIF, 'gif', 'WriteMode', 'append', 'DelayTime', 1/cfg.fps);
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

function [pathLonAct, pathLatAct, info] = buildActualRouteLocal(pathLonNom, pathLatNom, safeMask, freeMask, latRange, lonRange, cfg)
% 实际轨迹 = 原规划路径 + 三段局部避让段
% 每一段避让完成后都会回接回原规划路径

n = numel(pathLonNom);
mainScale = max(diff(lonRange), diff(latRange));

kHead = round(cfg.headFrac * n);
kFish = round(cfg.fishFrac * n);
kOver = round(cfg.overFrac * n);

kHeadPre  = max(2, kHead - 16);
kHeadPost = min(n, kHead + 18);

kFishPre  = max(2, kFish - 18);
kFishPost = min(n, kFish + 20);

kOverPre  = max(2, kOver - 20);
kOverPost = min(n, kOver + 30);

% 对遇：右转避让（starboard）
headWp = chooseOffsetWaypointLocal(pathLonNom, pathLatNom, kHead, ...
    cfg.headAvoidSign, cfg.headAvoidAmpFrac*mainScale, 0.004*mainScale, ...
    safeMask, freeMask, latRange, lonRange, cfg.snapRadius);

% 交叉相遇：更明显右转避让
fishWp = chooseOffsetWaypointLocal(pathLonNom, pathLatNom, kFish, ...
    cfg.fishAvoidSign, cfg.fishAvoidAmpFrac*mainScale, 0.004*mainScale, ...
    safeMask, freeMask, latRange, lonRange, cfg.snapRadius);

% 追越：幅度更大，并设置两个控制点后回到原航线
overWp1 = chooseOffsetWaypointLocal(pathLonNom, pathLatNom, kOver-3, ...
    cfg.overAvoidSign, cfg.overAvoidAmpFrac*mainScale, 0.006*mainScale, ...
    safeMask, freeMask, latRange, lonRange, cfg.snapRadius);

overWp2 = chooseOffsetWaypointLocal(pathLonNom, pathLatNom, kOver+8, ...
    cfg.overAvoidSign, 0.75*cfg.overAvoidAmpFrac*mainScale, 0.004*mainScale, ...
    safeMask, freeMask, latRange, lonRange, cfg.snapRadius);

% 分段拼接，保证避让后回到原虚线
seg1_nom = [pathLonNom(1:kHeadPre), pathLatNom(1:kHeadPre)];
seg1_dyn = buildSegmentViaWaypointsLocal( ...
    [pathLonNom(kHeadPre),  pathLatNom(kHeadPre); ...
     headWp(1),            headWp(2); ...
     pathLonNom(kHeadPost), pathLatNom(kHeadPost)], ...
     safeMask, latRange, lonRange);

seg2_nom = [pathLonNom(kHeadPost:kFishPre), pathLatNom(kHeadPost:kFishPre)];
seg2_dyn = buildSegmentViaWaypointsLocal( ...
    [pathLonNom(kFishPre),  pathLatNom(kFishPre); ...
     fishWp(1),            fishWp(2); ...
     pathLonNom(kFishPost), pathLatNom(kFishPost)], ...
     safeMask, latRange, lonRange);

seg3_nom = [pathLonNom(kFishPost:kOverPre), pathLatNom(kFishPost:kOverPre)];
seg3_dyn = buildSegmentViaWaypointsLocal( ...
    [pathLonNom(kOverPre),  pathLatNom(kOverPre); ...
     overWp1(1),           overWp1(2); ...
     overWp2(1),           overWp2(2); ...
     pathLonNom(kOverPost), pathLatNom(kOverPost)], ...
     safeMask, latRange, lonRange);

seg4_nom = [pathLonNom(kOverPost:end), pathLatNom(kOverPost:end)];

P = [
    seg1_nom;
    seg1_dyn(2:end,:);
    seg2_nom(2:end,:);
    seg2_dyn(2:end,:);
    seg3_nom(2:end,:);
    seg3_dyn(2:end,:);
    seg4_nom(2:end,:)
];

P = uniqueConsecutiveRowsLocal(P);
[pathLonAct, pathLatAct] = deal(P(:,1), P(:,2));

info.kHead = kHead;
info.kFish = kFish;
info.kOver = kOver;
info.headWp = headWp;
info.fishWp = fishWp;
info.overWp1 = overWp1;
info.overWp2 = overWp2;
end

function segGeo = buildSegmentViaWaypointsLocal(ctrlGeo, safeMask, latRange, lonRange)
% ctrlGeo: [lon lat]
% 用局部A*把多个控制点连起来

segRC = [];
for i = 1:size(ctrlGeo,1)-1
    pA = ctrlGeo(i,:);
    pB = ctrlGeo(i+1,:);
    rc = astarBetweenGeoLocal(pA, pB, safeMask, latRange, lonRange, 65);

    if isempty(rc)
        % fallback: straight sampled then snap
        lonTmp = linspace(pA(1), pB(1), 80)';
        latTmp = linspace(pA(2), pB(2), 80)';
        [lonTmp, latTmp] = snapTrajectoryToMaskLocal(lonTmp, latTmp, safeMask, safeMask, latRange, lonRange, 8);
        [rTmp, cTmp] = geoPathToRCPathLocal(lonTmp, latTmp, size(safeMask), latRange, lonRange);
        rc = uniqueConsecutiveRowsLocal([rTmp, cTmp]);
    end

    rc = smoothPathGreedyLOSLocal(rc, safeMask);

    if i > 1 && ~isempty(rc)
        rc(1,:) = [];
    end
    segRC = [segRC; rc]; %#ok<AGROW>
end

[latSeg, lonSeg] = rcPathToGeoLocal(segRC, size(safeMask), latRange, lonRange);
segGeo = [lonSeg, latSeg];
segGeo = uniqueConsecutiveRowsLocal(segGeo);
end

function wp = chooseOffsetWaypointLocal(pathLon, pathLat, k, preferredSign, amp, tangShift, safeMask, freeMask, latRange, lonRange, snapRadius)
% preferredSign = -1 表示右舷侧（starboard），+1 表示左舷侧
latMean = mean(latRange);
lonScale = cosd(latMean);
[tVec, nVec] = tangentNormalFromPathLocal(pathLon, pathLat, k, lonScale);

base = [pathLon(k), pathLat(k)];

% 注意：nVec 是左法向，右舷 = -nVec
cand = zeros(6,2);
cand(1,:) = base + tangShift * tVec + preferredSign * amp * nVec;
cand(2,:) = base + tangShift * tVec + preferredSign * 0.75*amp * nVec;
cand(3,:) = base + 0.5*tangShift * tVec + preferredSign * 0.60*amp * nVec;
cand(4,:) = base + tangShift * tVec - preferredSign * 0.60*amp * nVec;
cand(5,:) = base;
cand(6,:) = base + 0.3*tangShift * tVec;

for i = 1:size(cand,1)
    p = cand(i,:);
    [r0, c0] = geoToRCIndexLocal(p(1), p(2), size(safeMask), latRange, lonRange);
    [r1, c1, ok] = snapRCToAllowedLocal(safeMask, r0, c0, snapRadius);
    if ~ok
        [r1, c1, ok] = snapRCToAllowedLocal(freeMask, r0, c0, snapRadius);
    end
    if ok
        [lat1, lon1] = rc2geoLocal([r1, c1], size(safeMask), latRange, lonRange);
        wp = [lon1, lat1];
        return;
    end
end

wp = base;
end

function ships = createScenarioShipsLocal(pathLonNom, pathLatNom, safeMask, freeMask, latRange, lonRange, cfg)
% 简化成 3 艘目标船：
% 1) Head-on ship：对遇
% 2) Fishing boat：交叉相遇
% 3) Target ship 3：同向慢船，供本船追越避让

n = numel(pathLonNom);
mainScale = max(diff(lonRange), diff(latRange));

kHead = round(cfg.headFrac * n);
kFish = round(cfg.fishFrac * n);
kOver = round(cfg.overFrac * n);

ships = repmat(struct( ...
    'name', '', ...
    'lon', [], ...
    'lat', [], ...
    'heading', [], ...
    'color', zeros(1,3), ...
    'sizeScale', 1), 3, 1);

% 1) 对遇船：直接沿既定规划航线反向驶来
ships(1) = createShipOnReferenceLocal( ...
    'Head-on ship', pathLonNom, pathLatNom, ...
    min(n, kHead+20), max(1, kHead-16), ...
    round(0.08*cfg.nFrames), round(0.30*cfg.nFrames), ...
    0.0, safeMask, freeMask, latRange, lonRange, ...
    cfg.shipColors(1,:), 0.95, cfg.nFrames);

% 2) 渔船：横穿航道，形成交叉相遇
ships(2) = createFishingBoatLocal( ...
    'Fishing boat', pathLonNom, pathLatNom, kFish, ...
    freeMask, latRange, lonRange, cfg.shipColors(2,:), ...
    cfg.fishingBoatScale, cfg.nFrames);

% 3) 追越目标船：与本船同向、速度更慢
ships(3) = createShipOnReferenceOffsetLocal( ...
    'Target ship 3', pathLonNom, pathLatNom, ...
    max(1, kOver-15), min(n, kOver+38), ...
    round(0.60*cfg.nFrames), round(0.98*cfg.nFrames), ...
    0.0, safeMask, freeMask, latRange, lonRange, ...
    cfg.shipColors(3,:), 0.90, cfg.nFrames);

% 让追越目标船更慢：让它前半程停留更久
ships(3).lon(1:round(0.66*cfg.nFrames)) = ships(3).lon(round(0.60*cfg.nFrames));
ships(3).lat(1:round(0.66*cfg.nFrames)) = ships(3).lat(round(0.60*cfg.nFrames));
ships(3).heading = computeHeadingSeriesLocal(ships(3).lon, ships(3).lat);
end

function ship = createShipOnReferenceLocal(name, refLon, refLat, idxStart, idxEnd, ...
    frameStart, frameEnd, offsetAmp, safeMask, freeMask, latRange, lonRange, ...
    color, sizeScale, nFrames)

[lon, lat, hd] = buildReferenceTrajectoryLocal( ...
    refLon, refLat, idxStart, idxEnd, frameStart, frameEnd, ...
    offsetAmp, safeMask, freeMask, latRange, lonRange, nFrames);

ship = struct( ...
    'name', name, ...
    'lon', lon, ...
    'lat', lat, ...
    'heading', hd, ...
    'color', color, ...
    'sizeScale', sizeScale);
end

function ship = createShipOnReferenceOffsetLocal(name, refLon, refLat, idxStart, idxEnd, ...
    frameStart, frameEnd, offsetAmp, safeMask, freeMask, latRange, lonRange, ...
    color, sizeScale, nFrames)

[lon, lat, hd] = buildReferenceTrajectoryLocal( ...
    refLon, refLat, idxStart, idxEnd, frameStart, frameEnd, ...
    offsetAmp, safeMask, freeMask, latRange, lonRange, nFrames);

ship = struct( ...
    'name', name, ...
    'lon', lon, ...
    'lat', lat, ...
    'heading', hd, ...
    'color', color, ...
    'sizeScale', sizeScale);
end

function [lon, lat, hd] = buildReferenceTrajectoryLocal(refLon, refLat, idxStart, idxEnd, ...
    frameStart, frameEnd, offsetAmp, safeMask, freeMask, latRange, lonRange, nFrames)

idxStart = max(1, min(numel(refLon), idxStart));
idxEnd   = max(1, min(numel(refLon), idxEnd));

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

    lonOff = zeros(nSeg,1);
    latOff = zeros(nSeg,1);

    for i = 1:nSeg
        [~, nVec] = tangentNormalFromPathLocal(lonSeg, latSeg, i, lonScale);
        lonOff(i) = lonSeg(i) + offsetAmp * nVec(1);
        latOff(i) = latSeg(i) + offsetAmp * nVec(2);
    end

    lonSeg = lonOff;
    latSeg = latOff;
end

[lonSeg, latSeg] = snapOnlyInvalidPointsToMaskLocal( ...
    lonSeg, latSeg, safeMask, freeMask, latRange, lonRange, 8);

trackLen = max(2, frameEnd - frameStart + 1);
[lonTrack, latTrack] = resamplePolylineByCountLinearLocal(lonSeg, latSeg, trackLen);

lon = nan(nFrames,1);
lat = nan(nFrames,1);

lon(1:frameStart) = lonTrack(1);
lat(1:frameStart) = latTrack(1);

lon(frameStart:frameEnd) = lonTrack;
lat(frameStart:frameEnd) = latTrack;

lon(frameEnd:end) = lonTrack(end);
lat(frameEnd:end) = latTrack(end);

hd = computeHeadingSeriesLocal(lon, lat);
end

function ship = createFishingBoatLocal(name, refLon, refLat, kCross, freeMask, latRange, lonRange, color, sizeScale, nFrames)
latMean = mean(latRange);
lonScale = cosd(latMean);
mainScale = max(diff(lonRange), diff(latRange));

[tVec, nVec] = tangentNormalFromPathLocal(refLon, refLat, kCross, lonScale);
base = [refLon(kCross), refLat(kCross)];

% 从本船右舷侧横穿到左舷侧，制造本船应避让的交叉相遇
pStart = base - 0.020*mainScale*nVec + 0.004*mainScale*tVec;
pEnd   = base + 0.020*mainScale*nVec - 0.006*mainScale*tVec;

pathRC = astarBetweenGeoLocal(pStart, pEnd, freeMask, latRange, lonRange, 60);
if isempty(pathRC)
    lonTemp = linspace(pStart(1), pEnd(1), 80)';
    latTemp = linspace(pStart(2), pEnd(2), 80)';
    [lonTemp, latTemp] = snapTrajectoryToMaskLocal(lonTemp, latTemp, freeMask, freeMask, latRange, lonRange, 8);
else
    [latTemp, lonTemp] = rcPathToGeoLocal(pathRC, size(freeMask), latRange, lonRange);
end

frameStart = round(0.46 * nFrames);
frameEnd   = round(0.72 * nFrames);

[lonTrack, latTrack] = resamplePolylineByCountLinearLocal(lonTemp, latTemp, max(2, frameEnd-frameStart+1));

lon = nan(nFrames,1);
lat = nan(nFrames,1);

lon(1:frameStart) = lonTrack(1);
lat(1:frameStart) = latTrack(1);

lon(frameStart:frameEnd) = lonTrack;
lat(frameStart:frameEnd) = latTrack;

lon(frameEnd:end) = lonTrack(end);
lat(frameEnd:end) = latTrack(end);

ship = struct( ...
    'name', name, ...
    'lon', lon, ...
    'lat', lat, ...
    'heading', computeHeadingSeriesLocal(lon, lat), ...
    'color', color, ...
    'sizeScale', sizeScale);
end

function hd = computeHeadingSeriesLocal(lon, lat)
n = numel(lon);
hd = zeros(n,1);
for k = 1:n
    hd(k) = headingFromPathLocal(lon, lat, k);
end
hd = smoothHeadingLocal(hd);
end

function [candX, candY, selX, selY] = generateDWACandidatesSafeLocal(pathLon, pathLat, k, safeMask, latRange, lonRange, cfg)
nCand = cfg.dwaNumCandidates;
candX = cell(nCand,1);
candY = cell(nCand,1);

psi = headingFromPathLocal(pathLon, pathLat, k);
angles = linspace(-cfg.dwaAngleDeg, cfg.dwaAngleDeg, nCand);
anglesRad = deg2rad(angles);

stepLen = cfg.dwaLookaheadFrac * diff(lonRange);
nSteps = cfg.dwaStepCount;

for i = 1:nCand
    a = anglesRad(i);
    x = nan(nSteps,1);
    y = nan(nSteps,1);

    x(1) = pathLon(k);
    y(1) = pathLat(k);

    for s = 2:nSteps
        alpha = (s-1) / (nSteps-1);
        psi_s = psi + alpha * a;

        xNew = x(s-1) + stepLen * cos(psi_s);
        yNew = y(s-1) + stepLen * sin(psi_s);

        if pointInMaskLocal(xNew, yNew, safeMask, latRange, lonRange)
            x(s) = xNew;
            y(s) = yNew;
        else
            x = x(1:s-1);
            y = y(1:s-1);
            break;
        end
    end

    candX{i} = x;
    candY{i} = y;
end

k2 = min(numel(pathLon), k + nSteps - 1);
selX = pathLon(k:k2);
selY = pathLat(k:k2);
end

function ok = pointInMaskLocal(lon, lat, mask, latRange, lonRange)
[r, c] = geoToRCIndexLocal(lon, lat, size(mask), latRange, lonRange);
ok = mask(r,c);
end

function [pathRC] = astarBetweenGeoLocal(pStart, pGoal, freeMask, latRange, lonRange, padding)
[r1, c1] = geoToRCIndexLocal(pStart(1), pStart(2), size(freeMask), latRange, lonRange);
[r2, c2] = geoToRCIndexLocal(pGoal(1),  pGoal(2),  size(freeMask), latRange, lonRange);

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
sLocal = [r1-rmin+1, c1-cmin+1];
gLocal = [r2-rmin+1, c2-cmin+1];

pathLocal = astarGridLocal(localMask, sLocal, gLocal);
if isempty(pathLocal)
    pathRC = [];
    return;
end

pathRC = pathLocal;
pathRC(:,1) = pathRC(:,1) + rmin - 1;
pathRC(:,2) = pathRC(:,2) + cmin - 1;
end

function pathRC = astarGridLocal(freeMask, startRC, goalRC)
[nRows, nCols] = size(freeMask);
N = nRows * nCols;

sIdx = sub2ind([nRows, nCols], startRC(1), startRC(2));
gIdx = sub2ind([nRows, nCols], goalRC(1), goalRC(2));

gScore = inf(N,1);
parent = zeros(N,1,'uint32');
state = zeros(N,1,'uint8');

heapNode = zeros(N,1,'uint32');
heapF    = inf(N,1);
heapPos  = zeros(N,1,'uint32');
heapSize = uint32(0);

gScore(sIdx) = 0;
h0 = octileHeuristicLocal(startRC(1), startRC(2), goalRC(1), goalRC(2));
[heapNode, heapF, heapPos, heapSize] = heapPushLocal(heapNode, heapF, heapPos, heapSize, uint32(sIdx), h0);
state(sIdx) = 1;

dr = int32([-1  0  1  1  1  0 -1 -1]);
dc = int32([-1 -1 -1  0  1  1  1  0]);
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

        if mod(k,2) == 1
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
if size(pathRC,1) <= 2
    pathRC2 = pathRC;
    return;
end

pathRC2 = pathRC(1,:);
i = 1;
n = size(pathRC,1);

while i < n
    linked = false;
    for j = n:-1:(i+1)
        if hasSafeLineOfSightLocal(pathRC(i,:), pathRC(j,:), freeMask)
            pathRC2 = [pathRC2; pathRC(j,:)]; %#ok<AGROW>
            i = j;
            linked = true;
            break;
        end
    end
    if ~linked
        pathRC2 = [pathRC2; pathRC(i+1,:)]; %#ok<AGROW>
        i = i + 1;
    end
end
end

function tf = hasSafeLineOfSightLocal(rc1, rc2, freeMask)
[nRows, nCols] = size(freeMask);

r1 = rc1(1); c1 = rc1(2);
r2 = rc2(1); c2 = rc2(2);

nStep = max(abs(r2-r1), abs(c2-c1)) * 10 + 1;
rs = linspace(r1, r2, nStep);
cs = linspace(c1, c2, nStep);

tf = true;
for k = 1:nStep
    rr = rs(k);
    cc = cs(k);

    rFloor = floor(rr);
    rCeil  = ceil(rr);
    cFloor = floor(cc);
    cCeil  = ceil(cc);

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

function [rPath, cPath] = geoPathToRCPathLocal(lon, lat, sz, latRange, lonRange)
n = numel(lon);
rPath = zeros(n,1);
cPath = zeros(n,1);

for i = 1:n
    [rPath(i), cPath(i)] = geoToRCIndexLocal(lon(i), lat(i), sz, latRange, lonRange);
end
end

function M = uniqueConsecutiveRowsLocal(M)
if isempty(M)
    return;
end
keep = [true; any(diff(M,1,1)~=0,2)];
M = M(keep,:);
end

function valid = pointsInMaskLocal(lon, lat, mask, latRange, lonRange)
valid = false(numel(lon),1);
sz = size(mask);

for k = 1:numel(lon)
    [r, c] = geoToRCIndexLocal(lon(k), lat(k), sz, latRange, lonRange);
    valid(k) = mask(r,c);
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
    rmin = max(1, r0-rad);
    rmax = min(nRows, r0+rad);
    cmin = max(1, c0-rad);
    cmax = min(nCols, c0+rad);

    found = false;
    for r = rmin:rmax
        for c = cmin:cmax
            if mask(r,c)
                d = hypot(r-r0, c-c0);
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
n = size(pathRC,1);
latPath = zeros(n,1);
lonPath = zeros(n,1);

for i = 1:n
    [latPath(i), lonPath(i)] = rc2geoLocal(pathRC(i,:), sz, latRange, lonRange);
end
end

function [tVec, nVec] = tangentNormalFromPathLocal(lon, lat, k, lonScale)
n = numel(lon);
k1 = max(1, k-6);
k2 = min(n, k+6);

vMetric = [(lon(k2)-lon(k1))*lonScale, (lat(k2)-lat(k1))];
nv = norm(vMetric);
if nv < eps
    vMetric = [1, 0];
    nv = 1;
end

tMetric = vMetric / nv;
nMetric = [-tMetric(2), tMetric(1)];

tVec = [tMetric(1)/lonScale, tMetric(2)];
nVec = [nMetric(1)/lonScale, nMetric(2)];

tNorm = hypot(tVec(1)*lonScale, tVec(2));
nNorm = hypot(nVec(1)*lonScale, nVec(2));

tVec = tVec / tNorm;
nVec = nVec / nNorm;
end

function psi = headingFromPathLocal(lon, lat, k)
n = numel(lon);
k1 = max(1, k-2);
k2 = min(n, k+2);
psi = atan2(lat(k2) - lat(k1), lon(k2) - lon(k1));
end

function hd = smoothHeadingLocal(hd)
hd = unwrap(hd);
hd = smoothdata(hd, 'movmean', 7);
end

function [vx, vy] = fusiformVerticesLocal(lon, lat, psi, L, W)
body = [ ...
     0.50*L,  0.00*W;
     0.25*L,  0.22*W;
     0.00*L,  0.32*W;
    -0.28*L,  0.22*W;
    -0.50*L,  0.00*W;
    -0.28*L, -0.22*W;
     0.00*L, -0.32*W;
     0.25*L, -0.22*W];

R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
rotBody = (R * body')';

vx = rotBody(:,1) + lon;
vy = rotBody(:,2) + lat;
end

function [dxLab, dyLab] = getShipLabelOffsetLocal(i, lonW, latH)
switch i
    case 1
        dxLab = -0.008 * lonW;  dyLab =  0.018 * latH;   % Head-on ship
    case 2
        dxLab =  0.012 * lonW;  dyLab =  0.020 * latH;   % Fishing boat
    case 3
        dxLab =  0.015 * lonW;  dyLab = -0.020 * latH;   % Target ship 3
    otherwise
        dxLab = 0.012 * lonW;   dyLab = 0.015 * latH;
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
        plot(ax, [lonLines(i) lonLines(i)], [latRange(1) latRange(2)], '-', ...
            'Color', [0.85 0.85 0.85], 'LineWidth', 0.15);
    end
    for i = 1:numel(latLines)
        plot(ax, [lonRange(1) lonRange(2)], [latLines(i) latLines(i)], '-', ...
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
ylabel(ax, 'Latitude/(°)',  'FontName', 'Times New Roman');

axis(ax, 'image');
axis(ax, 'tight');
end

function [lonNew, latNew] = resamplePolylineByCountLinearLocal(lon, lat, nOut)
% 稳健重采样：
% 1) 去掉 NaN/Inf
% 2) 去掉连续重复点
% 3) 去掉重复弧长采样点
% 4) 单点退化时也能正常返回

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

keep = [true; abs(diff(lon)) > eps(max(abs(lon))+1) | abs(diff(lat)) > eps(max(abs(lat))+1)];
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