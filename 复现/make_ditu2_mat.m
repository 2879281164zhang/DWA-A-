function make_ditu2_mat(sourceInput)
% 生成 ditu2.mat
%
% 功能：
% 1) 从黑白栅格图像或工作区变量 ditu2 生成标准占据栅格地图
% 2) 输出变量：
%       ditu2 = 1 表示障碍
%       ditu2 = 0 表示自由
% 3) 保存坐标信息
%
% 坐标信息：
%   北纬范围：35.9690 ~ 36.0614
%   东经范围：120.2090 ~ 120.4070
%
% 用法：
%   make_ditu2_mat
%   make_ditu2_mat('ditu2.png')
%   make_ditu2_mat(yourMatrix)

clc;
close all;

% ===== 关键修复：没有输入参数时，先给默认值 =====
if nargin < 1
    sourceInput = [];
end

%% ===================== 固定参数 =====================
latMin = 35.9690;
latMax = 36.0614;

lonMin = 120.2090;
lonMax = 120.4070;

% 输入解释方式：
% 'imageBW'   : 输入是黑白图，黑=障碍，白=自由
% 'occupancy' : 输入已经是占据栅格，1=障碍，0=自由
inputMode = 'imageBW';

% 仅当 inputMode='imageBW' 时有效
bwThreshold = 0.5;
invertBW = false;   % 若黑白反了，就改成 true

%% ===================== 读取输入 =====================
[sourceData, sourceName] = readSourceData(sourceInput);

%% ===================== 生成标准 ditu2 =====================
switch lower(inputMode)
    case 'occupancy'
        if ~ismatrix(sourceData) || ~isnumeric(sourceData)
            error('occupancy 模式下，输入必须是二维数值矩阵。');
        end
        ditu2 = uint8(sourceData ~= 0);

    case 'imagebw'
        grayMap = toGray01(sourceData);
        BWfree = grayMap >= bwThreshold;   % 白=自由

        if invertBW
            BWfree = ~BWfree;
        end

        ditu2 = uint8(~BWfree);            % 1=障碍, 0=自由

    otherwise
        error('inputMode 只能是 ''imageBW'' 或 ''occupancy''。');
end

%% ===================== 基本检查 =====================
[nRows, nCols] = size(ditu2);

if nRows < 2 || nCols < 2
    error('地图尺寸过小，无法生成有效的 ditu2.mat。');
end

if all(ditu2(:) == 0)
    warning('当前生成结果中没有障碍物，ditu2 全为 0。请检查 bwThreshold 或 invertBW 设置。');
end

if all(ditu2(:) == 1)
    warning('当前生成结果中全为障碍物，ditu2 全为 1。请检查 bwThreshold 或 invertBW 设置。');
end

%% ===================== 保存坐标信息 =====================
% 按你当前表述：
% X轴 = 北纬
% Y轴 = 东经
x_axis_lat_user = linspace(latMin, latMax, nCols);
y_axis_lon_user = linspace(lonMin, lonMax, nRows);

% 按常见地图栅格约定：
% 列 -> 经度增加
% 行 -> 纬度通常从北到南
lon_axis_col_standard = linspace(lonMin, lonMax, nCols);
lat_axis_row_standard = linspace(latMax, latMin, nRows);

mapInfo = struct();
mapInfo.mapName = 'ditu2';
mapInfo.sourceName = sourceName;
mapInfo.rows = nRows;
mapInfo.cols = nCols;
mapInfo.obstacleValue = 1;
mapInfo.freeValue = 0;

mapInfo.latRange = [latMin, latMax];
mapInfo.lonRange = [lonMin, lonMax];

mapInfo.userAxisConvention = 'X=latitude, Y=longitude';
mapInfo.x_axis_lat_user = x_axis_lat_user;
mapInfo.y_axis_lon_user = y_axis_lon_user;

mapInfo.standardAxisConvention = 'column=longitude, row=latitude';
mapInfo.lon_axis_col_standard = lon_axis_col_standard;
mapInfo.lat_axis_row_standard = lat_axis_row_standard;

mapInfo.inputMode = inputMode;
mapInfo.bwThreshold = bwThreshold;
mapInfo.invertBW = invertBW;

%% ===================== 保存 MAT 文件 =====================
save('ditu2.mat', ...
    'ditu2', ...
    'mapInfo', ...
    'x_axis_lat_user', 'y_axis_lon_user', ...
    'lon_axis_col_standard', 'lat_axis_row_standard');

%% ===================== 预览 =====================
previewMap = 1 - double(ditu2);   % 障碍黑，自由白

figure('Color', 'w', 'Name', 'ditu2.mat Preview');

subplot(1,3,1);
showInputPreview(sourceData);
title('输入数据预览');

subplot(1,3,2);
imagesc(previewMap);
axis image off;
colormap(gca, gray(256));
caxis([0 1]);
title('生成的 ditu2');

subplot(1,3,3);
imagesc(previewMap);
axis image off;
colormap(gca, gray(256));
caxis([0 1]);
title(sprintf('rows=%d, cols=%d', nRows, nCols));

fprintf('\n======== ditu2.mat 已生成 ========\n');
fprintf('文件名: ditu2.mat\n');
fprintf('地图尺寸: %d x %d\n', nRows, nCols);
fprintf('输入来源: %s\n', sourceName);
fprintf('障碍值: 1\n');
fprintf('自由值: 0\n');
fprintf('北纬范围: %.4f ~ %.4f\n', latMin, latMax);
fprintf('东经范围: %.4f ~ %.4f\n', lonMin, lonMax);

fprintf('\n已保存变量:\n');
fprintf('  - ditu2\n');
fprintf('  - mapInfo\n');
fprintf('  - x_axis_lat_user\n');
fprintf('  - y_axis_lon_user\n');
fprintf('  - lon_axis_col_standard\n');
fprintf('  - lat_axis_row_standard\n');

end

%% ===================== 本地函数 =====================

function [data, sourceName] = readSourceData(sourceInput)
% 读取来源：
% 1) 直接传入矩阵
% 2) 工作区变量 ditu2
% 3) 选择图片文件

if nargin >= 1 && ~isempty(sourceInput)
    if isnumeric(sourceInput) || islogical(sourceInput)
        data = sourceInput;
        sourceName = 'function input matrix';
        return;
    elseif ischar(sourceInput) || isstring(sourceInput)
        sourceInput = char(sourceInput);
        if exist(sourceInput, 'file') ~= 2
            error('找不到输入文件：%s', sourceInput);
        end
        data = imread(sourceInput);
        sourceName = sourceInput;
        return;
    else
        error('sourceInput 必须是矩阵或图像路径。');
    end
end

% 优先读取工作区里的 ditu2
hasVar = evalin('base', 'exist(''ditu2'', ''var'')');
if hasVar
    data = evalin('base', 'ditu2');
    sourceName = 'base workspace variable: ditu2';
    return;
end

% 否则弹窗选图
[file, path] = uigetfile( ...
    {'*.png;*.jpg;*.jpeg;*.bmp;*.tif', 'Image Files (*.png,*.jpg,*.jpeg,*.bmp,*.tif)'}, ...
    '选择 ditu2 对应的黑白栅格图');

if isequal(file, 0)
    error('未选择任何输入图像。');
end

fullName = fullfile(path, file);
data = imread(fullName);
sourceName = fullName;
end

function G = toGray01(A)
% 转成 [0,1] 灰度
A = double(A);

if ndims(A) == 3
    if max(A(:)) > 1
        A = A / 255;
    end
    G = 0.299 * A(:,:,1) + 0.587 * A(:,:,2) + 0.114 * A(:,:,3);
else
    if max(A(:)) > 1
        A = A / 255;
    end
    G = A;
end

G(G < 0) = 0;
G(G > 1) = 1;
end

function showInputPreview(A)
if isnumeric(A) || islogical(A)
    if ndims(A) == 2
        imagesc(A);
        axis image off;
        colormap(gca, gray(256));
    elseif ndims(A) == 3
        image(uint8(normalizeImageTo255(A)));
        axis image off;
    else
        imagesc(zeros(10));
        axis image off;
    end
else
    imagesc(zeros(10));
    axis image off;
end
end

function B = normalizeImageTo255(A)
A = double(A);
amin = min(A(:));
amax = max(A(:));

if amax > amin
    A = (A - amin) / (amax - amin);
else
    A = zeros(size(A));
end

B = round(255 * A);
B = max(0, min(255, B));
end