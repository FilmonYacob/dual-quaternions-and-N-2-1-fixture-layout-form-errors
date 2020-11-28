% Predicts part quality given part with form error and N-2-1 layout
clear; clc; close all;
rng(2020) % for repeatability
include_namespace_dq

% Get part
% Get Fixture
%%
PsNominal = getPs12_2_1Fixture();
numExp = 5;
rs = round(myNormalRand(16, numExp, -0.2, 0.2), 2);
rs(10, 3) = -rs(10, 3); % Center of mass passes through the line.
rs(10, 4) = -rs(10, 4);
ParallelismKPC1 = [];
ParallelismKPC2 = [];
k = 0;
%%
for expNum = 1:numExp % 5 experiments
    expNum
    [vFeatures, FFs] = getPocketedPart();

    % create cutting planes
    cuttingPlaneNominalStn1 = vFeatures{1};
    cuttingPlaneNominalStn2 = vFeatures{1};

    cuttingPlaneNominalStn2(:, 3) = cuttingPlaneNominalStn2(:, 3) - 20; %S1
    cuttingPlaneNominalStn10 = vFeatures{10};
    cuttingPlaneNominalStn10(:, 2) = cuttingPlaneNominalStn10(:, 2) * 0 + 160; %S2

    %     [vFeatures, FFs] = getPocketedPartDownSample(numExp, percent_);

    Ps = PsNominal;
    Ps(1:12, 3) = Ps(1:12, 3) + rs(1:12, expNum);
    Ps(13:14, 2) = Ps(13:14, 2) + rs(13:14, expNum);
    Ps(15, 1) = Ps(15, 1) + rs(15, expNum);

    % convert fixture to a plane with form errors
    FFixPrim = delaunay(Ps(1:12, 1), Ps(1:12, 2));
    vFixPrim = Ps(1:12, :);
    %% tool deviation from nominal
    tooltipDev = rs(16, expNum);

    % Operation 1
    %
    %disp('Operation 1 ...')
    stn = 1;
    nMachinedF = 1;
    vFeatures = assemblePartwithFormTo_12_2_1_Locators(Ps, vFeatures, FFs, vFixPrim, FFixPrim, stn);

    % Machine the part by replacing the machining feature
    vFeatures{nMachinedF} = cuttingPlaneNominalStn1 + [0, 0, tooltipDev];
  
    % Operation 2
    %%
    % %disp('Operation 2 ...')
    stn = 2;
    nMachinedF1 = 3; % index of machining feature 1
    nMachinedF2 = 10; % index of machining feature 2
    nPrim = 1;

    % Rotate 180 degree around x axis
    R = rot2dquat(180, [1, 0, 0]');
    vFeatures = moveVerticesBy(vFeatures, R, [0, 200, 121]);

    % Assemble to fixture
    vFeatures = assemblePartwithFormTo_12_2_1_Locators(Ps, vFeatures, FFs, vFixPrim, FFixPrim, stn);

    % Machine the part
    R = zeros(8, 1); R(1) = 1;
    vFeatures{nMachinedF1} = moveVerticesBy(cuttingPlaneNominalStn2, R, [0, 0, tooltipDev]);
    vFeatures{nMachinedF2} = moveVerticesBy(cuttingPlaneNominalStn10, R, [0, 0, 0]);
    %% Move to inspection station and conduct measurement
                vFeatures = moveToInpsectonStation(vFeatures, FFs, nPrim, [0 0 1], vFixPrim, FFixPrim)
%     vFeatures = moveToInspectionStationPntCld(vFeatures, nPrim, numExp); %double check using 'pctransform' of matlab
    %%
    kpc1 = abs(conductMeasurement(vFeatures, FFs, nMachinedF1, [0, 0, -1]));
    kpc2 = abs(conductMeasurement(vFeatures, FFs, nMachinedF2, [0, -1, 0])) - ...
        abs(conductMeasurement(vFeatures, FFs, 4, [0, -1, 0]));

    k = k + 1;
    KPC1{k} = kpc1(:, 3);
    KPC2{k} = abs(kpc2(:, 2));
    SZ{k} = size(vFeatures{nMachinedF1}, 1);
    ParallelismKPC1 = [ParallelismKPC1; expNum, round(max(kpc1(:, 3))-min(kpc1(:, 3)), 4)];
    ParallelismKPC2 = [ParallelismKPC2; round(max(kpc2(:, 2))-min(kpc2(:, 2)), 4)];
    disp(expNum)
end
%%
close  all
%% Compare with CAD/CAM readings
Paralellism = [ParallelismKPC1, ParallelismKPC2];
for kpcs = 1:2
    if kpcs == 1
        CADCAM = [100.2592, 100.3312, 99.5152, 99.5872; ...
            100.2948, 99.9485, 100.6617, 100.3154; ...
            99.7529, 99.7975, 99.5472, 99.5918; ...
            99.8578, 99.7138, 99.7218, 99.5778; ...
            99.9734, 99.6854, 100.0704, 99.7794];
        KPC = KPC1;
        ParalellismCADCAM = max(CADCAM, [], 2) - min(CADCAM, [], 2);
        percentageErrorKPC1 = abs((ParalellismCADCAM - Paralellism(:, 2))*100./ParalellismCADCAM);

    else

        CADCAM = [39.6987, 39.7234, 40.1460, 40.1706; ...
            39.6412, 39.5260, 39.0194, 38.9041; ...
            39.4321, 39.4470, 39.6285, 39.6434; ...
            39.9374, 39.8889, 39.6972, 39.6488; ...
            39.8892, 39.7910, 39.3351, 39.2312];

        KPC = KPC2;
        ParalellismCADCAM2 = max(CADCAM, [], 2) - min(CADCAM, [], 2);
        percentageErrorKPC2 = abs((ParalellismCADCAM2 - Paralellism(:, 3))*100./ParalellismCADCAM2);
    end

    errs = [];
    for i = 1:5
        cKPC = KPC{i};
        errs = [errs; (cKPC - CADCAM(i, :)')];
    end

    figure
    errs = abs([errs(1:4, :)'; errs(5:8, :)'; errs(9:12, :)'; errs(13:16, :)'; errs(17:20, :)']);
    X = categorical({'1', '2', '3', '4', '5'});
    b = bar(X, errs);
    xlabel('Experiment')
    ylabel('Prediction error (mm)')
end

ParalellismCADCAMs = [ParalellismCADCAM, ParalellismCADCAM2];
Paralellism;
%%
function vFeatures = assemblePartwithFormTo_12_2_1_Locators(Ps, vFeatures, FFs, vFixPrim, FFixPrim, stn)
% Iteratively adjusts to a new position%

switch stn
    case 1
        nPrim = 2; % datum index
        nSec = 4;
    case 2
        nPrim = 1;
        nSec = 6;
end
%%
maxIter = 0;
while maxIter < 1
    maxIter = maxIter + 1;
    %disp(strcat('Iteration ', string(maxIter), '...'))

    rayDir = [0, 0, -1]; % due to gravity
    %disp('Assemblying primary...')
    [vFeatures, normalPrim, iStableLocs] = AssembleToPrimaryDataum(vFeatures, FFs, nPrim, rayDir, vFixPrim, FFixPrim);

    %disp('Assemblying secondary...')
    vFeatures = AssembleToSecondaryDataum(Ps, vFeatures, FFs, nSec, normalPrim, iStableLocs);

    nTer = 5;
    %disp('Assemblying teriary...')
    vFeatures = AssembleToTeriaryDataum(Ps, vFeatures, FFs, nTer, normalPrim, iStableLocs);
end
%
%% Check for interference
disp(iStableLocs) % check the stable contacts

% primary
tol = 2 * 10^-4;
vFeature = vFeatures{nPrim};
FF1 = FFs{nPrim};
for p = 1:3
    [intersect, t, ~, ~, xcoor] = TriangleRayIntersection( ...
        Ps(iStableLocs(p), :), rayDir, vFeature(FF1(:, 1), :), vFeature(FF1(:, 2), :), vFeature(FF1(:, 3), :), 'lineType', 'line');
    interPnts = real(unique(round(xcoor(intersect, :), 6), 'rows'));
    assert(norm(Ps(iStableLocs(p), :)-interPnts) < tol)
end

% secondary
vFeature = vFeatures{nSec};
FF1 = FFs{nSec};
for p = 13:14
    [intersect, t, ~, ~, xcoor] = TriangleRayIntersection( ...
        Ps(p, :), [0 1 0], vFeature(FF1(:, 1), :), vFeature(FF1(:, 2), :), vFeature(FF1(:, 3), :), 'lineType', 'line');
    interPnts = real(unique(round(xcoor(intersect, :), 6), 'rows'));
    assert(norm(Ps(p, :)-interPnts) < tol)
end

% teriary
vFeature = vFeatures{nTer};
FF1 = FFs{nTer};
[intersect, t, ~, ~, xcoor] = TriangleRayIntersection( ...
    Ps(15, :), [1 0 0], vFeature(FF1(:, 1), :), vFeature(FF1(:, 2), :), vFeature(FF1(:, 3), :), 'lineType', 'line');
interPnts = real(unique(round(xcoor(intersect, :), 6), 'rows'));
assert(norm(Ps(15, :)-interPnts) < tol)
end
%%
function [vFeatures, normalPrim, iStableLocs] = AssembleToPrimaryDataum(vFeatures, FFs, nPrim, rayDir, vFixPrim, FFixPrim)
% gets the difference surface, apply convexhull, stable plane, assemble the
% plane to a perfect plane, transform vFeatures by the same amount

include_namespace_dq

tol = 2 * 10^-4;

asseDir = find(abs(rayDir) == max(abs(rayDir)));
vFeature = vFeatures{nPrim};
FFPrim = FFs{nPrim};
vMean = mean(vFeature);
% myScatter(vMean,'r*'); hold on
[diffSurf, ~, ~] = projPntsGetDiffSurf(vFeature, FFPrim, rayDir, vFixPrim, FFixPrim, asseDir);
% myScatter(diffSurf,'c*'); hold on

try
    [hF, ~] = convhull([diffSurf(:, 1), diffSurf(:, 2), diffSurf(:, 3)]);
catch
    % induce small variation if points are coplanar
    diffSurf(:, 3) = diffSurf(:, 3) + rand(size(diffSurf, 1), 1) * 10^-6;

    if size(diffSurf, 1) == 3
        hF = [1, 2, 3];
    else
        [hF, ~] = convhull([diffSurf(:, 1), diffSurf(:, 2), diffSurf(:, 3)]);
    end
end

% all facets
allPlanes = [diffSurf(hF(:, 1), :), diffSurf(hF(:, 2), :), diffSurf(hF(:, 3), :)];

% get stable facets
%%
[v8planes, fPs, Vs] = getAllStablevFacets(allPlanes, rayDir, vMean);

iContactPlane = find(fPs(:, asseDir) == max(fPs(:, asseDir))); % points that are top most

% stable plane (DQ) and vertices
v8planeActual = v8planes(:, iContactPlane(1));
P1 = Vs(iContactPlane, 1:3);
P2 = Vs(iContactPlane, 4:6);
P3 = Vs(iContactPlane, 7:9);
vFacet = [P1; P2; P3]; % points of a stable plane

[k1, ~] = ismember(diffSurf(:, 1:3), P1, 'rows');
[k2, ~] = ismember(diffSurf(:, 1:3), P2, 'rows');
[k3, ~] = ismember(diffSurf(:, 1:3), P3, 'rows');

psi = (1:12)';
iStableLocs = [psi(k1), psi(k2), psi(k3)];

% Transform the part to new position
%%
v8PerfectPlane = vec8(k_); % Due to gravitational force's direction
[R, err] = getErrTransformationParasFormError(v8PerfectPlane, v8planeActual, asseDir, []);

% Rotate
v8PerfectPlane = moveFeaturesBy(v8PerfectPlane, R);
% rotate part by the same magnitude
vFeatures = moveVerticesBy(vFeatures, R, [0, 0, 0]);

% Translate
R = zeros(8, 1); R(1) = 1;
T = vec8(1-E_*err(1, :)/2);
v8PerfectPlane = moveFeaturesBy(v8PerfectPlane, T);

% Translate part by the same magnitude
vFeatures = moveVerticesBy(vFeatures, R, err); % err-coordiantes differ
err = getProjectedTestPointsPlucker2(v8PerfectPlane, [0, 0, 0], v8PerfectPlane(2:4, 1)) - ...
    getProjectedTestPointsPlucker2(v8planeActual, [0, 0, 0], v8PerfectPlane(2:4, 1));
assert(norm(err) < tol)

% Info for secondary and teriary direction
normalPrim = v8PerfectPlane(2:4);
end
%%
function [diffSurf, iFeatureCorrsPoint, interDists] = projPntsGetDiffSurf(vFeature, FFPrim, rayDir, vFixPrim, F, asseDir)
iFeatureCorrsPoint = [];
diffSurf = [];
interPnts = [];
interDists = [];
singedDists = [];

for f = 1:size(vFixPrim, 1)
    fPoint = vFixPrim(f, :);
    [intersect, t, ~, ~, xcoor] = TriangleRayIntersection(fPoint, -rayDir, vFeature(FFPrim(:, 1), :), vFeature(FFPrim(:, 2), :), vFeature(FFPrim(:, 3), :), 'lineType', 'line');

    interPnt = unique(round(xcoor(intersect, :), 6), 'rows');
    interDist = unique(round(t(intersect, :), 6), 'rows');
    singedDists = [singedDists; interDist];

    if ~isempty(interPnt)
        iFeatureCorrsPoint = [iFeatureCorrsPoint; interPnt];
        diffSurf = [diffSurf; getDiffSurf(interPnt, fPoint, asseDir)];
        interPnts = [interPnts; interPnt];
        interDists = [interDists; interDist];
    end
end
end
%%
function diffSurf = getDiffSurf(target, moving, asseDir)
switch asseDir
    case 1 % in x direction
        diffSurf = [moving(:, 1) - target(:, 1), moving(:, 2:3)];
    case 2 % in y direction
        diffSurf = [moving(:, 1), (moving(:, 2) - target(:, 2)), moving(:, 3)];
    case 3 % in z direction
        diffSurf = [moving(:, 1:2), moving(:, 3) - target(:, 3)];
end
end
%%
function [v8planes, fPs, Vs] = getAllStablevFacets(allPlanes, rayDir, vMean)
% checks for stability
include_namespace_dq
v8planes = [];
 
fPs = [];
Vs = [];
L = Plucker.pointdir(vMean, rayDir);
for i = 1:size(allPlanes, 1)
    P1 = allPlanes(i, 1:3);
    P2 = allPlanes(i, 4:6);
    P3 = allPlanes(i, 7:9);

    [intersect, ~, ~, ~, xcoor] = TriangleRayIntersection(vMean, [0, 0, 1], P1, P2, P3, 'lineType', 'line');
    interPnt = unique(round(xcoor(intersect, :), 6), 'rows');

    if ~isempty(interPnt) && inOrOutTraingle(P1, P2, P3, interPnt, 3) % get stable vFacet
        [~, planeEquation] = getAPlane(P1, P2, P3, P1, 3);
        v8planes = [v8planes, vec8(getDQPlane(planeEquation(1:3), P2))];
        fPs = [fPs; interPnt];
        Vs = [Vs; allPlanes(i, :)]; % Vertices of convexhull
    end
end

end
%%
function vFeatures = AssembleToSecondaryDataum(Ps, vFeatures, FFs, nSec, normalPrim, iStableLocs)
% Assemble to the secondary datum

tol = 2 * 10^-3;

[~, ULPT] = getDirectionofAssembly(Ps, normalPrim, iStableLocs);

vFeature = vFeatures{nSec};
FF2 = FFs{nSec};

interPnts = [];
for f = 13:14
    fPoint = Ps(f, :);

    [intersect, ~, ~, ~, xcoor] = TriangleRayIntersection(fPoint, ULPT', vFeature(FF2(:, 1), :), vFeature(FF2(:, 2), :), vFeature(FF2(:, 3), :), 'lineType', 'line');
    interPnt = unique(round(xcoor(intersect, :), 6), 'rows');

    if ~isempty(interPnt)
        interPnts = [interPnts; interPnt];
    end
end

% Rotate part to make it parallel with locators
p13p14vec = interPnts(1, :) - interPnts(2, :);

p13p14vec(3) = p13p14vec(3);

p13_14_dir_test = Ps(13, :) - Ps(14, :);
alpha = rad2deg(anglePoints3d(normalize(Ps(13, :)-Ps(14, :)), normalize(p13p14vec)));

if alpha > 90
    alpha = 180 - alpha;
end

% if abs(alpha)<tol
if p13_14_dir_test(2) > 0
    R = rot2dquat(-alpha, normalPrim);
    interPntsTemp = moveVerticesBy(interPnts, R, [0, 0, 0]);
    errTemp = Ps(13:14, :) - [interPntsTemp(1, :); interPntsTemp(2, :)];

    if abs(errTemp(1, 2)-errTemp(2, 2)) > tol
        R = rot2dquat(alpha, normalPrim);
        interPnts = moveVerticesBy(interPnts, R, [0, 0, 0]);
        err = Ps(13:14, :) - [interPnts(1, :); interPnts(2, :)];

        assert(abs(err(1, 2)-err(2, 2)) < tol); % checks equal distance

    else
        interPnts = interPntsTemp;
        err = errTemp;
    end
else
    R = rot2dquat(alpha, normalPrim);
    interPntsTemp = moveVerticesBy(interPnts, R, [0, 0, 0]);
    errTemp = Ps(13:14, :) - [interPntsTemp(1, :); interPntsTemp(2, :)];

    if abs(errTemp(1, 2)-errTemp(2, 2)) > tol
        R = rot2dquat(-alpha, normalPrim);
        interPnts = moveVerticesBy(interPnts, R, [0, 0, 0]);
        err = Ps(13:14, :) - [interPnts(1, :); interPnts(2, :)];
        
        assert(abs(err(1, 2)-err(2, 2)) < tol); % checks equal distance
    else
        interPnts = interPntsTemp;
        err = errTemp;
    end
end
vFeatures = moveVerticesBy(vFeatures, R, [0, 0, 0]);

% Translational motion
R = zeros(8, 1);
R(1) = 1;

vFeatures = moveVerticesBy(vFeatures, R, err(1, :));
interPnts = moveVerticesBy(interPnts, R, err(1, :));

err2 = Ps(13:14, :) - [interPnts(1, :); interPnts(2, :)];
assert(abs(norm(err2(:, 2))) < tol);

%disp('Passed secondary check')
end
%%
function vFeatures = AssembleToTeriaryDataum(Ps, vFeatures, FFs, nPST, normalPrim, iStableLocs)
% Assemble to the Teriary datum
tol = 10^-8;

[ULPS, ~] = getDirectionofAssembly(Ps, normalPrim, iStableLocs);

vFeature = vFeatures{nPST};
FF1 = FFs{nPST};

[intersect, t, ~, ~, xcoor] = TriangleRayIntersection( ...
    Ps(15, :), ULPS, vFeature(FF1(:, 1), :), vFeature(FF1(:, 2), :), vFeature(FF1(:, 3), :), 'lineType', 'line');
interPnts = real(unique(round(xcoor(intersect, :), 6), 'rows'));

% Translate to teriary locator
R = zeros(8, 1);
R(1) = 1;
t = Ps(15, :) - interPnts;
vFeatures = moveVerticesBy(vFeatures, R, t(1, :));
interPnts = moveVerticesBy(interPnts, R, t(1, :));
assert(norm(Ps(15, :)-interPnts) < tol)
%disp('Passed Tertiary check')
end
%%
function [ULPS, ULPT] = getDirectionofAssembly(Ps, normalPrim, iStableLocs)
% get intersection lines and Direction based on fixture orientation

primaryF = getDQPlane(normalPrim, Ps(iStableLocs(1), :)');
P1314 = [Ps(13, 1:2), 0];

[~, planeEquationS] = getAPlane(Ps(13, :), Ps(14, :), P1314, Ps(13, :), 2);
secondaryF = getDQPlane(planeEquationS(1:3), Ps(13, 1:3));

nTao = cross(normalPrim, planeEquationS(1:3)');
tertiaryF = getDQPlane(nTao, Ps(15, :));

fixDatums.PrimaryF = primaryF;
fixDatums.SecondaryF = secondaryF;
fixDatums.TertiaryF = tertiaryF;

fixDatums = [fixDatums.PrimaryF.vec8, ...
    fixDatums.SecondaryF.vec8, ...
    fixDatums.TertiaryF.vec8];

ULPS = checkAndFlipNormalDir(cross(normalPrim, fixDatums(2:4, 2)));
ULPT = checkAndFlipNormalDir(cross(normalPrim, fixDatums(2:4, 3)));

end
%%
function vFeatures = moveToInpsectonStation(vFeatures, FFs, nPrim, rayDir, vFixPrim, FFixPrim)
include_namespace_dq

%disp('Assemblying to granite...')

tol = 10^-4;

% Flat granite
asseDir = find(abs(rayDir) == max(abs(rayDir)));
vFeature = vFeatures{nPrim};
vMean = mean(vFeature);

try
    [hF, ~] = convhull([vFeature(:, 1), vFeature(:, 2), vFeature(:, 3)]);
catch
    vFeature(:, 3) = vFeature(:, 3) + rand(size(vFeature, 1), 1) * 10^-6;
    [hF, ~] = convhull([vFeature(:, 1), vFeature(:, 2), vFeature(:, 3)]);
end
allPlanes = [vFeature(hF(:, 1), :), vFeature(hF(:, 2), :), vFeature(hF(:, 3), :)];

% gets stable facets
%%
[v8planes, Ps, Vs] = getAllStablevFacets(allPlanes, rayDir, vMean);
iContactPlane = find(Ps(:, asseDir) == max(Ps(:, asseDir))); % points that are top most

% stable plane (DQ) and vertices
v8planeActual = v8planes(:, iContactPlane(1));

% Transform the part to new position
%%
v8PerfectPlane = vec8(k_);
[R, err] = getErrTransformationParasFormError(v8planeActual, v8PerfectPlane, asseDir, []);
R = -R;
R(1) = 1;

% Rotate
v8planeActualTemp = moveFeaturesBy(v8planeActual, R);
if sum(abs(v8planeActualTemp(2:4))) > tol
    R = -R; R(1) = 1;
end

v8planeActual = moveFeaturesBy(v8planeActual, R);
vFeatures = moveVerticesBy(vFeatures, R, [0, 0, 0]);

% Translate
err = getProjectedTestPointsPlucker2(v8PerfectPlane, [0, 0, 0], v8PerfectPlane(2:4, 1)) - ...
    getProjectedTestPointsPlucker2(v8planeActual, [0, 0, 0], v8PerfectPlane(2:4, 1));
R = zeros(8, 1); R(1) = 1;
%%
vFeatures = moveVerticesBy(vFeatures, R, err);
vpErr = vFeatures{nPrim};
assert(abs(sum(vpErr(:, 3))) < 0.1)

%disp('Assembled to granite');
end
%%
function interPnts = conductMeasurement(vFeatures, FFs, nMachinedF, rayDir)
%%
%disp('Perfoming inspection...')

vFeature = vFeatures{nMachinedF};
FF = FFs{nMachinedF};

if nMachinedF == 3
    tps = [40, 40, 120; 40, 160, 120; 160, 40, 120; 160, 160, 120];
elseif nMachinedF == 10 || 4
    tps = [40, 210, 100; 40, 210, 120; 160, 210, 100; 160, 210, 120];
end

interPnts = zeros(4, 3);
for i = 1:size(tps, 1)
    [intersect, t, ~, ~, xcoor] = TriangleRayIntersection( ...
        tps(i, :), rayDir, vFeature(FF(:, 1), :), vFeature(FF(:, 2), :), vFeature(FF(:, 3), :), 'lineType', 'line');
    interPnts(i, :) = unique(round(xcoor(intersect, :), 6), 'rows');
end
end
%%
function vFeatures = moveToInspectionStationPntCld(vFeatures, nAssem)
include_namespace_dq

vFeature = real(vFeatures{6});
[vfeaturesNominal, ~] = getPocketedPart();
dtm = vFeature;
dtm(:, 2) = dtm(:, 2) * 0; % Flat granite

[tform, ~] = pcregistericp(pointCloud(vFeature), pointCloud(dtm));

for s = 1:size(vFeatures, 2)
    pnt = vFeatures{s};
    stockTranformed = pctransform(pointCloud(real(pnt)), tform);
    vFeatures{s} = stockTranformed.Location;
end

vFeature = vFeatures{5};
[vfeaturesNominal, ~] = getPocketedPart();
dtm = vFeature;
dtm(:, 1) = dtm(:, 1) * 0; % Flat granite
[tform, ~] = pcregistericp(pointCloud(vFeature), pointCloud(dtm));

for s = 1:size(vFeatures, 2)
    pnt = vFeatures{s};
    stockTranformed = pctransform(pointCloud(pnt), tform);
    vFeatures{s} = stockTranformed.Location;
end
%%
%disp('Assemblying to granite...')
vFeature = vFeatures{nAssem};
[vfeaturesNominal, ~] = getPocketedPart();
dtm = vFeature;
dtm(:, 3) = dtm(:, 3) * 0; % Flat granite
[tform, ~] = pcregistericp(pointCloud(vFeature), pointCloud(dtm));

for s = 1:size(vFeatures, 2)
    pnt = vFeatures{s};
    stockTranformed = pctransform(pointCloud(pnt), tform);
    vFeatures{s} = stockTranformed.Location;
end

tol = 10^-5;
vpErr = vFeatures{nAssem};
assert(abs(sum(vpErr(:, 3))) < tol)
%disp('Assembled to granite');
end
