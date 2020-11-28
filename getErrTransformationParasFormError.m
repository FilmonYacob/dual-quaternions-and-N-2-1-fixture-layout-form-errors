function [R, errD] = getErrTransformationParasFormError(dqRefFeature,dqMovingFeature,xyzDir,vFacet)
include_namespace_dq
% move out this
dqRefFeature = DQ(dqRefFeature);
dqMovingFeature = DQ(dqMovingFeature);
normalRef = dqRefFeature.P.vec3;
normalFeature = dqMovingFeature.P.vec3;
dFeature = norm(dqMovingFeature.D.vec4);

global aOfRotation
 

switch xyzDir
    case 2
%         aOfRotation = load('aOfRotation.mat');
        axisOfRotation = normalize(aOfRotation.aOfRotation);
        
        [~, planeEquation] = getAPlane(vFacet(1,:),vFacet(2,:),vFacet(3,:),vFacet(1,:),1);
        normalFeature = planeEquation(1:3)';
        %         normalFeature(3) = 0;
        normalFeature = normalize(normalFeature);
        
    case 1
        % No rotation in this axix
        normalFeature = [0,1,0];
        R = zeros(8,1); R(1) = 1;
        newTerPlane = DQ(normalize(normalFeature)) + E_ * 0 * normalFeature;
        
        %         minimumTranslation =
        
        errD = newTerPlane - dqMovingFeature;
        dqRefFeature = newTerPlane;
        normalRef = dqRefFeature.P.vec3;
        %         plot(newTerPlane,'plane',20,'color','g'); hold on;
        
    case 3
        axisOfRotation = getMyNormal([0,0,0],normalRef',normalFeature');
        aOfRotation = normalRef;
%         save('aOfRotation.mat','aOfRotation');
end

% Rotation is needed only in direction of x-2 and z-3
if xyzDir == 2 || xyzDir == 3
    
    alpha = rad2deg(anglePoints3d(normalRef',normalFeature'));
    if alpha > 90
        alpha = 180 - alpha;
    end
    
    R = rot2dquat(alpha,axisOfRotation');
    movedFeature = DQmult(R,dqMovingFeature.vec8,DQconj(R));
    errD = (normalize(dqRefFeature.P) + E_ * dqRefFeature.D) - (normalize(DQ(movedFeature).P) + E_ * DQ(movedFeature).D);
    
 
    % checks rotation direction
    if norm(errD.P.vec4) > 0.0001
        R = rot2dquat(-alpha,axisOfRotation');
        movedFeature = DQmult(R,dqMovingFeature.vec8,DQconj(R));
        errD = (normalize(dqRefFeature.P) + E_ * dqRefFeature.D) - (normalize(DQ(movedFeature).P) + E_ * DQ(movedFeature).D);
    end
    
    %     if xyzDir == 3
    %         delete 'aOfRotation.mat';
    %         RT = vec8(DQ(R)+E_*errD.D/2);
    %         aOfRotation = normalize(DQ(DQmult(RT,dqRefFeature.vec8,DQconj(RT))).P.vec3);
    %         save('aOfRotation.mat','aOfRotation');
    %     end
    
 
end
R =  -R;
R(1) = 1;
%
dqRefFeature = dqRefFeature.vec8;
errD = getProjectedTestPointsPlucker2(movedFeature, [0, 0, 0], normalRef) - ...
    getProjectedTestPointsPlucker2(dqRefFeature, [0, 0, 0], normalRef);
end
