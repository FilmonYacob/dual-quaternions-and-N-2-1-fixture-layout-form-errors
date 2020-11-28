function plane = getDQPlane(planeNormal,PlanePoint)
% converts normals and a point to dual quaternion
    include_namespace_dq
    planeNormal = normalize(planeNormal);
    plane = DQ(planeNormal) + E_ * dot(PlanePoint,planeNormal);
end