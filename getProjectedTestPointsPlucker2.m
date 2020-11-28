function testPnts = getProjectedTestPointsPlucker2(v8WorkPiece, tp, dirs)
% gets projeced points for given test points TP and direction DIRS 

testPnts = zeros(size(tp, 1), 3);

for i = 1:size(tp, 1)
    if isempty(dirs)
        f = find(abs(v8WorkPiece(2:4, i)) == max(abs(v8WorkPiece(2:4, i))));
        if f == 3
            dir = [0, 0, 1];
        elseif f == 2
            dir = [0, 1, 0];
        elseif f == 1
            dir = [1, 0, 0];
        end
        L = Plucker.pointdir(tp(i, :), dir);
        p = L.intersect_plane2(v8WorkPiece(2:5, i));
    else
        dir = dirs;
        L = Plucker.pointdir(tp(i, :), dirs);
        p = L.intersect_plane2(v8WorkPiece(2:5));
                 
    end
    testPnts(i, :) = p';
end
end