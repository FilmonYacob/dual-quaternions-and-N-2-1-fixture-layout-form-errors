function Vs = moveVerticesBy(Vs, R, t)
include_namespace_dq

if isstruct(Vs)
    fnames = fieldnames(Vs);
    R(isnan(R)) = 0;
    
    for s = 1:size(fnames, 1)
        V = Vs.(string(fnames(s, 1)));
        total = DQmult(R, pos2dquat(V'), DQconj(R)) + pos2dquat(t');
        Vs.(string(fnames(s, 1))) = total(6:8, :)';
    end
elseif iscell(Vs)
    for s = 1:size(Vs, 2)
        V = Vs{s};
        total = DQmult(R, pos2dquat(V'), DQconj(R)) + pos2dquat(t');
        Vs{s} = total(6:8, :)';
    end
    
else
    total = DQmult(R, pos2dquat(Vs'), DQconj(R)) + pos2dquat(t');
    Vs = total(6:8, :)';
end