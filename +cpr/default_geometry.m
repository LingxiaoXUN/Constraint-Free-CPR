function geometry=default_geometry
%DEFAULT_GEOMETRY Load installation and attachment transforms from geometry.json.
% motorInitial maps motor frames to world coordinates. baseLocal and tipLocal
% map rod end frames to their motor and platform frames, respectively.
root=fileparts(fileparts(mfilename('fullpath')));
g=jsondecode(fileread(fullfile(root,'data','geometry.json')));
geometry.platformInitial=g.platformInitial;
geometry.motorInitial=g.motorInitial;
geometry.baseLocal=zeros(4,4,6);
geometry.tipLocal=zeros(4,4,6);
geometry.motorForRod=zeros(1,6);
geometry.pulleyOffset=g.pulleyOffset(:);
for j=1:numel(g.links)
    link=g.links(j);
    if strcmp(link.side,'start')
        geometry.baseLocal(:,:,link.rod)=link.transform;
        geometry.motorForRod(link.rod)=link.motor;
    else
        geometry.tipLocal(:,:,link.rod)=link.transform;
    end
end
end
