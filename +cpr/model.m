function m=model(cfg)
%MODEL Assemble the fixed six-rod topology and independent-coordinate maps.
% Unknowns are the platform pose, interior rod poses and element strain
% slopes. Motor and platform attachments determine the rod boundary poses.
if isfield(cfg,'geometry'), g=cfg.geometry; else, g=cpr.default_geometry; end
m.cfg=cfg; m.N=cfg.N; m.h=cfg.L/cfg.N;
A=pi*cfg.radius^2; I=pi*cfg.radius^4/4; G=cfg.E/(2*(1+cfg.nu));
m.K=diag([cfg.E*I,cfg.E*I,2*G*I,G*A,G*A,cfg.E*A]);
m.massPerLength=cfg.rho*A; m.energyScale=cfg.E*I/cfg.L;
m.platformInitial=g.platformInitial; m.motorInitial=g.motorInitial;
m.pulleyOffset=g.pulleyOffset(:);
if isfield(cfg.pulley,'offset'), m.pulleyOffset=cfg.pulley.offset(:); end
m.baseLocal=g.baseLocal; m.tipLocal=g.tipLocal; m.motorForRod=g.motorForRod;
m.ndof=6+36*(cfg.N-1)+36*cfg.N;
m.interiorIds=zeros(6,cfg.N-1,6); m.slopeIds=zeros(6,cfg.N,6);
offset=6;
for k=1:6
 for j=1:cfg.N-1, m.interiorIds(:,j,k)=offset+(1:6); offset=offset+6; end
 for e=1:cfg.N, m.slopeIds(:,e,k)=offset+(1:6); offset=offset+6; end
end
assert(offset==m.ndof);
m.scale=zeros(m.ndof,1); m.scale(1:6)=[ones(3,1);cfg.L*ones(3,1)];
m.maps=cell(cfg.N,6); m.indices=cell(cfg.N,6);
% P maps independent increments to [left pose; right pose; strain slope].
% A prescribed base has zero variation. A tip follows the platform through
% Ad(inv(tipLocal)); this removes the attachment constraint equations.
for k=1:6
 for j=1:cfg.N-1, m.scale(m.interiorIds(:,j,k))=[ones(3,1);cfg.L*ones(3,1)]; end
 for e=1:cfg.N
  m.scale(m.slopeIds(:,e,k))=[ones(3,1)/cfg.L^2;ones(3,1)/cfg.L];
  P=zeros(18,m.ndof); % Small temporary map; cache only participating columns.
  if e>1, P(1:6,m.interiorIds(:,e-1,k))=eye(6); end
  if e<cfg.N, P(7:12,m.interiorIds(:,e,k))=eye(6);
  else, P(7:12,1:6)=cpr.SE3.Ad(cpr.SE3.inv(m.tipLocal(:,:,k))); end
  P(13:18,m.slopeIds(:,e,k))=eye(6);
  ids=find(any(P,1)); m.indices{e,k}=ids; m.maps{e,k}=full(P(:,ids));
 end
end
end
