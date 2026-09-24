function opt=visual_options(cfg)
%VISUAL_OPTIONS Merge display defaults with cfg.visualization.
% Missing fields use defaults, so archived configurations can still be plotted.
opt=struct('enabled',false,'every',1,'showMotorLabels',true,...
    'view',[38 24],'lockView',true,'axisLimits',[],...
    'motorWidth',.026,'motorHeight',.010,'motorEndPadding',.006,...
    'platformThickness',.004,'rodLineWidth',2.6);
if isfield(cfg,'visualization')
    names=fieldnames(cfg.visualization);
    for j=1:numel(names), opt.(names{j})=cfg.visualization.(names{j}); end
end
end
