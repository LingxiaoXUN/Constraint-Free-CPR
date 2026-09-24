function fig=plot_shape(m,state,theta,load,visible,existingFigure)
%PLOT_SHAPE Draw reconstructed rods and rigid parts in millimeters.
% Motor blocks are built around their local attachments and transformed by
% the motor poses. Block and plate dimensions affect only the drawing.
if nargin<5, visible='on'; end
if nargin<6, existingFigure=[]; end
opt=cpr.visual_options(m.cfg);
sample=cpr.sample(m,state,theta); T=cpr.nodes(m,state,theta);
[~,motors]=cpr.base_poses(m,theta);
viewAngles=opt.view; limits=[];
if ~isempty(existingFigure) && isgraphics(existingFigure,'figure')
    fig=existingFigure; ax=fig.CurrentAxes;
    if isempty(ax)
        ax=axes(fig);
    else
        if ~opt.lockView, viewAngles=ax.View; end
        previous=fig.UserData;
        if isstruct(previous) && isfield(previous,'limits_m'), limits=previous.limits_m; end
        cla(ax);
    end
    set(fig,'Visible',visible);
else
    fig=figure('Color','w','Visible',visible,'Name','Six-rod parallel robot',...
        'NumberTitle','off','Position',[160 80 960 780],'MenuBar','none','ToolBar','none');
    ax=axes(fig);
end
hold(ax,'on');
colors=[.12 .43 .65; .90 .43 .16; .22 .59 .47];
rodHandles=gobjects(1,3); bases=zeros(3,6); tips=zeros(3,6);

% Reconstruct the centerline within each element at the material sample points.
for k=1:6
    p=sample.position(:,:,k)*1000; motor=m.motorForRod(k);
    h=plot3(ax,p(1,:),p(2,:),p(3,:),'LineWidth',opt.rodLineWidth,'Color',colors(motor,:));
    rodHandles(motor)=h;
    bases(:,k)=T(1:3,4,1,k)*1000; tips(:,k)=T(1:3,4,end,k)*1000;
end

% Use the convex hull of the platform attachments as the plate outline.
xy=reshape(m.tipLocal(1:2,4,:),2,6);
order=convhull(xy(1,:),xy(2,:)); order=order(1:end-1);
top=tips(:,order)/1000;
bottom=top-opt.platformThickness*state.platform(1:3,3);
draw_prism(ax,top,bottom,[.77 .81 .86],[.28 .33 .40],'platform');
plot3(ax,tips(1,:),tips(2,:),tips(3,:),'o','MarkerSize',4.5,...
    'MarkerFaceColor','w','MarkerEdgeColor',[.30 .35 .41],'LineWidth',.8);

for j=1:3
    ii=find(m.motorForRod==j);
    local=reshape(m.baseLocal(1:3,4,ii),3,2);
    center=mean(local,2);
    long=local(:,2)-local(:,1); span=norm(long); long=long/span;
    normal=m.baseLocal(1:3,3,ii(1));
    normal=normal-long*(long'*normal); normal=normal/norm(normal);
    transverse=cross(normal,long);
    halfLength=span/2+opt.motorEndPadding;
    rectangle=[-1 1 1 -1;-1 -1 1 1];
    topLocal=center+[halfLength*long,opt.motorWidth/2*transverse]*rectangle;
    bottomLocal=topLocal-opt.motorHeight*normal;
    R=motors(1:3,1:3,j); origin=motors(1:3,4,j);
    top=R*topLocal+origin; bottom=R*bottomLocal+origin;
    draw_prism(ax,top,bottom,.72*colors(j,:)+.28,[.21 .27 .33],sprintf('motor%d',j));
    plot3(ax,bases(1,ii),bases(2,ii),bases(3,ii),'o','MarkerSize',5,...
        'MarkerFaceColor','w','MarkerEdgeColor',colors(j,:),'LineWidth',1.1);
    if opt.showMotorLabels
        label=1000*mean(bottom,2); label(3)=1000*min(bottom(3,:))-16;
        label(1:2)=label(1:2)-12*[cosd(viewAngles(1));sind(viewAngles(1))];
        text(ax,label(1),label(2),label(3),sprintf('M%d',j),...
            'FontName','Arial','FontSize',11,'FontWeight','bold',...
            'Color',[.22 .28 .35],'HorizontalAlignment','center','VerticalAlignment','top');
    end
end
if load.tension>0
    [~,~,~,rope]=cpr.pulley_load(state.platform,m.pulleyOffset,m.cfg.pulley.anchor,load.tension);
    p=1000*[rope.attachment,m.cfg.pulley.anchor];
    plot3(ax,p(1,:),p(2,:),p(3,:),'--','Color',[.38 .40 .44],'LineWidth',1.3);
    plot3(ax,p(1,2),p(2,2),p(3,2),'s','MarkerSize',6,'MarkerFaceColor',[.38 .40 .44]);
end

% Keep the initial axis limits throughout the trajectory unless explicitly set.
if ~isempty(opt.axisLimits)
    limits=opt.axisLimits;
elseif isempty(limits)
    limits=default_limits(m,load,sample,opt);
end
set(ax,'Units','normalized','Position',[.10 .13 .80 .73],...
    'FontName','Arial','FontSize',11,'LineWidth',.8,'Box','off',...
    'XColor',[.35 .39 .44],'YColor',[.35 .39 .44],'ZColor',[.35 .39 .44],...
    'GridColor',[.53 .59 .65],'GridAlpha',.14,'TickDir','out','Projection','orthographic');
grid(ax,'on'); daspect(ax,[1 1 1]);
xlim(ax,1000*limits(1,:)); ylim(ax,1000*limits(2,:)); zlim(ax,1000*limits(3,:));
view(ax,viewAngles); axis(ax,'vis3d');
xlabel(ax,'x (mm)'); ylabel(ax,'y (mm)'); zlabel(ax,'z (mm)');
title(ax,'Six-rod parallel robot','FontSize',16,'FontWeight','bold','Color',[.16 .22 .29]);
subtitle(ax,sprintf('\\theta_1 = %.1f°     \\theta_2 = %.1f°     \\theta_3 = %.1f°     |     N = %d',...
    theta*180/pi,m.N),'FontSize',11,'Color',[.36 .41 .46]);
legend(ax,rodHandles,{'Motor 1 rods','Motor 2 rods','Motor 3 rods'},...
    'Orientation','horizontal','Location','southoutside','Box','off','FontSize',10);
set(fig,'UserData',struct('config',m.cfg,'theta_rad',theta,'load',load,...
    'sample',sample,'limits_m',limits));
if opt.lockView
    rotate3d(fig,'off'); pan(fig,'off'); zoom(fig,'off');
    disableDefaultInteractivity(ax);
else
    enableDefaultInteractivity(ax); rotate3d(fig,'on');
end
ax.Toolbar.Visible='off';
end

function draw_prism(ax,top,bottom,color,edge,tag)
% Join matching top and bottom polygons. Input vertices are in meters.
n=size(top,2); verts=1000*[top,bottom]';
faces=[(1:n)',mod((1:n)',n)+1,mod((1:n)',n)+1+n,(1:n)'+n];
patch(ax,'Vertices',verts,'Faces',faces,'FaceColor',.84*color,...
    'EdgeColor',edge,'LineWidth',.65,'Tag',[tag '_sides']);
patch(ax,'Vertices',verts,'Faces',1:n,'FaceColor',color,...
    'EdgeColor',edge,'LineWidth',1.0,'Tag',[tag '_top']);
patch(ax,'Vertices',verts,'Faces',n+(n:-1:1),'FaceColor',.7*color,...
    'EdgeColor',edge,'LineWidth',.65,'Tag',[tag '_bottom']);
end

function limits=default_limits(m,load,sample,opt)
% Set initial plot bounds from the installation geometry, rod length and state.
base=cpr.base_poses(m,zeros(3,1)); p=reshape(base(1:3,4,:),3,6);
center=mean(m.cfg.axisPoints(1:2,:),2);
halfSpan=max(abs(p(1:2,:)-center),[],2)+.22*m.cfg.L;
limits=[center-halfSpan,center+halfSpan];
limits(3,:)=[min(m.cfg.axisPoints(3,:))-.15*m.cfg.L,...
    max(m.motorInitial(3,4,:))+1.13*m.cfg.L];
points=reshape(sample.position,3,[]);
if load.tension>0, points=[points,m.cfg.pulley.anchor]; end
padding=max(opt.motorHeight,.05*m.cfg.L);
limits(:,1)=min(limits(:,1),min(points,[],2)-padding);
limits(:,2)=max(limits(:,2),max(points,[],2)+padding);
end
