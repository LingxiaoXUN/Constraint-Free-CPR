function save_run(out,run)
%SAVE_RUN Write MAT states, CSV trajectories and JSON run metadata.
% Trajectory rows describe accepted states; attempts.csv includes rejected
% continuation trials. Rotation matrices map body coordinates to world coordinates.
if ~isfolder(out), mkdir(out); end
save(fullfile(out,'states.mat'),'run','-v7');
fid=fopen(fullfile(out,'config.json'),'w'); fprintf(fid,'%s',jsonencode(run.cfg,PrettyPrint=true)); fclose(fid);
rows=struct([]);
for i=1:numel(run.records)
 r=run.records(i); T=r.state.platform;
 row=struct('sample',i,'parameter',r.parameter,'max_motor_deg',max(abs(r.theta))*180/pi,...
  'theta1_rad',r.theta(1),'theta2_rad',r.theta(2),'theta3_rad',r.theta(3),...
  'x_m',T(1,4),'y_m',T(2,4),'z_m',T(3,4),'updates',r.info.updates,...
  'scaled_residual',r.info.residualScaled,'seconds',r.info.seconds,'accepted',r.info.converged);
 for ii=1:3,for jj=1:3,row.(sprintf('R%d%d',ii,jj))=T(ii,jj);end,end
 physical=r.theta(run.cfg.commandForMotor);
 for motor=1:3,row.(sprintf('motor%d_rad',motor))=physical(motor);end
 if isfield(r,'load')
  row.tension_N=r.load.tension;
  row.gravity_x=r.load.gravity(1);row.gravity_y=r.load.gravity(2);row.gravity_z=r.load.gravity(3);
 end
 if isfield(r,'step'),row.step=r.step;row.time_s=r.parameter;end
 rows=[rows,row]; %#ok<AGROW>
end
if ~isempty(rows), writetable(struct2table(rows),fullfile(out,'trajectory.csv')); end
if run.completed && ismember(string(run.caseName),["unloaded","loaded"])
 targets=[0,5+(0:9)*2*pi/(10*run.cfg.omega)]; indices=zeros(size(targets));
 for j=1:numel(targets)
  [gap,indices(j)]=min(abs([run.records.parameter]-targets(j)));
  assert(gap<1e-10,'cpr:PhaseSample','An exact phase sample is missing.');
 end
 writetable(struct2table(rows(indices)),fullfile(out,'phase_samples.csv'));
end
arows=struct([]);
for i=1:numel(run.attempts)
 a=run.attempts(i); inf=a.info;
 row=struct('attempt',i,'target_parameter',a.parameter,'fraction',a.targetFraction,...
  'accepted',inf.converged,'reason',string(inf.reason),'updates',inf.updates,'residual',inf.residualScaled,'seconds',inf.seconds);
 if isfield(a,'stage')
  row.stage=string(a.stage);row.step=a.step;row.command_accepted=a.commandAccepted;
 end
 arows=[arows,row]; %#ok<AGROW>
end
if ~isempty(arows), writetable(struct2table(arows),fullfile(out,'attempts.csv')); end
fid=fopen(fullfile(out,'status.json'),'w');fprintf(fid,'%s',jsonencode(struct('completed',run.completed,'environment',run.environment,'records',numel(run.records),'attempts',numel(run.attempts)),PrettyPrint=true));fclose(fid);
end
