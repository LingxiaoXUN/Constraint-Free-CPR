function report=run_tests
%RUN_TESTS Derivatives, KKT regression fixtures and the public control API.
root=fileparts(mfilename('fullpath'));addpath(root,fullfile(root,'tests'));
report.core=test_core; report.regression=test_legacy_fixtures;
report.control=test_control_interface;
report.matlab=version; report.timestamp=char(datetime('now'));
out=fullfile(root,'results');if ~isfolder(out),mkdir(out);end
fid=fopen(fullfile(out,'tests.json'),'w');fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
fprintf('All robot-control package checks passed.\n');
end
