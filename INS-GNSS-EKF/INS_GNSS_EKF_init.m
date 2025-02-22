% PSINS Toolbox initialization.
% See also  glvs.
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/09/2013, 31/01/2015

%%%%%%%%%  初始化：设置路径等   %%%%%%%%%%%%%%%%

initfile = dir('INS_GNSS_EKF_init.m');
if isempty(initfile)  
    uiwait(warndlg('Initialization failure!!! Please set Matlab current working directory to PSINS.','PSINS','modal'));
    return;
end
%% Remove old PSINS path from search path.
pp = [';',path,';'];
kpsins = strfind(pp, 'psins');
ksemicolon = strfind(pp, ';');
krm = length(kpsins);
for k=1:krm
	k1 = find(ksemicolon<kpsins(k),1,'last');
	k2 = find(kpsins(k)<ksemicolon,1,'first');
	pk = pp(ksemicolon(k1)+1:ksemicolon(k2)-1);
	rmpath(pk);
end
%% Add new PSINS directories to search path.
rootpath = pwd;
pp = genpath(rootpath);
idx = strfind(pp,';');
for k=length(idx):-1:2  % remove vc60 & doc
    if ~isempty(strfind(pp(idx(k-1)+1:idx(k)),'vc60')), pp(idx(k-1)+1:idx(k))=[];
    elseif ~isempty(strfind(pp(idx(k-1)+1:idx(k)),'doc')), pp(idx(k-1)+1:idx(k))=[]; end
end
mytestflag = 0;
if exist('mytest\mytestinit.m', 'file')
    mytestflag = 1;
end
datapath = [rootpath, '\data\'];
if isempty(find(rootpath=='\',1))  % for Unix
    rootpath(rootpath=='\')='/';
    datapath(datapath=='\')='/';
    pp(pp=='\')='/';
end
addpath(pp);
res = savepath; % disp(res);
%% Create PSINS environment file
fid = fopen('psinsenvi.m', 'wt');
fprintf(fid, 'function [rpath, dpath, mytestflag] = envi()\n');
fprintf(fid, '\trpath = ''%s'';\n', rootpath);
fprintf(fid, '\tdpath = ''%s'';\n', datapath);
fprintf(fid, '\tmytestflag = %d;\n', mytestflag);
fclose(fid);
clear pp rootpath datapath res fid mytestflag;
glvs;
% disp('   *** PSINS Toolbox Initialization Done! ***');
uiwait(msgbox('         INS-GNSS-EKF Simulation Initialization Done!     ','PSINS','modal'));

