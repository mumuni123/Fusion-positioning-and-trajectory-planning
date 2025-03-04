function imuerr = imuerrset(eb, db, web, wdb, sqrtR0G, TauG, sqrtR0A, TauA, dKGii, dKAii, dKGij, dKAij, Ka2, rxyz, dtGA)
% SIMU errors setting, including gyro & acc bias, noise and installation errors, etc.
%
% Prototype: imuerr = imuerrset(eb, db, web, wdb, sqrtR0G, TauG, sqrtR0A, TauA, dKGii, dKAii, dKGij, dKAij, Ka2, rxyz, dtGA)
% Inputs: including information as follows
%     eb - gyro constant bias (deg/h)
%     db - acc constant bias (ug)
%     web - angular random walk (deg/sqrt(h))
%     wdb - velocity random walk (ug/sqrt(Hz))
%     sqrtR0G,TauG - gyro correlated bias, sqrtR0G in deg/h and TauG in s
%     sqrtR0A,TauA - acc correlated bias, sqrtR0A in ug and TauA in s
%     dKGii - gyro scale factor error (ppm)
%     dKAii - acc scale factor error (ppm)
%     dKGij - gyro installation error (arcsec)
%     dKAij - acc installation error (arcsec)
%     Ka2 - acc quadratic coefficient (ug/g^2)
%       where, 
%                |dKGii(1) dKGij(4) dKGij(5)|         |dKAii(1) 0        0       |
%         dKg  = |dKGij(1) dKGii(2) dKGij(6)| , dKa = |dKAij(1) dKAii(2) 0       |
%                |dKGij(2) dKGij(3) dKGii(3)|         |dKAij(2) dKAij(3) dKAii(3)|
%         dKga = [dKg(:,1); dKg(:,2); dKg(:,3);  dKa(:,1); dKa(2:3,2); dKa(3,3)];
%     rxyz - acc inner-lever-arm, =[rx;ry;rz] (cm)
%     dtGA - time-asynchrony between gyro & acc, dtGA=Tacc_later-Tgyro_early>0 (ms)
% Output: imuerr - SIMU error structure array
%
% Chinese NOTE：为何用eb,db符号？---ε（epsilon）长得像且读音首字恰好是e, ?（nabla）长得像D（或小写d）；第二字母都是b系投影的。
%
% Example:
%     For inertial grade SIMU, typical errors are:
%       eb=0.01dph, db=50ug, web=0.001dpsh, wdb=10ugpsHz
%       scale factor error=10ppm, askew installation error=10arcsec
%       sqrtR0G=0.001dph, taug=1000s, sqrtR0A=10ug, taug=1000s
%    then call this funcion by
%       imuerr = imuerrset(0.01,100,0.001,10, 0.001,1000,10,1000, 10,10,10,10, 10, 10, 10);
%
% See also  imuadderr, gabias, avperrset, insinit, kfinit.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/09/2013, 06/03/2014, 15/02/2015, 22/08/2015, 17/08/2016, 24/07/2020
global glv
    if nargin==1  % for specific defined case 
        switch eb
            case 1,  imuerr = imuerrset(0.01, 100, 0.001, 10);
            case 2,  imuerr = imuerrset(0.01,100,0.001,10, 0.001,300,10,300, 10,10,10,10, 10, 2, 1);
        end
        return;
    end
    o31 = zeros(3,1); o33 = zeros(3);
    imuerr = struct('eb',o31, 'db',o31, 'web',o31, 'wdb',o31,...
        'sqg',o31, 'taug',inf(3,1), 'sqa',o31, 'taua',inf(3,1), 'dKg',o33, 'dKa',o33, 'dKga',zeros(15,1),'Ka2',o31); 
    %% constant bias & random walk
    if ~exist('web', 'var'), web=0; end
    if ~exist('wdb', 'var'), wdb=0; end
    if length(eb)==2, eb=[eb(1);eb(1);eb(2)]; end
    if length(db)==2, db=[db(1);db(1);db(2)]; end
    if length(web)==2, web=[web(1);web(1);web(2)]; end
    if length(wdb)==2, wdb=[wdb(1);wdb(1);wdb(2)]; end
    imuerr.eb(1:3) = eb*glv.dph;   imuerr.web(1:3) = web*glv.dpsh;
    imuerr.db(1:3) = db*glv.ug;    imuerr.wdb(1:3) = wdb*glv.ugpsHz;
    %% correlated bias
    if exist('sqrtR0G', 'var')
        if TauG(1)==inf, imuerr.sqg(1:3) = sqrtR0G*glv.dphpsh;   % algular rate random walk !!!
        elseif TauG(1)>0, imuerr.sqg(1:3) = sqrtR0G*glv.dph.*sqrt(2./TauG); imuerr.taug(1:3) = TauG; % Markov process
        end
    end
    if exist('sqrtR0A', 'var')
        if TauA(1)==inf, imuerr.sqa(1:3) = sqrtR0A*glv.ugpsh;   % specific force random walk !!!
        elseif TauA(1)>0, imuerr.sqa(1:3) = sqrtR0A*glv.ug.*sqrt(2./TauA); imuerr.taua(1:3) = TauA; % Markov process
        end
    end
    %% scale factor error
    if exist('dKGii', 'var')
        imuerr.dKg = setdiag(imuerr.dKg, dKGii*glv.ppm);
    end
    if exist('dKAii', 'var')
        imuerr.dKa = setdiag(imuerr.dKa, dKAii*glv.ppm);
    end
    %% installation angle error
    if exist('dKGij', 'var')
        dKGij = ones(6,1).*dKGij*glv.sec;
        imuerr.dKg(2,1) = dKGij(1); imuerr.dKg(3,1) = dKGij(2); imuerr.dKg(3,2) = dKGij(3); 
        imuerr.dKg(1,2) = dKGij(4); imuerr.dKg(1,3) = dKGij(5); imuerr.dKg(2,3) = dKGij(6);
    end
    if exist('dKAij', 'var')
        dKAij = ones(3,1).*dKAij*glv.sec;
        imuerr.dKa(2,1) = dKAij(1); imuerr.dKa(3,1) = dKAij(2); imuerr.dKa(3,2) = dKAij(3); 
    end
    imuerr.dKga = [imuerr.dKg(:,1); imuerr.dKg(:,2);   imuerr.dKg(:,3);
                   imuerr.dKa(:,1); imuerr.dKa(2:3,2); imuerr.dKa(3,3)];
    %% acc 2nd scale factor error
    if exist('Ka2', 'var')
        imuerr.Ka2(1:3) = Ka2*glv.ugpg2; 
    end
    %% acc inner-lever-arm error
    if exist('rxyz', 'var')
        if length(rxyz)==1, rxyz(1:6)=rxyz; end
        if length(rxyz)==6, rxyz(7:9)=[0;0;0]; end 
        imuerr.rx = rxyz(1:3)/100; imuerr.ry = rxyz(4:6)/100; imuerr.rz = rxyz(7:9)/100;
    end
    %% time-asynchrony between gyro & acc
    if exist('dtGA', 'var')
        imuerr.dtGA = dtGA/1000; 
    end
               
