function gpsplot(vpGPS, ts)
% GPS plot.
%
% Prototype: gpsplot(vpGPS, ts)
% Inputs: vpGPS - [vnGPS, posGPS, tag, t] or [posGPS, tag, t]
%              the tag column may not exist.
%         ts - GPS sampling interval
%          
% See also  imuplot, insplot, gpsload, gpssimu, avpfile, igsplot, gpssatplot, pos2dplot, pos2bd.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 19/02/2014
global glv
    [m,n] = size(vpGPS);
    if n==8 || n==5  % 14/9/2024
        gpsplot(vpGPS(:,[1:end-2,end]));
        satNum = fix(vpGPS(:,end-1));  DOP = (vpGPS(:,end-1)-satNum)*1000;
        myfig,
        subplot(211), plot(vpGPS(:,end), satNum);  xygo('satNum');
        subplot(212), plot(vpGPS(:,end), DOP);  xygo('DOP');
        return;
    end
    if nargin<2, ts=1; end
    if mod(n,3)==0, vpGPS = [vpGPS,(1:length(vpGPS))'*ts]; end
    t = vpGPS(:,end);
    if t(1)>10000
        t = t+(t(2)-2*t(1));
    end
    if mod(n,3)==1  % if not exist tag, then add 1
        vpGPS=[vpGPS(:,1:end-1),ones(m,1),vpGPS(:,end)];  
    end
    idx = find(vpGPS(:,end-1)==0);
    myfigure,
    if n>=6  % if include velocity & position
        vnS = vpGPS(:,1:3); posS = vpGPS(:,4:6);
%         subplot(221), plot(t, vnS); xygo('V');
        subplot(221), plot(t, [vnS,normv(vnS)]); xygo('V');
        hold on, plot(t(idx), vnS(idx,:), 'c*');
        subplot(223), plot(t, [[posS(:,1)-posS(1,1),(posS(:,2)-posS(1,2))*cos(posS(1,1))]*glv.Re,posS(:,3)-posS(1,3)]); xygo('DP');
            hold on, plot(t(idx), [[posS(idx,1)-posS(1,1),(posS(idx,2)-posS(1,2))*cos(posS(1,1))]*glv.Re,posS(idx,3)-posS(1,3)], 'c*');
        subplot(2,2,[2,4]), plot3(r2d(posS(:,2)), r2d(posS(:,1)), posS(:,3)); xygo('lon', 'lat');
            hold on, plot3(r2d(posS(idx,2)), r2d(posS(idx,1)), posS(idx,3), 'c*'), plot3(r2d(posS(1,2)), r2d(posS(1,1)), posS(1,3), 'rp');
    else  % position only
        posS = vpGPS(:,1:3);
        subplot(121), plot(t, [[posS(:,1)-posS(1,1),(posS(:,2)-posS(1,2))*cos(posS(1,1))]*glv.Re,posS(:,3)-posS(1,3)]); xygo('DP');
            hold on, plot(t(idx), [[posS(idx,1)-posS(1,1),(posS(idx,2)-posS(1,2))*cos(posS(1,1))]*glv.Re,posS(idx,3)], 'c*');
        title(sprintf('pos0=[%.6f, %.6f, %.3f]', r2dm(posS(1,1)), r2dm(posS(1,2)), posS(1,3)));
        subplot(122), plot3(r2d(posS(:,2)), r2d(posS(:,1)), posS(:,3)); xygo('lon', 'lat'); zlabel(labeldef('H'));
            hold on, plot3(r2d(posS(idx,2)), r2d(posS(idx,1)), posS(idx,3), 'c*'), plot3(r2d(posS(1,2)), r2d(posS(1,1)), posS(1,3), 'rp');
    end
