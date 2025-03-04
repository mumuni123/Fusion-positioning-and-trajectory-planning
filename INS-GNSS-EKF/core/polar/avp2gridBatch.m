function avp = avp2gridBatch(avp, n2g)
% Batch processing for AVP translation, from ENU-frame to grid-frame,
% or reverse.
%
% Prototype: avp = avp2gridBatch(avp, n2g)
% Input: avp - [att,vn,blh] in ENU-frame, if norm(blh)>Re/2 for in grid-frame
%        n2g - ==1 from ENU-frame to grid-frame, ==0 reverse
% Output: avp - [attG,vG,pe] in ECEF-frame, or in ENU-frame
%
% See also  avp2ecef, blh2xyz, avp2ecefBatch.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 28/11/2024
global glv
    if nargin<2, n2g=1; end
    if norm(avp(1,7:9))>glv.Re/2, n2g=0; end
    if n2g==0  % grid-xyz to avp-blh
        for k=1:size(avp,1)
            blh = xyz2llh(avp(k,7:9));
            CnG = pos2cng(blh);
            avp(k,1:9) = [m2att(CnG*a2mat(avp(k,1:3))); CnG*avp(k,4:6)'; blh]';
        end
        if k==1, avp=avp(1,1:9)'; end
        return;
    end
    B = avp(:,7); L = avp(:,8); H = avp(:,9);
    sB = sin(B); cB = cos(B); sL = sin(L); cL = cos(L);
    N = glv.Re./sqrt(1-glv.e2*sB.^2);
    X = (N+H).*cB.*cL;    Y = (N+H).*cB.*sL;    Z = (N*(1-glv.e2)+H).*sB;
    pe = [X, Y, Z];
    Cen = [ -sL,  -sB.*cL,  cB.*cL,...
             cL,  -sB.*sL,  cB.*sL,...
             0*B,  cB,      sB ];
    gs = cB.*sL; gc = cB.*cL;  zeta = 1./sqrt(1-gs.*gs);
    CeG = [ -zeta.*gs.*gc,  -zeta.*sB,   gc, ...
             1./zeta,        0*B,        gs, ...
            -zeta.*gs.*sB,   zeta.*gc,   sB ];
    CnG = [ Cen(:,1).*CeG(:,1)+Cen(:,4).*CeG(:,4)+Cen(:,7).*CeG(:,7), ...
            Cen(:,1).*CeG(:,2)+Cen(:,4).*CeG(:,5)+Cen(:,7).*CeG(:,8), ...
            Cen(:,1).*CeG(:,3)+Cen(:,4).*CeG(:,6)+Cen(:,7).*CeG(:,9), ...
            Cen(:,2).*CeG(:,1)+Cen(:,5).*CeG(:,4)+Cen(:,8).*CeG(:,7), ...
            Cen(:,2).*CeG(:,2)+Cen(:,5).*CeG(:,5)+Cen(:,8).*CeG(:,8), ...
            Cen(:,2).*CeG(:,3)+Cen(:,5).*CeG(:,6)+Cen(:,8).*CeG(:,9), ...
            Cen(:,3).*CeG(:,1)+Cen(:,6).*CeG(:,4)+Cen(:,9).*CeG(:,7), ...
            Cen(:,3).*CeG(:,2)+Cen(:,6).*CeG(:,5)+Cen(:,9).*CeG(:,8), ...
            Cen(:,3).*CeG(:,3)+Cen(:,6).*CeG(:,6)+Cen(:,9).*CeG(:,9) ];    
    vG = [ CnG(:,1).*avp(:,4)+CnG(:,4).*avp(:,5)+CnG(:,7).*avp(:,6),...
           CnG(:,2).*avp(:,4)+CnG(:,5).*avp(:,5)+CnG(:,8).*avp(:,6),...
           CnG(:,3).*avp(:,4)+CnG(:,6).*avp(:,5)+CnG(:,9).*avp(:,6) ];
    Cnb = a2matBatch(avp(:,1:3));
    CGb = [ CnG(:,1).*Cnb(:,1)+CnG(:,4).*Cnb(:,4)+CnG(:,7).*Cnb(:,7), ...
            CnG(:,1).*Cnb(:,2)+CnG(:,4).*Cnb(:,5)+CnG(:,7).*Cnb(:,8), ...
            CnG(:,1).*Cnb(:,3)+CnG(:,4).*Cnb(:,6)+CnG(:,7).*Cnb(:,9), ...
            CnG(:,2).*Cnb(:,1)+CnG(:,5).*Cnb(:,4)+CnG(:,8).*Cnb(:,7), ...
            CnG(:,2).*Cnb(:,2)+CnG(:,5).*Cnb(:,5)+CnG(:,8).*Cnb(:,8), ...
            CnG(:,2).*Cnb(:,3)+CnG(:,5).*Cnb(:,6)+CnG(:,8).*Cnb(:,9), ...
            CnG(:,3).*Cnb(:,1)+CnG(:,6).*Cnb(:,4)+CnG(:,9).*Cnb(:,7), ...
            CnG(:,3).*Cnb(:,2)+CnG(:,6).*Cnb(:,5)+CnG(:,9).*Cnb(:,8), ...
            CnG(:,3).*Cnb(:,3)+CnG(:,6).*Cnb(:,6)+CnG(:,9).*Cnb(:,9) ];
    attG = q2attBatch(m2quaBatch(CGb));
    avp(:,1:9) = [attG, vG, pe];
