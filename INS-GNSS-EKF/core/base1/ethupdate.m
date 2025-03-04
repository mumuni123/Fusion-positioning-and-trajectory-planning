function eth = ethupdate(eth, pos, vn)
% Update the Earth related parameters, much faster than 'earth'.
%
% Prototype: eth = ethupdate(eth, pos, vn)
% Inputs: eth - input earth structure array
%         pos - geographic position [lat;lon;hgt]
%         vn - velocity
% Outputs: eth - parameter structure array
%
% See also  ethinit, earth.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/05/2014
global glv
    if nargin==2,  vn = [0; 0; 0];  end
    eth.pos = pos;  eth.vn = vn;
    eth.sl = sin(pos(1));  eth.cl = cos(pos(1));
    if abs(eth.cl)<1e-7, eth.cl=1e-7; end  % polar, cos(pi/2-0.001/glv.Re)=1.57e-10, 1/12/2024
    eth.tl = eth.sl/eth.cl; 
    eth.sl2 = eth.sl*eth.sl;  sl4 = eth.sl2*eth.sl2;
    sq = 1-eth.e2*eth.sl2;  RN = eth.Re/sqrt(sq); 
    eth.RNh = RN+pos(3);  eth.clRNh = eth.cl*eth.RNh;
    eth.RMh = RN*(1-eth.e2)/sq+pos(3);
%     eth.wnie = [0; eth.wie*eth.cl; eth.wie*eth.sl];
    eth.wnie(2) = eth.wie*eth.cl; eth.wnie(3) = eth.wie*eth.sl;
%     eth.wnen = [-vn(2)/eth.RMh; vn(1)/eth.RNh; vn(1)/eth.RNh*eth.tl];
    eth.wnen(1) = -vn(2)/eth.RMh; eth.wnen(2) = vn(1)/eth.RNh; eth.wnen(3) = eth.wnen(2)*eth.tl;
%     eth.wnin = eth.wnie + eth.wnen;
    eth.wnin(1) = eth.wnie(1) + eth.wnen(1); eth.wnin(2) = eth.wnie(2) + eth.wnen(2); eth.wnin(3) = eth.wnie(3) + eth.wnen(3); 
%     eth.wnien = eth.wnie + eth.wnin;
    eth.wnien(1) = eth.wnie(1) + eth.wnin(1); eth.wnien(2) = eth.wnie(2) + eth.wnin(2); eth.wnien(3) = eth.wnie(3) + eth.wnin(3);
    eth.g = glv.g0*(1+5.2790414e-3*eth.sl2+2.32718e-5*sl4)-3.086e-6*pos(3);  % GJB6304-2008,Eq.(B.5)
%     gL = glv.g0*(1+glv.beta*eth.sl2-glv.beta1*(2*eth.sl*eth.cl)^2);  hR = pos(3)/(glv.Re*(1-glv.f*eth.sl2));
%     eth.g = gL*(1-2*hR-5*hR^2);
    eth.gn(3) = -eth.g;
    if ~isempty(glv.gfix), eth.gn(3)=-glv.gfix; end
%     eth.gcc = eth.gn - cros(eth.wnien,vn); % Gravitational/Coriolis/Centripetal acceleration
%     eth.gcc =  [ eth.wnien(3)*vn(2)-eth.wnien(2)*vn(3);  % faster than previous line
%                  eth.wnien(1)*vn(3)-eth.wnien(3)*vn(1);
%                  eth.wnien(2)*vn(1)-eth.wnien(1)*vn(2)+eth.gn(3) ];
    eth.gcc(1) = eth.wnien(3)*vn(2)-eth.wnien(2)*vn(3);
    eth.gcc(2) = eth.wnien(1)*vn(3)-eth.wnien(3)*vn(1);
    eth.gcc(3) = eth.wnien(2)*vn(1)-eth.wnien(1)*vn(2)+eth.gn(3);
    if isfield(eth, 'dgn')
        while eth.dgnt>eth.dgn(eth.dgnk,end) && eth.dgnk<eth.dgnlen, eth.dgnk=eth.dgnk+1; end
        eth.gcc = eth.gcc + eth.dgn(eth.dgnk,1:3)';
        eth.gn = eth.gn + eth.dgn(eth.dgnk,1:3)';
    end
