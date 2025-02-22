function msplot(mnp, x, y, xstr, ystr)
% my sub plot
global msplot_mn
    if mnp>=10, msplot_mn=fix(mnp/10)*10;
    else,       mnp=msplot_mn+mnp;       end
    if nargin<3, y=x; x=1:length(y); xstr='k'; ystr='value'; end  % msplot(mnp, y)
    if mod(mnp,10)==1, myfigure; end   % ����ǵ�һ��Сͼ�����½�һ��figure
    subplot(mnp); plot(x, y, 'linewidth', 2); grid on;
    if nargin==4, ystr = xstr; xstr = '\itt\rm / s'; end  % ���ֻ����һ���ַ�������Ĭ��xlabelΪʱ��
    if exist('ystr', 'var'), xlabel(xstr); ylabel(ystr); end