%matlab初始化

clc;             %清除命令窗口的内容
clear all;       %清除工作空间的所有变量，函数，和MEX文件
close all;       %关闭所有的figure窗口

n=20;    %三维地图的边长
[field, startposind, goalposind, costchart, fieldpointers] =maps(n); %随机生成包含障碍物，起始点，终止点等信息的矩阵
%% 开始进行路径规划
 
% 路径规划中用到的一些矩阵的初始化
setOpen = [startposind]; setOpenCosts = [0]; setOpenHeuristics = [Inf];
setClosed = []; setClosedCosts = [];
movementdirections = {'R','L','B','F','D','U','DB','DF','UB','UF','RB','RF','LB','LF','DR','DL','UR','UL'};  %父节点移动方向右、左、后、前、下、上，下后、下前、上后、上前，右后、右前、左后、左前，下右、下左、上右、上左
 
tic
% 这个while循环是本程序的核心，利用循环进行迭代来寻找终止点
while ~max(ismember(setOpen,goalposind)) && ~isempty(setOpen)
    [temp, ii] = min(setOpenCosts + setOpenHeuristics);     %寻找拓展出来的最小值 
    Corner_obstacles = true;
    
    %这个函数的作用就是把输入的点作为父节点，然后进行拓展找到子节点，并且找到子节点的代价，并且把子节点距离终点的代价找到
    [costs,heuristics,posinds] = findFValue(Corner_obstacles,n,setOpen(ii),setOpenCosts(ii), field,goalposind);
 
  setClosed = [setClosed; setOpen(ii)];     % 将找出来的拓展出来的点中代价最小的那个点串到矩阵setClosed 中 
  setClosedCosts = [setClosedCosts; setOpenCosts(ii)];    % 将拓展出来的点中代价最小的那个点的代价串到矩阵setClosedCosts 中
  
  % 从setOpen中删除刚才放到矩阵setClosed中的那个点
  %如果这个点位于矩阵的内部
  if (ii > 1 && ii < length(setOpen))
    setOpen = [setOpen(1:ii-1); setOpen(ii+1:end)];
    setOpenCosts = [setOpenCosts(1:ii-1); setOpenCosts(ii+1:end)];
    setOpenHeuristics = [setOpenHeuristics(1:ii-1); setOpenHeuristics(ii+1:end)];
    
  %如果这个点位于矩阵第一行
  elseif (ii == 1)
    setOpen = setOpen(2:end);
    setOpenCosts = setOpenCosts(2:end);
    setOpenHeuristics = setOpenHeuristics(2:end);
    
  %如果这个点位于矩阵的最后一行
  else
    setOpen = setOpen(1:end-1);
    setOpenCosts = setOpenCosts(1:end-1);
    setOpenHeuristics = setOpenHeuristics(1:end-1);
  end
    
  % 把拓展出来的点中符合要求的点放到setOpen 矩阵中，作为待选点
  for jj=1:length(posinds)
  
    if ~isinf(costs(jj))   % 判断该点（方格）处没有障碍物
        
      % 判断一下该点是否 已经存在于setOpen 矩阵或者setClosed 矩阵中
      % 如果我们要处理的拓展点既不在setOpen 矩阵，也不在setClosed 矩阵中
      if ~max([setClosed; setOpen] == posinds(jj))
        fieldpointers(posinds(jj)) = movementdirections(jj);
        costchart(posinds(jj)) = costs(jj);
        setOpen = [setOpen; posinds(jj)];
        setOpenCosts = [setOpenCosts; costs(jj)];
        setOpenHeuristics = [setOpenHeuristics; heuristics(jj)];
        
      % 如果我们要处理的拓展点已经在setOpen 矩阵中
      elseif max(setOpen == posinds(jj))
        I = find(setOpen == posinds(jj));
        % 如果通过目前的方法找到的这个点，比之前的方法好（代价小）就更新这个点
        if setOpenCosts(I) > costs(jj)
          costchart(setOpen(I)) = costs(jj);
          setOpenCosts(I) = costs(jj);
          setOpenHeuristics(I) = heuristics(jj);
          fieldpointers(setOpen(I)) = movementdirections(jj);
        end
        
        % 如果我们要处理的拓展点已经在setClosed 矩阵中
      else
        I = find(setClosed == posinds(jj));
        % 如果通过目前的方法找到的这个点，比之前的方法好（代价小）就更新这个点
        if setClosedCosts(I) > costs(jj)
          costchart(setClosed(I)) = costs(jj);
          setClosedCosts(I) = costs(jj);
          fieldpointers(setClosed(I)) = movementdirections(jj);
        end
      end
    end
  end 
  
  if isempty(setOpen) break; end
 
  
  drawnow; 
end
 p = findWayBack(n, goalposind, fieldpointers);
  plot3(p(:, 1)-0.5, p(:, 2)-0.5, p(:, 3)-0.5, 'Color', [0.2, 0.2, 0.2], 'LineWidth', 2);