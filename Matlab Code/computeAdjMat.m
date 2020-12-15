function [adjMat] = computeAdjMat(col,row)

%% 计算邻接矩阵 %%

adjMat = ones(row*col, row*col)*inf;
for i = 1:size(adjMat,1)
    if i <= col % 处于下边界
        if mod(i,col) == 1 % 处于左边界
            adjMat(i,i+1) = 1;
            adjMat(i,i+col) = 1;
        elseif mod(i,col) == 0 % 处于右边界
            adjMat(i,i-1) = 1;
            adjMat(i,i+col) = 1;
        else
            adjMat(i,i+1) = 1;
            adjMat(i,i-1) = 1;
            adjMat(i,i+col) = 1; 
        end
    elseif i > size(adjMat,1) - col % 处于上边界
        if mod(i,col) == 1 % 处于左边界
            adjMat(i,i+1) = 1;            
        elseif mod(i,col) == 0 % 处于右边界
            adjMat(i,i-1) = 1;
        else
            adjMat(i,i+1) = 1;
            adjMat(i,i-1) = 1;
        end  
        adjMat(i,i-col) = 1;
    else
        if mod(i,col) == 1 % 处于左边界
            adjMat(i,i+1) = 1;
        elseif mod(i,col) == 0 % 处于右边界
            adjMat(i,i-1) = 1;
        else
            adjMat(i,i+1) = 1;
            adjMat(i,i-1) = 1;
        end  
        adjMat(i,i-col) = 1;
        adjMat(i,i+col) = 1;
    end
end
end

