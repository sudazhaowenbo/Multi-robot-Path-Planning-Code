function [adjMat] = computeAdjMat(col,row)

%% �����ڽӾ��� %%

adjMat = ones(row*col, row*col)*inf;
for i = 1:size(adjMat,1)
    if i <= col % �����±߽�
        if mod(i,col) == 1 % ������߽�
            adjMat(i,i+1) = 1;
            adjMat(i,i+col) = 1;
        elseif mod(i,col) == 0 % �����ұ߽�
            adjMat(i,i-1) = 1;
            adjMat(i,i+col) = 1;
        else
            adjMat(i,i+1) = 1;
            adjMat(i,i-1) = 1;
            adjMat(i,i+col) = 1; 
        end
    elseif i > size(adjMat,1) - col % �����ϱ߽�
        if mod(i,col) == 1 % ������߽�
            adjMat(i,i+1) = 1;            
        elseif mod(i,col) == 0 % �����ұ߽�
            adjMat(i,i-1) = 1;
        else
            adjMat(i,i+1) = 1;
            adjMat(i,i-1) = 1;
        end  
        adjMat(i,i-col) = 1;
    else
        if mod(i,col) == 1 % ������߽�
            adjMat(i,i+1) = 1;
        elseif mod(i,col) == 0 % �����ұ߽�
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

