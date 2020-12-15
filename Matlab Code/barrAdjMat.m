function [adjMat] = barrAdjMat(loc,adjMat,row,col,value)

%% º∆À„¡⁄Ω”æÿ’Û %%
for j = 1:size(loc,1)
    x = loc(j,1);
    y = loc(j,2);
    i1 = (y - 1)*row + x;
    if x + 1 <= col
        i2 = (y - 1)*row + x+1;
        adjMat(i1,i2) = value;
        adjMat(i2,i1) = value;
    end
    if x - 1 >= 1
        i2 = (y - 1)*row + x-1;
        adjMat(i1,i2) = value;
        adjMat(i2,i1) =value;
    end
    if y+1 <= row
        i2 = (y+1 - 1)*row + x;
        adjMat(i1,i2) = value;
        adjMat(i2,i1) = value;    
    end
    
    if y-1 >= 1
        i2 = (y-1 - 1)*row + x;
        adjMat(i1,i2) = inf;
        adjMat(i2,i1) = inf;    
    end
end

