function [flag] = isSatisfyLimit(map,point,Loc)
x = point(1);
y = point(2);
% for i = 1:size(Loc,1)
%     if Loc(i,3) ~= 0 && Loc(i,4) ~= 0
%         map(Loc(i,4),Loc(i,3)) = 0;
%     else
%         break;
%     end
% end
        
flag = 1;
for i = [-1,0,1]
    for j = [-1,0,1]
        a = x+i;
        b = y+j;
        if a > 0 && a <= size(map,2) && b > 0 && b<= size(map,1)
            if ~map(y+j,x+i)
                flag = 0;
                break;
            end
        else
            flag = 0;
            break;
        end
    end
    if flag == 0
        break;
    end
end
end

