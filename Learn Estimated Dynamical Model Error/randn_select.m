function output = randn_select(M,row_or_col,Num)
%RANDN_SELECT form a randn matrix
[R,C] = size(M);
if(strcmp(row_or_col,'row'))
    if Num > R
        disp('Exceed the total rows of matrix');
        output = [];
    else
        k=randperm(R,Num);
        for i=1:Num
            output(i,:) = M(k(i),:);   
        end
    end

elseif(strcmp(row_or_col,'col'))
    if Num > C
        disp('Exceed the total columns of matrix');
        output = [];
    else
        k=randperm(C,Num);
        for i=1:Num
            output(:,i) = M(:,k(i));   
        end
    end
end
end

