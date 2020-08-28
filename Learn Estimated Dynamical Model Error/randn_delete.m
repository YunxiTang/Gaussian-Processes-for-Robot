function output = randn_delete(M,row_or_col)
%RANDN_DELETE delete on a colomn or row randomly from a matrix
    [R,C] = size(M);
    if(strcmp(row_or_col,'row'))
        k=randperm(R,1);
        M(k,:)=[];
    elseif(strcmp(row_or_col,'col'))
        k=randperm(C,1);
        M(:,k)=[];
    end
    output = M;
end

