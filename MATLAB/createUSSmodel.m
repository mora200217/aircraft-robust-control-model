function [USSmodel] = createUSSmodel(linmodel,Delta)
% Create a new SS model with 5% uncertainty for each element of A,B,C and D

% Get the dimensions of A,B,C and D
[n,m] = size(linmodel.B);
[p,n] = size(linmodel.C);

% Create a ureal object with 5% uncertainty

% Iterate over each element of A,B,C and D and add uncertainty
for i=1:n
    for j=1:n
        if linmodel.A(i,j)~=0
            A(i,j) = ureal(['A' num2str(i) num2str(j)],linmodel.A(i,j),'Percentage',Delta);
            else
            A(i,j)=0;
        end
    end
    for j=1:m
        if linmodel.B(i,j)~=0
            B(i,j) = ureal(['B' num2str(i) num2str(j)],linmodel.B(i,j),'Percentage',Delta);
            else
            B(i,j)=0;
        end
    end
end

for i=1:p
    for j=1:n
        if linmodel.C(i,j)~=0
            C(i,j) = ureal(['C' num2str(i) num2str(j)],linmodel.C(i,j),'Percentage',Delta);
            else
            C(i,j)=0;
        end
    end
    for j=1:m
        if linmodel.D(i,j)~=0
            D(i,j) = ureal(['D' num2str(i) num2str(j)],linmodel.D(i,j),'Percentage',Delta);
        else
            D(i,j)=0;
        end
    end
end

% Create a USS model
USSmodel = uss(ss(A,B,C,D));

end