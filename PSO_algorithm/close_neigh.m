function G = close_neigh(x,L,fit_v)

%Inuput:
%   x- population
%   L - number of neighbors
%   fit_v - vecto of fittness for each point
%Output:
%   G - set of closest neighbours

G = zeros(size(x));

for i = 1:size(x,1)
    dist = zeros(size(x,1),1);
    
    for j = 1:size(x,1)
        if i == j
            dist(j) = inf;
        else
            dist(j) = norm(x(i,:)-x(j,:));
        end
    end
    
    [~, ii] = sort(dist);
    fit = fit_v(ii);
    P = x(ii,:);
    [~,ind] = min(fit(1:L));
    G(i,:) = P(ind,:);    
end

end

