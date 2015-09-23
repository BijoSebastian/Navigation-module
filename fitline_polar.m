function [ r,alpha] = fitline_polar( rho,theta )

N = size(theta, 2);
    
if (size(theta, 1) ~= 1) || (size(rho, 1) ~= 1)
    error('fitLinePolar only accepts column vectors');
end
if (size(rho, 2) ~= N)
    error('theta and rho must have matching size. But size(theta, 2) = %d, and size(rho, 2) = %d', size(theta, 2), size(rho, 2));
end

rhoSquare = rho .* rho;

cs = cos(theta);
cs2 = cos(2 * theta);

sn = sin(theta);
sn2 = sin(2 * theta);


thetaTemp = theta' * ones(1, N);
thetaDyadSum = thetaTemp + thetaTemp';
cosThetaDyadSum = cos(thetaDyadSum);

rhoDyad = rho'* rho;
csIJ = sum(sum(rhoDyad .* cosThetaDyadSum));

y = rhoSquare * sn2' - 2/N * rho * cs' * rho * sn';

x = rhoSquare * cs2' - csIJ / N;

alpha = 0.5 * (atan2(y, x) + pi);

r = rho * cos(theta - ones(size(theta)) * alpha)' / N;





