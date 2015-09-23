function [ dist ] = heuristic( P1, P2 )
%Function to calculate error on given particle
%Heuristic is Euclidean distance between desired and attained
dist = sqrt((P2(1) - P1(1))^2 + (P2(2) - P1(2))^2);
end