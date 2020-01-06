function [neighbors] = getNeighbors(rStruct, row, col)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

neighbors = [];

if  row-1 > 0 && rStruct.weight(row-1,col) ~= 0 && rStruct.isExplored(row-1, col) == false %row-1, col
    neighbors = [neighbors; row-1, col];
    
end    
if  row+1 <= size(rStruct.weight,1) && rStruct.weight(row+1,col) ~= 0 && rStruct.isExplored(row+1, col) == false %row+1, col
    neighbors = [neighbors; row+1, col];
   
end
if  col-1 > 0 && rStruct.weight(row,col-1) ~= 0 && rStruct.isExplored(row, col-1) == false %row, col-1
    neighbors = [neighbors; row, col-1];
    
end
if  col+1 <= size(rStruct.weight,2) && rStruct.weight(row,col+1) ~= 0 && rStruct.isExplored(row, col+1) == false %row, col+1
    neighbors = [neighbors; row, col+1];
end

end

