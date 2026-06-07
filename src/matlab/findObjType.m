function [objT, theLabel] = findObjType(SF, colorIndex)
% findObjType Classify object type using shape factor and color index.
%
% Inputs:
%   SF         - shape factor = perimeter^2/(4*pi*area)
%   colorIndex - 1 red, 2 green, 3 blue, otherwise unknown
%
% Outputs:
%   objT      - numeric object type
%   theLabel  - label, e.g., CR = Circular Red
%
% Original Appendix A used:
%   SF < 1.19  -> circle
%   SF < 1.53  -> rectangle
%   otherwise  -> unknown
%
% A triangle branch is added as a TODO-compatible extension because the report
% states that the system sorts circle, rectangle, and triangle objects.

if SF < 1.19
    shapePrefix = 'C';
    baseType = 0;
elseif SF < 1.53
    shapePrefix = 'R';
    baseType = 3;
else
    shapePrefix = 'T';
    baseType = 6;
end

switch colorIndex
    case 1
        colorSuffix = 'R';
        colorOffset = 1;
    case 2
        colorSuffix = 'G';
        colorOffset = 2;
    case 3
        colorSuffix = 'B';
        colorOffset = 3;
    otherwise
        colorSuffix = 'U';
        colorOffset = 0;
end

if colorOffset == 0
    objT = 0;
else
    objT = baseType + colorOffset;
end

theLabel = [shapePrefix colorSuffix];
end
