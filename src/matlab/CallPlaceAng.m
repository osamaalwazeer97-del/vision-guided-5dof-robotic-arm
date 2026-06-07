function placeAng = CallPlaceAng(objtype)
% CallPlaceAng Return the joint command for the storage bin of each object.
%
% Command vector format:
%   [q1 q2 q3 q4 q5 gripper enablePower resetInitial]
%
% IMPORTANT:
% The values below are placeholders extracted from Appendix A. In the PDF,
% all bins used the same placeholder vector. Replace each case with your real
% calibrated bin joint angles before running the physical robot.

Gclose = 125;
Gopen  = 160;

switch objtype
    case 1  % CR: circular red
        placeAng = [0 0 0 0 0 Gclose 1 0];
    case 2  % CG: circular green
        placeAng = [0 0 0 0 0 Gclose 1 0];
    case 3  % CB: circular blue
        placeAng = [0 0 0 0 0 Gclose 1 0];
    case 4  % RR: rectangular red
        placeAng = [0 0 0 0 0 Gclose 1 0];
    case 5  % RG: rectangular green
        placeAng = [0 0 0 0 0 Gclose 1 0];
    case 6  % RB: rectangular blue
        placeAng = [0 0 0 0 0 Gclose 1 0];
    case 7  % TR: triangular red - TODO calibrate
        placeAng = [0 0 0 0 0 Gclose 1 0];
    case 8  % TG: triangular green - TODO calibrate
        placeAng = [0 0 0 0 0 Gclose 1 0];
    case 9  % TB: triangular blue - TODO calibrate
        placeAng = [0 0 0 0 0 Gclose 1 0];
    otherwise
        placeAng = [0 0 0 0 0 Gopen 1 0];
end
end
