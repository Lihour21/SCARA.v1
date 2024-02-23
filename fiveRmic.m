function [ rmic, ymic, xmiw, ymiw, Xmiwgrid, Ymiwgrid, Flaggrid ] = ...
    fiveRmic( r1, r2, r3, n)
% fiveRmic Greatest circle inscribed in the useful workspace of a
% symmetrical parallel mechanism 5R . 
% 
% (fiveRmic Maximal Inscribed Circle within the usable workspace of a 5R
% symmetrical parallel mechanism)
%
% Detailed graphic decription in:
%   X-J. Liu, J. Wang, G. Pristschow. Kinematics, singularity and workspace
%   analysis of planar 5R symmetrical parallel mechanisms. Mechanism and
%   Machine Theory, 41(2):145-169, 2006.
% Entry
%   r1, normalized input link length
%   r2, normalized length of the intermediate link
%   r3, normalized length from origin to fixed pairs (only in
%       x, symmetrical). 
%       Normalization: D = (R1 + R2 + R3)/3; r_i = R_i/D, i = 1, 2, 3. 
% Optional inputs
%   n, number of points for discretization of the workspace
% Outputs
%   rmic, MIC radius
%   ymic, position of the center of the  maximal inscribed circle MIC (it is centered on x)
%   xmiw, vector with x positions equally spaced within the MIC
%   ymiw, vector with y positions equally spaced within the MIC
%   Xmiwgrid, mesh (matrix) with points X. n x n. Includes points on the outside
% of MIC.
%   Ymiwgrid, mesh (matrix) with Y points. n x n. Includes points on the outside
% of MIC.
%   Flaggrid, array with record of interior (1) and exterior positions
% (0) to MIC
%
% S.Lihour
%
% AUTOBOTx Lab, AI FARM Robotics Factory
% 2024

% Output assignment in the case of 3 input arguments
if nargin == 3
    xmiw = [ ]; ymiw = [ ]; Xmiwgrid  = [ ]; Ymiwgrid = [ ]; Flaggrid = [ ];
end
% Determination of the characteristics of the circle
if r1 <= 0 || r2 <= 0 || r3 <= 0 || r1 >= 3 || r2 >= 3 ...
        || r3 >= 1.5 || r1 + r2 + r3 > 3 + 1e-10 || r3 > r1 + r2 
    rmic = nan; % unnormalized mechanism, assign empty output []
    ymic = [ ];
    if nargin == 4
        xmiw = [ ]; ymiw = [ ]; Xmiwgrid  = [ ]; Ymiwgrid = [ ]; Flaggrid = [ ];
    end
elseif r1 + r3 < r2 % subregions IIIa y IVa
    rmic = ( r1 + r2 - abs( r1 - r2 ))/2;
    ymic = ((( r1 + r2 + abs( r1 - r2 ))^2)/4 - r3^2)^(1/2);
    if nargin == 4
        [ Xmiwgrid, Ymiwgrid, Flaggrid, xmiw, ymiw ] = ...
            discreteMIC( rmic, ymic, n );
    end
elseif r1 + r3 >= r2 % Other regions
    ycol = ( r1^2 - ( r2 - r3 )^2)^(1/2);
    ymic = (( r1 + r2 + ycol)^2 - r3^2)/(2*( r1 + r2 + ycol ));
    rmic = abs(ymic) - ycol;
    % verification for interior limits of the work space
    rmic2 = ( r1 + r2 - abs( r2 - r1 ))/2;
    if rmic2 < rmic
        rmic = rmic2;
        ymic = (( abs( r2 - r1 ) + rmic )^2 - r3^2 )^(1/2);
    else
        ymic = (( r1 + r2 + ycol)^2 - r3^2)/(2*( r1 + r2 + ycol ));
    end
    if nargin == 4
        [ Xmiwgrid, Ymiwgrid, Flaggrid, xmiw, ymiw ] = ...
            discreteMIC( rmic, ymic, n ); 
    end
end
% Discretization of the MIC. Three matrices and two vectors are obtained.
% Two arrays contain the mesh x - and the third has an indicator for
% the points that are in the circle. The vectors are constructed from
% of the mesh contain only the points on the circle
    function [ Xmiwgrid, Ymiwgrid, Flaggrid, xmiw, ymiw ] = ...
            discreteMIC( rmic, ymic, n )
        Xmiwgrid = linspace( -rmic, rmic, n );
        Ymiwgrid = linspace( ymic - rmic, ymic + rmic, n );
        [ Xmiwgrid, Ymiwgrid ] = meshgrid( Xmiwgrid, Ymiwgrid );
        Flaggrid = zeros( n, n );
        k3 = 0;
        for k2 = 1:n
            for k1 = 1:n
                r = ( Xmiwgrid( k2, k1 )^2 + ...
                    ( Ymiwgrid( k2, k1 ) - ymic )^2)^(1/2);
                if r <= rmic % inner point
                    k3 = k3 + 1;
                    Flaggrid( k2, k1 ) = 1;
                    xmiw(k3) = Xmiwgrid( k2, k1 );
                    ymiw(k3) = Ymiwgrid( k2, k1 );
                end
            end
        end
        if k3 == 0
            xmiw(k3) = [ ];
            ymiw(k3) = [ ];
        end
    end
end