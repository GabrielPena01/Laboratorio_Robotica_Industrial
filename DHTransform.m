% In this part We're going to define our DH function
function Result = DHTransform(Theta, d, a, alpha)
    Result = [cosd(Theta) -cosd(alpha)*sind(Theta) sind(alpha)*sind(Theta) a*cosd(Theta);
              sind(Theta) cosd(alpha)*cosd(Theta) -sind(alpha)*cosd(Theta) a*sind(Theta);
              0 sind(alpha) cosd(alpha) d;
              0 0 0 1];
end