function [xRandom, yRandom] = random_on_circle(xCenter, yCenter, radius, numPoints)
    angles = linspace(0, 2*pi, 720); % 720 is the total number of points
    x = radius * cos(angles) + xCenter;
    y = radius * sin(angles) + yCenter;
    % Now get random locations along the circle.
    randomIndexes = randperm(length(angles), numPoints);
    xRandom = x(randomIndexes);
    yRandom = y(randomIndexes);
end