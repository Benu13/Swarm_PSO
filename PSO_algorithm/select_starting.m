function [out_coord] = select_starting(start_X_bound,start_Y_bound, numberOfPoints, minAllowableDistance)

x = randi(start_X_bound, 10000, 1);
y = randi(start_Y_bound, 10000, 1);

% Initialize first point.
keeperX = x(1);
keeperY = y(1);

counter = 2;
for k = 2 : numberOfPoints
  % Get a trial point.
  thisX = x(k);
  thisY = y(k);
  % Check distance
  distances = sqrt((thisX-keeperX).^2 + (thisY - keeperY).^2);
  minDistance = min(distances);
  if minDistance >= minAllowableDistance
    keeperX(counter) = thisX;
    keeperY(counter) = thisY;
    counter = counter + 1;
  end
end

out_coord = [keeperX', keeperY'];

end

