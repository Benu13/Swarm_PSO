addpath('Test_logs')
load logg8.mat
% addpath('path')
% img = imread('mappa.png');
% img = imbinarize(img);
% img = img(:,:,1);
% img = imresize(img,[240 240]);
% dist_circ = 10;
% img = uint8(img);
% img(:,:,2)=img(:,:,1);
% img(:,:,3)=img(:,:,1);
% img = uint8(255 * mat2gray(img));

for i = 1:ff
    imshow(img,[],'InitialMagnification', 800);
    hold on
    for j = 1:size(movement,2)
        scatter(movement{1,j}(i,1),movement{1,j}(i,2))
        viscircles([movement{1,j}(i,1),movement{1,j}(i,2)],max_robot_speed);
%         for k=1:i
%             img(movement{1,j}(i,2),movement{1,j}(i,1),:) = [0,255,0];  
%         end
    end
    hold off
    pause(0.2)
end