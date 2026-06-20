% RGB camera bring-up (requires Image Acquisition Toolbox)
cam = webcam(1);  % choose device index
for k = 1:300
    img = snapshot(cam);
    imshow(img); drawnow;
end
clear cam;
