I = im2double(imread('scene.png'));

% Edges (Sobel)
[Gx, Gy] = imgradientxy(I,'sobel');
G = hypot(Gx, Gy);
edges = G > 0.2;

% Corners (Harris)
corners = detectHarrisFeatures(I);

% Blobs (LoG)
blobs = detectSURFFeatures(I); % uses scale-space extrema idea

fprintf('corners=%d blobs=%d\n', corners.Count, blobs.Count);
