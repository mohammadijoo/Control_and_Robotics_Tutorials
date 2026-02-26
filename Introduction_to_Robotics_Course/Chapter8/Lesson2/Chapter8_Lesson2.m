% Checkerboard settings
patternSize = [9, 6];
squareSize = 25; % mm

% Detect corners
images = imageDatastore('calib_images');
[imagePoints, boardSize] = detectCheckerboardPoints(images.Files);

% Generate world points on plane Z=0
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Read one image to get size
I = readimage(images,1);
imageSize = [size(I,1), size(I,2)];

% Calibrate
params = estimateCameraParameters(imagePoints, worldPoints, 'ImageSize', imageSize);

% Results
K = params.IntrinsicMatrix';
dist = [params.RadialDistortion, params.TangentialDistortion];
rms = params.MeanReprojectionError;

disp(K);
disp(dist);
disp(rms);
