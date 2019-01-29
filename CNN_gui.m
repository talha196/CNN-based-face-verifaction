function varargout = CNN_gui(varargin)
% CNN_GUI MATLAB code for CNN_gui.fig
%      CNN_GUI, by itself, creates a new CNN_GUI or raises the existing
%      singleton*.
%
%      H = CNN_GUI returns the handle to a new CNN_GUI or the handle to
%      the existing singleton*.
%
%      CNN_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CNN_GUI.M with the given input arguments.
%
%      CNN_GUI('Property','Value',...) creates a new CNN_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before CNN_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to CNN_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help CNN_gui

% Last Modified by GUIDE v2.5 10-Aug-2016 19:52:14

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @CNN_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @CNN_gui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before CNN_gui is made visible.
function CNN_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to CNN_gui (see VARARGIN)

% Choose default command line output for CNN_gui
handles.output = hObject;
axes(handles.tasweer_viewer);
axis equal off;
axes(handles.axes4);
axis equal off;
axes(handles.axes2);
axis equal off;

% Update handles structure
handles.net=load('data/vgg_face.mat');
guidata(hObject, handles);

% UIWAIT makes CNN_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = CNN_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function output_score_Callback(hObject, eventdata, handles)
% hObject    handle to output_score (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of output_score as text
%        str2double(get(hObject,'String')) returns contents of output_score as a double


% --- Executes during object creation, after setting all properties.
function output_score_CreateFcn(hObject, eventdata, handles)
% hObject    handle to output_score (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%  net=load('data/vgg_face.mat');
% --- Executes on button press in upl_img.
function upl_img_Callback(hObject, eventdata, handles)
% hObject    handle to upl_img (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[filename, pathname] = uigetfile(...    
    {'*.jpg; *.JPG; *.jpeg; *.JPEG; *.img; *.IMG; *.tif; *.TIF; *.tiff; *.png; *.TIFF','Supported Files (*.jpg,*.img,*.tiff,*.png)'; ...
    '*.jpg','jpg Files (*.jpg)';...
    '*.JPG','JPG Files (*.JPG)';...
    '*.jpeg','jpeg Files (*.jpeg)';...
    '*.JPEG','JPEG Files (*.JPEG)';...
    '*.img','img Files (*.img)';...
    '*.IMG','IMG Files (*.IMG)';...
    '*.tif','tif Files (*.tif)';...
    '*.TIF','TIF Files (*.TIF)';...
    '*.tiff','tiff Files (*.tiff)';...
    '*.TIFF','TIFF Files (*.TIFF)'},...    
    'MultiSelect', 'on');
if isequal(filename,0)
%    error(' Load Error: No files selected! Load cancelled.')
else

end

% launch the figure box

imagefilename=fullfile(pathname,filename);
im = imread(imagefilename);
axes(handles.tasweer_viewer);
imagesc(im)
axis equal off


%passing the image to the network
im_ = single(im) ; % note: 255 range
im_ = imresize(im_, handles.net.meta.normalization.imageSize(1:2));
im_ = bsxfun(@minus,im_,handles.net.meta.normalization.averageImage);
res = vl_simplenn(handles.net, im_);
scores = squeeze(gather(res(end).x));
[bestScore, best] = max(scores);
%   printing scores
set(handles.output_score, 'String', handles.net.meta.classes.description{best});
guidata(hObject,handles);

% net=load('data/vgg_face.mat');
% --- Executes on button press in cam_launch.
function cam_launch_Callback(hObject, eventdata, handles)

% hObject    handle to cam_launch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
faceDetector = vision.CascadeObjectDetector();

% Create the point tracker object.
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

% Create the webcam object.
cam = webcam(1);

% Capture one frame to get its size.
videoFrame = snapshot(cam);
% CNN_face(net,videoFrame);
frameSize = size(videoFrame);
% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

runLoop = true;
numPts = 0;
frameCount = 0;
J=0;
while runLoop

    % Get the next frame.
    videoFrame = snapshot(cam);
    videoFrameGray = rgb2gray(videoFrame);
    frameCount = frameCount + 1;
    if numPts < 10
        % Detection mode.
        bbox = faceDetector.step(videoFrameGray);
          for i = 1:size(bbox,1)
            J= imcrop(videoFrame,bbox(i,:));
           
          end
          axes(handles.tasweer_viewer);
          imagesc(J);
          axis equal off;
          % passing the image to the network
          im_ = single(J) ; % note: 255 range
          im_ = imresize(im_, handles.net.meta.normalization.imageSize(1:2));
          im_ = bsxfun(@minus,im_,handles.net.meta.normalization.averageImage);
          res = vl_simplenn(handles.net, im_);
          scores = squeeze(gather(res(end).x));
          [bestScore, best] = max(scores);
          %   printing scores
          set(handles.output_score, 'String', handles.net.meta.classes.description{best});
        if ~isempty(bbox)
            % Find corner points inside the detected region.
            points = detectMinEigenFeatures(videoFrameGray, 'ROI', bbox(1, :));

            % Re-initialize the point tracker.
            xyPoints = points.Location;
            numPts = size(xyPoints,1);
            release(pointTracker);
            initialize(pointTracker, xyPoints, videoFrameGray);

            % Save a copy of the points.
            oldPoints = xyPoints;

            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPoints = bbox2points(bbox(1, :));

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);

            % Display a bounding box around the detected face.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);

            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPoints, '+', 'Color', 'white');
        end

    else
        % Tracking mode.
        [xyPoints, isFound] = step(pointTracker, videoFrameGray);
        visiblePoints = xyPoints(isFound, :);
        oldInliers = oldPoints(isFound, :);

        numPts = size(visiblePoints, 1);

        if numPts >= 10
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform, oldInliers, visiblePoints] = estimateGeometricTransform(...
                oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);

            % Apply the transformation to the bounding box.
            bboxPoints = transformPointsForward(xform, bboxPoints);

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);

            % Display a bounding box around the face being tracked.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);
            
            % Display tracked points.
            videoFrame = insertMarker(videoFrame, visiblePoints, '+', 'Color', 'white');
            
            % Reset the points.
            oldPoints = visiblePoints;
            setPoints(pointTracker, oldPoints);
        end

    end

    % Display the annotated video frame using the video player object.
     step(videoPlayer, videoFrame);

    % Check whether the video player window has been closed.
     runLoop = isOpen(videoPlayer);
end

 clear cam;
 release(videoPlayer);
 release(pointTracker);
 release(faceDetector);


% --- Executes on button press in pair.
function pair_Callback(hObject, eventdata, handles)
% hObject    handle to pair (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[filename, pathname] = uigetfile(...    
    {'*.jpg; *.JPG; *.jpeg; *.JPEG; *.img; *.IMG; *.tif; *.TIF; *.tiff; *.png; *.TIFF','Supported Files (*.jpg,*.img,*.tiff,*.png)'; ...
    '*.jpg','jpg Files (*.jpg)';...
    '*.JPG','JPG Files (*.JPG)';...
    '*.jpeg','jpeg Files (*.jpeg)';...
    '*.JPEG','JPEG Files (*.JPEG)';...
    '*.img','img Files (*.img)';...
    '*.IMG','IMG Files (*.IMG)';...
    '*.tif','tif Files (*.tif)';...
    '*.TIF','TIF Files (*.TIF)';...
    '*.tiff','tiff Files (*.tiff)';...
    '*.TIFF','TIFF Files (*.TIFF)'},...    
    'MultiSelect', 'on');
if isequal(filename,0)
%    error(' Load Error: No files selected! Load cancelled.')
else

end

% launch the figure box

imagefilename=fullfile(pathname,filename);
im1 = imread(imagefilename);
axes(handles.axes2);
imagesc(im1)
axis equal off
[filename, pathname] = uigetfile(...    
    {'*.jpg; *.JPG; *.jpeg; *.JPEG; *.img; *.IMG; *.tif; *.TIF; *.tiff; *.png; *.TIFF','Supported Files (*.jpg,*.img,*.tiff,*.png)'; ...
    '*.jpg','jpg Files (*.jpg)';...
    '*.JPG','JPG Files (*.JPG)';...
    '*.jpeg','jpeg Files (*.jpeg)';...
    '*.JPEG','JPEG Files (*.JPEG)';...
    '*.img','img Files (*.img)';...
    '*.IMG','IMG Files (*.IMG)';...
    '*.tif','tif Files (*.tif)';...
    '*.TIF','TIF Files (*.TIF)';...
    '*.tiff','tiff Files (*.tiff)';...
    '*.TIFF','TIFF Files (*.TIFF)'},...    
    'MultiSelect', 'on');
if isequal(filename,0)
%    error(' Load Error: No files selected! Load cancelled.')
else

end

% launch the figure box

imagefilename=fullfile(pathname,filename);
im2 = imread(imagefilename);
axes(handles.axes4);
imagesc(im2)
axis equal off
c=Pair_matching(handles.net,im1,im2);
d=num2str(c);
if(c<=0.625)

    
    set(handles.match_result, 'String','Matches' );
    set(handles.PM_score, 'String',d );
  
   

else

        set(handles.match_result, 'String','Not matches' );
         set(handles.PM_score, 'String',d );
        
end
 guidata(hObject,handles);





function match_result_Callback(hObject, eventdata, handles)
% hObject    handle to match_result (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of match_result as text
%        str2double(get(hObject,'String')) returns contents of match_result as a double


% --- Executes during object creation, after setting all properties.
function match_result_CreateFcn(hObject, eventdata, handles)
% hObject    handle to match_result (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function PM_score_Callback(hObject, eventdata, handles)
% hObject    handle to PM_score (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PM_score as text
%        str2double(get(hObject,'String')) returns contents of PM_score as a double


% --- Executes during object creation, after setting all properties.
function PM_score_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PM_score (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
