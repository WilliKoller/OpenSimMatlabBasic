%%
clear;
dataPath = 'C:\Users\Willi\Documents\UniDataLokal\DEMAND\VICON_DATA_SCHMELZ\TD01\2022-11-11';
trialList = GetSubDirsFirstLevelOnly(dataPath);

walkingSpeedLeft = [];
walkingSpeedRight = [];
stepIndexLeft = 1, stepIndexRight = 1; 
%%

for trialNr = 1 : numel(trialList)
    disp(['currently processing:' trialList{trialNr}]);

    currentFolder = [dataPath filesep trialList{trialNr}];

    clear preframes EMG EMGChannels ratioEmgToFrames emgFrequency;
    load(fullfile(currentFolder, 'settings.mat'));

    cycleIsSorted = 0;
    if isfield(cycle, 'left') && isfield(cycle, 'right')
        if issorted(cycle.left.start) && issorted(cycle.right.start)
            cycleIsSorted = 1;
        end
    elseif isfield(cycle, 'left')
        if issorted(cycle.left.start)
            cycleIsSorted = 1;
        end
    else
        if issorted(cycle.right.start)
            cycleIsSorted = 1;
        end
    end
    if cycleIsSorted

        % jrlFile = fullfile(currentFolder, 'Output', 'JRL', '_JointReaction_ReactionLoads.sto');
        % if isfile(jrlFile)
            % 
            % fid = fopen(fullfile(currentFolder, 'Output', 'IK', 'ikSettings.xml'), 'r');
            % f=fread(fid,'*char')';
            % fclose(fid);
            % i0 = strfind(f, '<marker_file>');
            % i1 = strfind(f, '</marker_file>');
            % 
            % markerFileName = f(i0 + 13 : i1 - 1);

            markerData = load_marker_trc(fullfile(currentFolder, 'marker_experimental.trc'));
            markerNames = fieldnames(markerData);
            heelMarker = markerNames(contains(markerNames, 'HEE'));
            heelMarker = heelMarker{1}(1 : end-2);
            heelMarker = strrep(heelMarker, 'LH', 'H');
            heelMarker = strrep(heelMarker, 'RH', 'H');

            try
                leftHeel = [cell2mat(markerData.(['L' heelMarker '_X'])), cell2mat(markerData.(['L' heelMarker '_Y'])), cell2mat(markerData.(['L' heelMarker '_Z']))];
                rightHeel = [cell2mat(markerData.(['R' heelMarker '_X'])), cell2mat(markerData.(['R' heelMarker '_Y'])), cell2mat(markerData.(['R' heelMarker '_Z']))];

                if isfield(cycle, 'left')
                    for j = 1 : size(cycle.left.start, 2)
                        durationInSeconds = double(cycle.left.end(j) + 1 - cycle.left.start(j) + 1) / frequency;
                        distanceX = leftHeel(cycle.left.start(j) + 1, 1) - leftHeel(cycle.left.end(j) + 1, 1);
                        distanceY = leftHeel(cycle.left.start(j) + 1, 2) - leftHeel(cycle.left.end(j) + 1, 2);
                        distance = sqrt(distanceX.^2 + distanceY.^2) / 1000;
                        walkingSpeedLeft(stepIndexLeft) = distance/durationInSeconds;
                        stepIndexLeft = stepIndexLeft + 1;
                    end
                end

                if isfield(cycle, 'right')
                    for j = 1 : size(cycle.right.start, 2)
                        durationInSeconds = double(cycle.right.end(j) + 1 - cycle.right.start(j) + 1) / frequency;
                        distanceX = rightHeel(cycle.right.start(j) + 1, 1) - rightHeel(cycle.right.end(j) + 1, 1);
                        distanceY = rightHeel(cycle.right.start(j) + 1, 2) - rightHeel(cycle.right.end(j) + 1, 2);
                        distance = sqrt(distanceX.^2 + distanceY.^2) / 1000;
                        walkingSpeedRight(stepIndexRight) = distance/durationInSeconds;
                        stepIndexRight = stepIndexRight + 1;
                    end
                end
            end
        % end

    end
end

walkingSpeedRight(walkingSpeedRight == 0) = nan;
walkingSpeedLeft(walkingSpeedLeft == 0) = nan;

%%
P1 = [82.99307592	51.46501852	-175.0096358];
P2 = [80.64589233	35.83298901	178.8016599];

P = [-0.0137108 -0.735761 0; -0.013588 -0.732529 0]
