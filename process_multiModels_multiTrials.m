%-------------------------------------------------------------------------%
% Copyright (c) 2022 Koller W.                                            %
%    Author:   Willi Koller,  2022                                        %
%    email:    willi.koller@univie.ac.at                                  %
% ----------------------------------------------------------------------- %

% ProcessMultiModelsMultiTrials (c) by Willi Koller, University of Vienna
%
% ProcessMultiModelsMultiTrials is licensed under a
% Creative Commons Attribution-NonCommercial 4.0 International License.
%
% You should have received a copy of the license along with this
% work. If not, see <http://creativecommons.org/licenses/by-nc/4.0/>.

clear;
import org.opensim.modeling.*;


% setupIKfile = fullfile(pwd, 'SetupFiles', 'setupIK_CP.xml');
% setupIK_reducedTorso_file = fullfile(pwd, 'SetupFiles', 'setupIK_CP_reducedTorso.xml');
setupIKfile = fullfile(pwd, 'SetupFiles', 'Settings_IK_plugInGait.xml');
setupIK_reducedTorso_file = []; %fullfile(pwd, 'SetupFiles', 'Settings_IK_plugInGait.xml');
setupIDfile = fullfile(pwd, 'SetupFiles', 'Settings_ID.xml');
setupSOfile = fullfile(pwd, 'SetupFiles', 'Settings_SO.xml');
setupJRLfile = fullfile(pwd, 'SetupFiles', 'Settings_JRL.xml');
% setupJRLfile = fullfile(pwd, 'SetupFiles', 'Settings_JRL_inFemurL_with_MuscleDirection_BodyKinematics.xml');
% setupJRL2file = fullfile(pwd, 'SetupFiles', 'Settings_JRL_inFemurR_with_MuscleDirection_BodyKinematics.xml');

%% Options for steps which should be run
b_runIK = 1;
b_checkMuscleMomentArms = 0; % there is a better script to check for discontinuities in muscle moment arms - https://doi.org/10.1016/j.gaitpost.2025.01.063
% best is to run only IK in the first step;
% then, use the other script to check and resolve muscle discontinuities;
% then, run ID / SO / JRL 
b_skipTrialIfMomentArmsWrong = 0;
b_runID = 1;
b_runSO = 1;
b_runJRL = 1;

% enable logging to command window
Logger.setLevelString('Info');
Logger.addSink(JavaLogSink());

%%
disp('Select the root output folder');
rootOutput = uigetdir(pwd, 'Select the root output folder');
if isequal(rootOutput, 0)
    return;
end
%%
disp('Select models to run the pipeline');
[modelsFileNames, scaledModelsFolder] = uigetfile('*.osim', 'Multiselect', 'on', 'Select models to run the pipeline');
if isequal(modelsFileNames, 0)
    return;
end
if(~iscell(modelsFileNames))
    modelsFileNames = cellstr(modelsFileNames);
end
%%
disp('Select trials to run the pipeline. Files must have been processed with a_process_C3D to create corresponding folder!');
[trialsFileNames, trialsFolder] = uigetfile('*.c3d', 'Multiselect', 'on', 'Select trials to run the pipeline. Files must have been processed with a_process_C3D to create corresponding folder!');
if isequal(trialsFileNames, 0)
    return;
end
if(~iscell(trialsFileNames))
    trialsFileNames = cellstr(trialsFileNames);
end

%% run simulations
import org.opensim.modeling.*;

for trialIndex = 1 : size(trialsFileNames, 2)
    
    [folderTemp, trialNameNoExt, extension] = fileparts(fullfile(trialsFolder, char(trialsFileNames(trialIndex))));
    
    folder = fullfile(folderTemp, trialNameNoExt);
    trialName = trialNameNoExt;
       
    c3dFile = fullfile(folder, 'c3dfile.c3d');
    trcFile = fullfile(folder, 'marker_experimental.trc');
    grfSetupFile = fullfile(folder, 'GRF.xml');
    load(fullfile(folder, 'settings.mat'));
    
    if isfield(cycle, 'left') && isfield(cycle, 'right') % 
        startTime = double(min(cycle.left.start(1), cycle.right.start(1))) / frequency;
        endTime = double(max(cycle.left.end(end), cycle.right.end(end))) / frequency;
    elseif isfield(cycle, 'left')
        startTime = double(cycle.left.start(1)) / frequency;
        endTime = double(cycle.left.end(end)) / frequency;
    elseif isfield(cycle, 'right')
        startTime = double(cycle.right.start(1)) / frequency;
        endTime = double(cycle.right.end(end)) / frequency;
    else
        startTime = 0;
        endTime = duration;
    end

    
    preframes = startTime * frequency;

    % the simulation starts 0.1 seconds prior to first step - this can
    % cause errors at static optimization for some muscle types -->
    % try to start simulation earlier (increase preframes)
    if preframes > 0.1 * frequency
        preframes = 0.1 * frequency;
    end

    startTime = startTime - preframes / frequency;
                
    for modelIndex = 1 : size(modelsFileNames, 2)
        try
            fprintf(2,['Processing model "' modelsFileNames{modelIndex} '" and trial "' trialsFileNames{trialIndex} '"\n']);
            [tempFolder, modelFileNameNoExt, extension] = fileparts(fullfile(scaledModelsFolder, char(modelsFileNames(modelIndex))));
            
            outputPath = fullfile(rootOutput, modelFileNameNoExt, trialName, 'Output');
            if ~exist(outputPath, 'dir')
                mkdir(outputPath)
            end
            
            modelFile = fullfile(scaledModelsFolder, char(modelsFileNames(modelIndex)));
            actuatorfile = fullfile(scaledModelsFolder, strcat(modelFileNameNoExt, '_actuators.xml'));
            motionFile = fullfile(outputPath, 'IK', 'IK.mot');
            
            osimModel = Model(modelFile);
            state = osimModel.initSystem();
            model_mass = osimModel.getTotalMass(state);
            save(fullfile(rootOutput, modelFileNameNoExt, trialName, 'settings.mat'), 'cycle', 'firstFrame', 'frequency', 'duration', 'model_mass', 'preframes', '-mat');
            
            
            if(b_runIK == 1)
                disp('Running Inverse Kinematic ... ');
                
                % first run with reduced markers for torso to get
                % interesting marker errors for lower legs

                if ~isempty(setupIK_reducedTorso_file)
                    disp('without torso ... ');
                    resultsAreValid = runIK(setupIK_reducedTorso_file, modelFile, trcFile, startTime, endTime, fullfile(outputPath, 'IK_reducedTorsoMarkers'), 1, frequency);
                    if(~resultsAreValid)
                        fprintf(2,'Maker Error too big for inverse Kinematics!\n');

                        fileID = fopen(fullfile(rootOutput, modelFileNameNoExt, trialName, 'IK_fehler.txt'),'w');
                        fprintf(fileID,'Marker errors higher than recommended! Check again!\n');
                        fclose(fileID);
                    end

                    % now run the same with more torso markers
                    % This overwrites the output from the previous run
                    disp('and now with torso ... ');
                end
                resultsAreValid = runIK(setupIKfile, modelFile, trcFile, startTime, endTime, fullfile(outputPath, 'IK'), 0, frequency);
                if(~resultsAreValid)
                    fileID = fopen(fullfile(rootOutput, modelFileNameNoExt, trialName, 'IK_withTorsoOutOfRecommendation.txt'),'w');
                    fprintf(fileID,'Marker errors with all torso markers higher than recommended! No problem if marker errors without torso are ok.');
                    fclose(fileID);
                end
            end

            if b_checkMuscleMomentArms
                disp('Checking muscle moment arms ... ');
                momentArmsAreWrong = checkMuscleMomentArms(modelFile, motionFile);
                if momentArmsAreWrong
                    fileID = fopen(fullfile(rootOutput, modelFileNameNoExt, trialName, 'MuscleMomentArmsAreWrong.txt'),'w');
                    fprintf(fileID,'Muscle Moment Arms are wrong! Alter the model and check again!\n');
                    fclose(fileID);
                    if b_skipTrialIfMomentArmsWrong
                        break;
                    end
                end
                
            end
            
            if(b_runID == 1)
                disp('Running Inverse Dynamics ... ');
                runID(setupIDfile, modelFile, motionFile, grfSetupFile, startTime, endTime, fullfile(outputPath, 'ID'));
            end
                        
            if(b_runSO == 1)
                disp('Running Static Optimization - this might take a while...');
                runSO(setupSOfile, actuatorfile, modelFile, motionFile, grfSetupFile, startTime, endTime, fullfile(outputPath, 'SO'));
            end
            
            if(b_runJRL == 1)
                disp('Estimating Joint Contact Forces ... ');
                so_forcesFile = fullfile(outputPath, 'SO', '_StaticOptimization_force.sto');
                runJRL(setupJRLfile, actuatorfile, modelFile, motionFile, grfSetupFile, so_forcesFile, startTime, endTime, fullfile(outputPath, 'JRL'));
                if exist('setupJRL2file', 'var') && isfile(setupJRL2file)
                    copyfile(fullfile(outputPath, 'JRL', '_JointReaction_ReactionLoads.sto'), fullfile(outputPath, 'JRL', 'inFemurL_JointReaction_ReactionLoads.sto'));
                    runJRL(setupJRL2file, actuatorfile, modelFile, motionFile, grfSetupFile, so_forcesFile, startTime, endTime, fullfile(outputPath, 'JRL'));
                    copyfile(fullfile(outputPath, 'JRL', '_JointReaction_ReactionLoads.sto'), fullfile(outputPath, 'JRL', 'inFemurR_JointReaction_ReactionLoads.sto'));
                    delete(fullfile(outputPath, 'JRL', '_JointReaction_ReactionLoads.sto'));
                end
            end
            
            disp(['Finished model "' modelsFileNames{modelIndex} '" and trial "' trialsFileNames{trialIndex} '"']);
            disp(' ');
        catch e
            fileID = fopen(fullfile(rootOutput, modelFileNameNoExt, trialName, 'fehler.txt'),'w');
            fprintf(fileID,'Es ist ein Fehler aufgetreten!\nFehlermeldung:\n%s',e.message);
            fclose(fileID);
        end
    end
end