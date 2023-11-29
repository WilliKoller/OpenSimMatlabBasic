function runJRL(setupFile, actuatorfile, modelFile, motionFile, grfFile, so_forcesFile, startTime, endTime, outputPath)
%runJRL Summary of this function goes here
%   Detailed explanation goes here
import org.opensim.modeling.*;

if ~exist(outputPath, 'dir')
    mkdir(outputPath)
end

% load additional libraries if you need them
if contains(lower(setupFile), 'muscledirection')
    opensimCommon.LoadOpenSimLibrary('C:\OpenSim 4.2\plugins\MuscleForceDirection')
end

jrlTool = AnalyzeTool(setupFile, false);
jrlTool.setModelFilename(modelFile);
jrlTool.setResultsDir(outputPath);
jrlTool.setInitialTime(startTime);
jrlTool.setFinalTime(endTime);
jrlTool.setExternalLoadsFileName(grfFile);
jrlTool.setCoordinatesFileName(motionFile);
jrlTool.setLowpassCutoffFrequency(6);
actuatorFilesArray = ArrayStr();
actuatorFilesArray.append(actuatorfile);
jrlTool.setForceSetFiles(actuatorFilesArray);

analysisSet = jrlTool.getAnalysisSet();
joint_reaction = JointReaction.safeDownCast(analysisSet.get(0));
joint_reaction.setStartTime(startTime);
joint_reaction.setEndTime(endTime);
joint_reaction.setForcesFileName(so_forcesFile);

% specify options for other analysis. Should work for almost all Analysis
% similar to these lines
if contains(lower(setupFile), 'muscledirection')
    muscle_force_direction = analysisSet.get(1);
    muscle_force_direction.setStartTime(startTime);
    muscle_force_direction.setEndTime(endTime);
end

if contains(lower(setupFile), 'bodykinematics')
    body_kinematics = BodyKinematics.safeDownCast(analysisSet.get(2));
    body_kinematics.setStartTime(startTime);
    body_kinematics.setEndTime(endTime);
end

jrlTool.print(fullfile(outputPath, 'jrlSettings.xml'));

runTool = AnalyzeTool(fullfile(outputPath, 'jrlSettings.xml'));
runTool.run();

end

