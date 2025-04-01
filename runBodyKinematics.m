function runBodyKinematics(setupFile, actuatorfile, modelFile, motionFile, grfFile, so_forcesFile, startTime, endTime, outputPath)
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

analysisSet = jrlTool.getAnalysisSet();
body_kinematics = BodyKinematics.safeDownCast(analysisSet.get(0));
body_kinematics.setStartTime(startTime);
body_kinematics.setEndTime(endTime);

jrlTool.print(fullfile(outputPath, 'jrlSettings.xml'));

runTool = AnalyzeTool(fullfile(outputPath, 'jrlSettings.xml'));
runTool.run();

end

