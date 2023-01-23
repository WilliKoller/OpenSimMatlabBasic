function [] = runID(setupFile, modelFile, motionFile, grfFile, startTime, endTime, outputPath)
%runID Summary of this function goes here
%   Detailed explanation goes here
import org.opensim.modeling.*;

if ~exist(outputPath, 'dir')
    mkdir(outputPath)
end

idTool = InverseDynamicsTool(setupFile, false);
idTool.setExternalLoadsFileName(grfFile);
idTool.setModelFileName(modelFile);
idTool.setCoordinatesFileName(motionFile);
idTool.setLowpassCutoffFrequency(6);
idTool.setStartTime(startTime);
idTool.setEndTime(endTime);
idTool.setResultsDir(outputPath);
idTool.setOutputGenForceFileName('inverse_dynamics.sto');
idTool.print(fullfile(outputPath, 'idSettings.xml'));

idTool.run();

end

