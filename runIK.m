function [resultsAreValid] = runIK(setupFile, modelFile, trcFile, startTime, endTime, outputPath, reportMarkerError, frequency)
% runIK Summary of this function goes here
%   Detailed explanation goes here
import org.opensim.modeling.*;

resultsAreValid = false;

if ~exist(outputPath, 'dir')
    mkdir(outputPath)
end

ikTool = InverseKinematicsTool(setupFile); 

osimModel = Model(modelFile);
ikTool.setModel(osimModel);
ikTool.setMarkerDataFileName(trcFile);
ikTool.setStartTime(startTime);
ikTool.setEndTime(endTime);
ikTool.setOutputMotionFileName(fullfile(outputPath, 'IK.mot'));
ikTool.print(fullfile(outputPath, 'ikSettings.xml'));
ikTool.run();

copyfile('_ik_marker_errors.sto', fullfile(outputPath, 'IK_marker_errors.sto'));
delete('_ik_marker_errors.sto');

if isfile('_ik_model_marker_locations.sto')
    copyfile('_ik_model_marker_locations.sto', fullfile(outputPath, 'IK_model_marker_locations.sto'));
    delete('_ik_model_marker_locations.sto');
end

% Check for errors!
% Add cycle to check!
%https://simtk-confluence.stanford.edu/display/OpenSim/Checklist+-+Evaluating+your+Simulation
% Is the maximum marker error less than 4 cm?
% Is the RMS error less than ~2 cm?

errors = load_sto_file(fullfile(outputPath, 'IK_marker_errors.sto'));
[maxMarkerError, maxMarkerIndex] = max(errors.marker_error_max);
[maxRMSError, maxRMSIndex] = max(errors.marker_error_RMS);
maxMarkerIndex = maxMarkerIndex - 1;
maxRMSIndex = maxRMSIndex - 1;
if (maxMarkerError <= 0.04 && maxRMSError <= 0.02)
    resultsAreValid = true;

    disp(['Max Marker Error: ' num2str(maxMarkerError) ' at time s= ' num2str(startTime + maxMarkerIndex / frequency)]);
    disp(['Max RMS Error: ' num2str(maxRMSError) ' at time s= ' num2str(startTime + maxRMSIndex / frequency)]);
else
    fprintf(2,['Max Marker Error: ' num2str(maxMarkerError) ' at time s= ' num2str(startTime + maxMarkerIndex / frequency) '\n']);
    fprintf(2,['Max RMS Error: ' num2str(maxRMSError) ' at time s= ' num2str(startTime + maxRMSIndex / frequency) '\n']);
    if reportMarkerError < 1
        fprintf(2,['Aber vermutlich egal ... \n']);
    end
end
if reportMarkerError >= 1
    fileID = fopen(fullfile(outputPath, 'marker_errors_summary.txt'),'w');
    fprintf(fileID,'Maximaler Marker Error: %.1f mm\n', maxMarkerError*1000);
    fprintf(fileID,'Maximaler Marker Error at time s= %0.3f\n', startTime + maxMarkerIndex / frequency);
    fprintf(fileID,'Maximaler RMS Error: %.1f mm\n', maxRMSError*1000);
    fprintf(fileID,'Maximaler RMS Error at time s= %0.3f\n', startTime + maxRMSIndex / frequency);
    fclose(fileID);
end
end

