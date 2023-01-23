function [resultsAreValid] = runIK(setupFile, modelFile, trcFile, startTime, endTime, outputPath, reportMarkerError)
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

copyfile('_ik_model_marker_locations.sto', fullfile(outputPath, 'IK_model_marker_locations.sto'));
delete('_ik_model_marker_locations.sto');


% Check for errors!
% Add cycle to check!
%https://simtk-confluence.stanford.edu/display/OpenSim/Checklist+-+Evaluating+your+Simulation
% Is the maximum marker error less than 4 cm?
% Is the RMS error less than ~2 cm?

errors = load_sto_file(fullfile(outputPath, 'IK_marker_errors.sto'));
[maxMarkerError maxMarkerIndex] = max(errors.marker_error_max);
[maxRMSError maxRMSIndex] = max(errors.marker_error_RMS);

if (maxMarkerError <= 0.04 && maxRMSError <= 0.02)
    resultsAreValid = true;

    disp(['Max Marker Error: ' num2str(maxMarkerError) ' at Frame ' num2str(maxMarkerIndex)]);
    disp(['Max RMS Error: ' num2str(maxRMSError) ' at Frame ' num2str(maxRMSIndex)]);
else
    fprintf(2,['Max Marker Error: ' num2str(maxMarkerError) ' at Frame ' num2str(maxMarkerIndex) '\n']);
    fprintf(2,['Max RMS Error: ' num2str(maxRMSError) ' at Frame ' num2str(maxRMSIndex) '\n']);
    if reportMarkerError < 1
        fprintf(2,['Aber vermutlich egal ... \n']);
    end
end
if reportMarkerError >= 1
    fileID = fopen(fullfile(outputPath, 'marker_errors_summary.txt'),'w');
    fprintf(fileID,'Maximaler Marker Error: %.1f mm\n', maxMarkerError*1000);
    fprintf(fileID,'Maximaler Marker Error at frame: %0.f\n', maxMarkerIndex);
    fprintf(fileID,'Maximaler RMS Error: %.1f mm\n', maxRMSError*1000);
    fprintf(fileID,'Maximaler RMS Error at frame: %0.f\n', maxRMSIndex);
    fclose(fileID);
end
end

