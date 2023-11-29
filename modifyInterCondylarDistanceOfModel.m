import org.opensim.modeling.*;

GetSubDirsFirstLevelOnly()

[modelFileName, path] = uigetfile('*.osim');
osimModel = Model(fullfile(path, modelFileName));

leftDistance = 0.04;
rightDistance = 0.03;

%% left
% med_cond_joint_l
medCondJoint = osimModel.getJointSet().get('med_cond_joint_l');
sagArtFrame = medCondJoint.get_frames(0);
if ~contains(char(sagArtFrame.toString), 'sagittal_articulation_frame')
    sagArtFrame = medCondJoint.get_frames(1);
    if ~contains(char(sagArtFrame.toString), 'sagittal_articulation_frame')
        return;
    end
end
baseTranslation = sagArtFrame.get_translation;
baseTranslation.set(2, leftDistance/2);
sagArtFrame.set_translation(baseTranslation)

% lat_cond_joint_l
latCondJoint = osimModel.getJointSet().get('lat_cond_joint_l');
sagArtFrameLat = latCondJoint.get_frames(0);c
if ~contains(char(sagArtFrameLat.toString), 'sagittal_articulation_frame')
    sagArtFrameLat = latCondJoint.get_frames(1);
    if ~contains(char(sagArtFrameLat.toString), 'sagittal_articulation_frame')
        return;
    end
end
baseTranslation = sagArtFrameLat.get_translation;
baseTranslation.set(2, -leftDistance/2);
sagArtFrameLat.set_translation(baseTranslation)

% med_cond_weld_l
medCondWeld = osimModel.getJointSet().get('med_cond_weld_l');
medCondFrame = medCondWeld.get_frames(0);
if ~contains(char(medCondFrame.toString), 'med_cond')
    medCondFrame = medCondWeld.get_frames(1);
    if ~contains(char(medCondFrame.toString), 'med_cond')
        return;
    end
end
baseTranslation = medCondFrame.get_translation;
baseTranslation.set(2, -leftDistance/2);
medCondFrame.set_translation(baseTranslation)



%% right
% med_cond_joint_l
medCondJoint = osimModel.getJointSet().get('med_cond_joint_r');
sagArtFrame = medCondJoint.get_frames(0);
if ~contains(char(sagArtFrame.toString), 'sagittal_articulation_frame')
    sagArtFrame = medCondJoint.get_frames(1);
    if ~contains(char(sagArtFrame.toString), 'sagittal_articulation_frame')
        return;
    end
end
baseTranslation = sagArtFrame.get_translation;
baseTranslation.set(2, -rightDistance/2);
sagArtFrame.set_translation(baseTranslation)

% lat_cond_joint_l
latCondJoint = osimModel.getJointSet().get('lat_cond_joint_r');
sagArtFrameLat = latCondJoint.get_frames(0);
if ~contains(char(sagArtFrameLat.toString), 'sagittal_articulation_frame')
    sagArtFrameLat = latCondJoint.get_frames(1);
    if ~contains(char(sagArtFrameLat.toString), 'sagittal_articulation_frame')
        return;
    end
end
baseTranslation = sagArtFrameLat.get_translation;
baseTranslation.set(2, rightDistance/2);
sagArtFrameLat.set_translation(baseTranslation)

% med_cond_weld_l
medCondWeld = osimModel.getJointSet().get('med_cond_weld_r');
medCondFrame = medCondWeld.get_frames(0);
if ~contains(char(medCondFrame.toString), 'med_cond')
    medCondFrame = medCondWeld.get_frames(1);
    if ~contains(char(medCondFrame.toString), 'med_cond')
        return;
    end
end
baseTranslation = medCondFrame.get_translation;
baseTranslation.set(2, rightDistance/2);
medCondFrame.set_translation(baseTranslation)


newModelName = strrep(modelFileName, '.osim', '_adjCond.osim');
osimModel.print(fullfile(path, newModelName))
