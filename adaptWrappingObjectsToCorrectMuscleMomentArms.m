function successfull = adaptWrappingObjectsToCorrectMuscleMomentArms(modelFilename, motionFilename, stepSize, threshold)
%CHECKMUSCLEMOMENTARM Summary of this function goes here
%   Detailed explanation goes here
tic

if ~exist('stepSize', 'var')
    stepSize = 0.001; % 1mm
end
if ~exist('threshold', 'var')
    threshold = 0.003;
end

import org.opensim.modeling.*

motion = Storage(motionFilename);

newModelFilename = strrep(modelFilename, '.osim', '_modWO.osim');
copyfile(modelFilename, newModelFilename);

hasDiscontinuities = 1; failed = 0;

while(hasDiscontinuities)

    model = Model(newModelFilename);

    model.initSystem();
    state = model.initSystem();

    flexIndexL = model.getCoordinateSet().getIndex('hip_flexion_l');
    flexCoordL = model.updCoordinateSet().get(flexIndexL);
    rotIndexL = model.getCoordinateSet().getIndex('hip_rotation_l');
    rotCoordL = model.updCoordinateSet().get(rotIndexL);
    addIndexL = model.getCoordinateSet().getIndex('hip_adduction_l');
    addCoordL = model.updCoordinateSet().get(addIndexL);
    flexIndexR = model.getCoordinateSet().getIndex('hip_flexion_r');
    flexCoordR = model.updCoordinateSet().get(flexIndexR);
    rotIndexR = model.getCoordinateSet().getIndex('hip_rotation_r');
    rotCoordR = model.updCoordinateSet().get(rotIndexR);
    addIndexR = model.getCoordinateSet().getIndex('hip_adduction_r');
    addCoordR = model.updCoordinateSet().get(addIndexR);

    numMuscles = model.getMuscles().getSize();
    muscleIndices = []; muscleNames = {};
    for i = 0 : numMuscles - 1
        tmp_muscleName = char(model.getMuscles().get(i).getName());
        if contains(tmp_muscleName, 'add') || contains(tmp_muscleName, 'gl') ...
                || contains(tmp_muscleName, 'semi') || contains(tmp_muscleName, 'bf') ...
                || contains(tmp_muscleName, 'grac') || contains(tmp_muscleName, 'piri') ...
                || contains(tmp_muscleName, 'sart') || contains(tmp_muscleName, 'tfl') ...
                || contains(tmp_muscleName, 'iliacus') || contains(tmp_muscleName, 'psoas') ...
                || contains(tmp_muscleName, 'rect')
            muscleIndices = [muscleIndices, i];
            muscleNames{end+1} = tmp_muscleName;
        end
    end

    flexMomentArms = zeros(motion.getSize(), length(muscleIndices));
    addMomentArms = zeros(motion.getSize(), length(muscleIndices));
    rotMomentArms = zeros(motion.getSize(), length(muscleIndices));

    for i = 1:motion.getSize()
        flexAngleL = motion.getStateVector(i-1).getData().get(flexIndexL) / 180 * pi;
        rotAngleL = motion.getStateVector(i-1).getData().get(rotIndexL) / 180 * pi;
        addAngleL = motion.getStateVector(i-1).getData().get(addIndexL) / 180 * pi;
        flexAngleR = motion.getStateVector(i-1).getData().get(flexIndexR) / 180 * pi;
        rotAngleR = motion.getStateVector(i-1).getData().get(rotIndexR) / 180 * pi;
        addAngleR = motion.getStateVector(i-1).getData().get(addIndexR) / 180 * pi;

        % Update the state with the joint angle
        coordSet = model.updCoordinateSet();
        coordSet.get(flexIndexL).setValue(state, flexAngleL);
        coordSet.get(rotIndexL).setValue(state, rotAngleL);
        coordSet.get(addIndexL).setValue(state, addAngleL);
        coordSet.get(flexIndexR).setValue(state, flexAngleR);
        coordSet.get(rotIndexR).setValue(state, rotAngleR);
        coordSet.get(addIndexR).setValue(state, addAngleR);

        % Realize the state to compute dependent quantities
        model.computeStateVariableDerivatives(state);
        model.realizeVelocity(state);

        % Compute the moment arm for each muscle
        for j = 1:length(muscleIndices)
            muscleIndex = muscleIndices(j);
            if strcmp(muscleNames{j}(end), 'l')
                flexMomentArm = model.getMuscles().get(muscleIndex).computeMomentArm(state, flexCoordL);
                flexMomentArms(i,j) = flexMomentArm;
                rotMomentArm = model.getMuscles().get(muscleIndex).computeMomentArm(state, rotCoordL);
                rotMomentArms(i,j) = rotMomentArm;
                addMomentArm = model.getMuscles().get(muscleIndex).computeMomentArm(state, addCoordL);
                addMomentArms(i,j) = addMomentArm;
            else
                flexMomentArm = model.getMuscles().get(muscleIndex).computeMomentArm(state, flexCoordR);
                flexMomentArms(i,j) = flexMomentArm;
                rotMomentArm = model.getMuscles().get(muscleIndex).computeMomentArm(state, rotCoordR);
                rotMomentArms(i,j) = rotMomentArm;
                addMomentArm = model.getMuscles().get(muscleIndex).computeMomentArm(state, addCoordR);
                addMomentArms(i,j) = addMomentArm;
            end
        end
    end

    toc

    % now check for discontinuities

    musclesWithDiscontinuities = {};
    
    for i = 1 : size(flexMomentArms, 2)
        dy = diff(flexMomentArms(:, i));
        discontinuity_indices = find(abs(dy) > threshold);
        if size(discontinuity_indices, 1) > 0
            musclesWithDiscontinuities{end + 1} = muscleNames{i};
        end

        dy = diff(addMomentArms(:, i));
        discontinuity_indices = find(abs(dy) > threshold);
        if size(discontinuity_indices, 1) > 0
            musclesWithDiscontinuities{end + 1} = muscleNames{i};
        end

        dy = diff(rotMomentArms(:, i));
        discontinuity_indices = find(abs(dy) > threshold);
        if size(discontinuity_indices, 1) > 0
            musclesWithDiscontinuities{end + 1} = muscleNames{i};
        end
    end

    if size(musclesWithDiscontinuities, 1) > 0
        % modify corresponding wrapping object
        musclesWithDiscontinuities = unique(musclesWithDiscontinuities);

        disp([num2str(numel(musclesWithDiscontinuities)) ' muscles with discontinuities were detected']);
        
        for m = 1 : numel(musclesWithDiscontinuities)
            muscleName = musclesWithDiscontinuities{m};
            muscle = model.getMuscles().get(muscleName);

            geoPath = muscle.getGeometryPath;
            wrapSet = geoPath.getWrapSet;
            for w = 1 : wrapSet.getSize()
                wrapObj = wrapSet.get(w-1);
                wrapObjName = wrapObj.getWrapObjectName;
                for i = 1 : model.getBodySet.getSize
                    currBody = model.getBodySet.get(i-1);
                    for j = 1 : currBody.getWrapObjectSet.getSize
                        if strcmp(currBody.getWrapObjectSet.get(j-1).getName, wrapObjName)
                            wrapObject = currBody.getWrapObjectSet.get(j-1);
                            wrapCylinder = WrapCylinder.safeDownCast(wrapObject);
                            radius = wrapCylinder.get_radius;
                            if radius - stepSize > 0
                                wrapCylinder.set_radius(radius - stepSize)
                                model.print(newModelFilename);
                                disp([muscleName ': radius decreased to ' num2str(radius - stepSize)]);
                                break;
                            else
                                disp([muscleName ' moment arm wrong but wrap object is already too small! Check manually']);
                                failed = 1;
                                hasDiscontinuities = 0;
                            end
                        end
                    end
                end
            end
        end
    else
        hasDiscontinuities = 0;
    end
end

if failed
    fprintf(2, 'Something went wrong!');
    successfull = 0;
else
    successfull = 1;
end

figure('Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8]);
plot(flexMomentArms);
title(['All muscle moment arms in motion ' motionFilename]);
legend(muscleNames, 'Interpreter', 'none');
ylabel('Hip Flexion Moment Arm (m)');
xlabel('Frame (after start time)');
drawnow;

end

