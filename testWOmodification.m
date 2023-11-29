model = fullfile(pwd, 'CP01_adjCond.osim');
trial = 'C:\Users\willi\ucloud\PhD\Study_OI_DIST\SimulationOutput\CP01_adjCond\3DGAIT_B_W12\Output\IK\IK.mot';
stepSize = 0.001;
threshold = 0.003;
adaptWrappingObjectsToCorrectMuscleMomentArms(model, trial, stepSize, threshold);
