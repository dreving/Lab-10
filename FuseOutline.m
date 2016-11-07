function [delta] = poseDiff(poseVec1,poseVec2)
%static function
x2 = poseVec2(1);
x1 = poseVec1(1);
y2 = poseVec2(2);
y1 = poseVec1(2);
th2 = poseVec2(3);
th1 = poseVec1(3);
delta = [x2-x1; y2 - y1; atan2(sin(th2-th1),cos(th2-th1))];
end

function [fusedPose] = fusePose(odoPose,lidPose)
fusedPose = odoPose + obj.fuseGain*CLASSNAME.poseDiff(odoPose,lidPose);
end



%when odometry is updated
%update both with odometry integration
%check if map is new
[success, lidPose] = lml.refinePose(fusedPose,ptsInModelFrame, maxIters)
if success
    fusedPose = fusePose(odoPose,lidPose)
end
%if map is successful, then run fused pose to update 


%runfusedPose