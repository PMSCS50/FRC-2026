package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;

public class PV_Align extends Command {
    private PhotonCamera camera;
    public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public PV_Align(PhotonCamera camera) {
        this.camera = camera;
    }
    public void execute() {
        
    }
    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }
    public void align() {
        var result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        
        PhotonTrackedTarget target = result.getBestTarget();
        int targetID = target.getFiducialId();
        double poseAmbiguity = target.getPoseAmbiguity();
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
        Pose3d robotPose;
        Pose3d targetPose;
        if (aprilTagLayout.getTagPose(target.getFiducialId()).isPresent()) {
            robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagLayout.getTagPose(target.getFiducialId()).get(), alternateCameraToTarget);
            targetPose = aprilTagLayout.getTagPose(target.getFiducialId()).get();
        }
        //double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, targetPose);
    }
}
