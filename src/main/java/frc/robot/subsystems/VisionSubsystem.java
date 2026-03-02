package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N6;

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera camera;
    private PhotonTrackedTarget target;

    public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private final PhotonPoseEstimator photonPoseEstimator;

    // Robot → Camera
    private static final Transform3d ROBOT_TO_CAMERA =
        new Transform3d(
            new Translation3d(0.23, 0.051, 0.25),
            new Rotation3d(0, 0, 0)
        );

    private boolean hasTarget = false;
    private int targetId = -1;

    // TAG → ROBOT (this is the important one)
    private Transform3d tagToRobot = null;

    public VisionSubsystem(String cameraName) {
        camera = new PhotonCamera(cameraName);
        this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagLayout, ROBOT_TO_CAMERA);
    }

    //These two methods are only for other commands and classes. We should not be using them in here
    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public PhotonTrackedTarget getBestTarget() {
        return getLatestResult().getBestTarget();
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets()) {
            hasTarget = false;
            targetId = -1;
            tagToRobot = null;
            return;
        }

        target = result.getBestTarget();

        hasTarget = true;
        targetId = target.getFiducialId();

        // Robot → Tag
        Transform3d robotToTag = getRobotToTarget();

        // Tag → Robot (MATCHES LIMELIGHT TARGET SPACE)
        tagToRobot = robotToTag.inverse();
    }

    public boolean hasTargets() {
        return hasTarget;
    }

    public boolean hasTarget(int desiredId) {
        return hasTarget && targetId == desiredId && tagToRobot != null;
    }

    public int getTargetId() {
        return targetId;
    }

    public Transform3d getRobotToTarget() {
        if (!hasTarget) {
            return null;
        }

        Transform3d cameraToTag = target.getBestCameraToTarget();
        if (cameraToTag == null) {
            return null;
        }
        return ROBOT_TO_CAMERA.plus(cameraToTag);
    }

    // Forward/back relative to TAG (meters)
    public double getX() {
        return tagToRobot != null ? tagToRobot.getX() : 0.0;
    }

    // Left/right relative to TAG (meters)
    public double getY() {
        return tagToRobot != null ? tagToRobot.getY() : 0.0;
    }

    // Robot yaw relative to TAG (radians)
    public double getYawRad() {
        return tagToRobot != null ? tagToRobot.getRotation().getZ() : 0.0;
    }

    //--------------------------
    //UpdateFieldToRobot helpers
    //--------------------------

    private Matrix<N6, N1> visionStdDevs = VecBuilder.fill(0,0,0,0,0,0);

    //Ambiguity of Photon poseEstimation
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose,
        List<PhotonTrackedTarget> targets) {

        if (estimatedPose.isEmpty() || targets.isEmpty()) {
            return;
        }

        int numTags = targets.size();

        double avgDist = 0.0;
        for (PhotonTrackedTarget target : targets) {
            avgDist += target.getBestCameraToTarget()
                                .getTranslation()
                                .getNorm();
        }
        avgDist /= numTags;

        double xyStd;
        double zStd;
        double yawStd;
        double rollPitchStd;

        if (numTags >= 2) {
            xyStd = 0.4 * avgDist;
            zStd = 0.6 * avgDist;
            yawStd = Math.toRadians(4);
            rollPitchStd = Math.toRadians(6);
        } else {
            xyStd = 1.0 * avgDist;
            zStd = 1.5 * avgDist;
            yawStd = Math.toRadians(10);
            rollPitchStd = Math.toRadians(15);
        }

        visionStdDevs = VecBuilder.fill(
            xyStd,         // X
            xyStd,         // Y
            zStd,          // Z
            rollPitchStd,  // Roll
            rollPitchStd,  // Pitch
            yawStd         // Yaw
        );
        }

    public Matrix<N6, N1> getEstimationStdDevs() {
        return visionStdDevs;
    }

    //gets fieldToRobot
    public Optional<EstimatedRobotPose> estimateMultiTagPose() {

        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var result : camera.getAllUnreadResults()) {
            visionEst = photonPoseEstimator.estimateCoprocMultiTagPose(result);
            if (vision.isEmpty()) {
                visionEst = photonPoseEstimator.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEst, result.getTargets());
        }
        
        return visionEst;
    }
}
