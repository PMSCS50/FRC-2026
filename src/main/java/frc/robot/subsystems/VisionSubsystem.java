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
import edu.wpi.first.math.numbers.N3;
//I cant see errors, so if any of these imports are unused, delete them

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera camera;
    private PhotonTrackedTarget target;

    public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private final PhotonPoseEstimator photonPoseEstimator;
    //private PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

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

    public PhotonTrackedTarget getLatestResult() {
        return camera.getLatestResult();
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

    private Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, Math.toRadians(10));

    //Ambiguity of Photon poseEstimation
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose,
    List<PhotonTrackedTarget> targets) {

        // If we don't have a pose estimate, fall back to default noise
        if (estimatedPose.isEmpty()) {
            visionStdDevs = VecBuilder.fill(0.9, 0.9, Math.toRadians(10));
            return;
        }

        int numTags = targets.size();

        // Average distance from camera to visible AprilTags
        double avgDist = 0.0;
        for (PhotonTrackedTarget target : targets) {
            avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        avgDist /= numTags;

        if (numTags >= 2) {
            visionStdDevs = VecBuilder.fill(
                    0.5 * avgDist,
                    0.5 * avgDist,
                    Math.toRadians(5));
        } else {
            visionStdDevs = VecBuilder.fill(
                    1.0 * avgDist,
                    1.0 * avgDist,
                    Math.toRadians(10));
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
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
