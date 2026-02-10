package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera camera;

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

        PhotonTrackedTarget target = result.getBestTarget();

        hasTarget = true;
        targetId = target.getFiducialId();

        // Camera → Tag
        Transform3d cameraToTag = target.getBestCameraToTarget();

        // Robot → Tag
        Transform3d robotToTag = ROBOT_TO_CAMERA.plus(cameraToTag);

        // Tag → Robot (MATCHES LIMELIGHT TARGET SPACE)
        tagToRobot = robotToTag.inverse();
    }

    public boolean hasTarget(int desiredId) {
        return hasTarget && targetId == desiredId && tagToRobot != null;
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
        return tagToRobot != null
            ? tagToRobot.getRotation().getZ()
            : 0.0;
    }
}
