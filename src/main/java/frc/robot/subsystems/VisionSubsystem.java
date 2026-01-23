package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera camera;
    // CHECKSTYLE:OFF ConstantName
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    
    private boolean hasTarget = false;
    
    // idk if this is private but i will just put it here
    private PhotonTrackedTarget target;
    
    private double targetYaw = 0.0;
    private double targetPitch = 0.0;
    private double targetArea = 0.0;
    private double targetSkew = 0.0;
    private List<TargetCorner> targetCorners = new ArrayList<>();
    private double poseAmbiguity = 0.0;

    // When camera is mounted, set these variables
    private Transform3d cameraToRobot = null;
    private double cameraHeightMeters = 0.0;
    private double cameraPitchRadians = 0.0;

    // Pipeline Indexes. Dont know if we will use these but im putting them here just in case
    public static final int APRILTAG_PIPELINE = 0;
    public static final int TAPE_PIPELINE = 1;
    public static final int DRIVER_PIPELINE = 2;
    
    private int targetID = -1;

    public VisionSubsystem(String cameraName) {
        camera = new PhotonCamera(cameraName);
    }

    // Optional overload to supply camera-to-robot transform at construction
    public VisionSubsystem(String cameraName, Transform3d cameraToRobot, double cameraHeightMeters, double cameraPitchRadians) {
        camera = new PhotonCamera(cameraName);
        this.cameraToRobot = cameraToRobot;
        this.cameraHeightMeters = cameraHeightMeters;
        this.cameraPitchRadians = cameraPitchRadians;
    }

    // Pipeline setters and getters, not sure if we will actually use these.
    public void setPipeline(int index) {
        camera.setPipelineIndex(index);
    }
    
    public int getPipeline() {
        return camera.getPipelineIndex();
    }


    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets()) {
            hasTarget = false;
            targetID = -1;
            return;
        }

        target = result.getBestTarget();

        hasTarget = true;
        targetYaw = target.getYaw();
        
        targetPitch = target.getPitch();
        targetArea = target.getArea();
        targetSkew = target.getSkew();
        targetCorners = target.getCorners();
        poseAmbiguity = target.getPoseAmbiguity();
        
        targetID = target.getFiducialId();
    }



    public boolean hasTargets() {
        return hasTarget;
    }

    public double getTargetYaw() {
        return hasTarget ? targetYaw : 0.0;
    }

    public double getTargetPitch() {
        return hasTarget ? targetPitch : 0.0;
    }

    public double getTargetArea() {
        return hasTarget ? targetArea : 0.0;
    }

    public double getTargetSkew() {
        return hasTarget ? targetSkew : 0.0;
    }

    public List<TargetCorner> getTargetCorners() {
        return hasTarget ? targetCorners : List.of();
    }

    public int getTargetID() {
        return targetID;
    }

    // Checks how reliable the target info is based on poseAmbiguity
    public boolean hasReliablePose(double maxAmbiguity) {
        return hasTarget && poseAmbiguity >= 0 && poseAmbiguity < maxAmbiguity;
    }

    public Optional<Transform3d> getBestCameraToTarget() {
        return hasTarget ? Optional.ofNullable(target.getBestCameraToTarget()) : Optional.empty();
    }
    
    public Optional<Transform3d> getAlternateCameraToTarget() {
        return hasTarget ? Optional.ofNullable(target.getAlternateCameraToTarget()) : Optional.empty();
    }
    

    public Optional<Pose3d> getFieldRelativePose() {
        if (!hasTarget || cameraToRobot == null) {
            return Optional.empty();
        }
        
        return aprilTagFieldLayout.getTagPose(targetID).map(tagPose -> PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), tagPose, cameraToRobot));
    }

    // setters for camera mount info
    public void setCameraToRobot(Transform3d cameraToRobot) {
        this.cameraToRobot = cameraToRobot;
    }

    public void setCameraHeightMeters(double cameraHeightMeters) {
        this.cameraHeightMeters = cameraHeightMeters;
    }

    public void setCameraPitchRadians(double cameraPitchRadians) {
        this.cameraPitchRadians = cameraPitchRadians;
    }

    // We Need cameraHeightMeters and cameraPitchRadians to calculate distance to target
    // This method should stay commented until the camera is properly mounted on the robot, 
    // And then we can also calculate targetHeightMeters afterward.
    
    /* 
    public double calcDistanceToTarget(double cameraHeightMeters, double targetHeightMeters, double cameraPitchRadians, double targetPitchRadians) {
        return PhotonUtils.calculateDistanceToTargetMeters(cameraHeightMeters, targetHeightMeters, cameraPitchRadians, targetPitchRadians);
    }
    */
    
    
    
}
