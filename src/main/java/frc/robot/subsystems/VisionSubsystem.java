package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout
    
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera camera;

    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    
    private boolean hasTarget = false;
    //idk if this is private but i will just put it here
    private PhotonTrackedTarget target;
    
    private double targetYaw = 0.0;
    private double targetPitch = 0.0;
    private double targetArea = 0.0;
    private double targetSkew = 0.0;
    List<TargetCorner> targetCorners = new ArrayList<>();
    double poseAmbiguity = 0.0;
    //When camera is mounted, set these variables
    Pose3d cameraToRobot;
    double cameraHeightMeters;
    double cameraPitchRadians;
    
    private int targetID = -1;

    public VisionSubsystem(String cameraName) {
        camera = new PhotonCamera(cameraName);
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

    public doubel getTargetCorners() {
        return hasTarget ? targetCorners : 0.0;
    }

    public int getTargetID() {
        return targetID;
    }

    //Wonder how photonvision handles null values
    public Transform3d getBestCameraToTarget() {
        return hasTarget ? target.getBestCameraToTarget() : null;
    }

    public Pose3d fieldRelativePos() {
        if (aprilTagFieldLayout.getTagPose(targetID).isPresent()) {
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(targetID).get(), cameraToRobot);
            return robotPose;
        } else {
            return null;
        }
    }

    /*
    public double calcDistanceToTarget() {
        return PhotonUtils.calculateDistanceToTargetMeters(double cameraHeightMeters, double targetHeightMeters, double cameraPitchRadians, double targetPitchRadians)
    }
    */
    
    
}






