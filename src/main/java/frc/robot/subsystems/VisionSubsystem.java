//FIIIIIIINAAAAAAAAAL FFFFFFFFLLLLLLLLLLAAAAAAAAAASSSSSSSSSSSSHHHHHHHHHHHHHHHHHHHHH!
//if you see this then vscode automatically connects to github

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
    private Pose3d cameraToRobot;
    private final double cameraHeightMeters;
    private double cameraPitchRadians;
    
    private int targetID = -1;

    public VisionSubsystem(String cameraName) {
        camera = new PhotonCamera(cameraName);
    }
    
    //Pipeline setters and getters, not sure if we will actually use these.
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

    //Checks how reliable the target info is based on poseAmbiguity
    public boolean hasReliablePose(double maxAmbiguity) {
        return hasTarget && poseAmbiguity >= 0 && poseAmbiguity < maxAmbiguity;
    }

    public Optional<Transform3d> getBestCameraToTarget() {
        return hasTarget ? Optional.of(target.getBestCameraToTarget()) : Optional.empty();
    }


    public Optional<Pose3d> getFieldRelativePose() {
        if (!hasTarget) {
            return Optional.empty();
        }
        
        return aprilTagFieldLayout.getTagPose(targetID).map(tagPose -> PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),tagPose,cameraToRobot));
    }

    //WE NEED cameraHeightMeters and cameraPitchRadians. This method should stay commented until we know those two, and then we can also calculate targetHeightMeters afterward.
    
    /*
    public double calcDistanceToTarget() {
        return PhotonUtils.calculateDistanceToTargetMeters(double cameraHeightMeters, double targetHeightMeters, double cameraPitchRadians, double targetPitchRadians)
    }
    */
    
    
    
}








