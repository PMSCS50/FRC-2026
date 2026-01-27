package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
//import java.util.HashMap;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.function.TriConsumer;



public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera camera;
    // CHECKSTYLE:OFF ConstantName
    public final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    
    private boolean hasTarget = false;
    
    // idk if this is private but i will just put it here

    PhotonTrackedTarget target;
    
    private double targetYaw = 0.0;
    private double targetPitch = 0.0;
    private double targetArea = 0.0;
    private double targetSkew = 0.0;
    private List<TargetCorner> targetCorners = new ArrayList<>();
    private double poseAmbiguity = 0.0;

    // When camera is mounted, set these variables
    Transform3d cameraToRobot = null;
    Transform3d robotToTarget = null;

    double cameraHeightMeters = 0.0;
    double cameraPitchRadians = 0.0;

    // Pipeline Indexes. Dont know if we will use these but im putting them here just in case
    public static final int APRILTAG_PIPELINE = 0;
    public static final int TAPE_PIPELINE = 1;
    public static final int DRIVER_PIPELINE = 2;
    
    private int targetID = -1;

    //gonna start trying to use this guy
    public PhotonPoseEstimator photonPoseEstimator;
    private PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    
    private Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.9, 0.9, Math.toRadians(10));// Added by ChatGPT

    private TriConsumer<Pose2d, Double, Matrix<N3, N1>> estConsumer = new TriConsumer<>();



    public VisionSubsystem(String cameraName) {
        camera = new PhotonCamera(cameraName);
       
    }

    // Optional overload to supply camera-to-robot transform at construction
    public VisionSubsystem(String cameraName, Transform3d cameraToRobot, double cameraHeightMeters, double cameraPitchRadians, TriConsumer<Pose2d, Double, Matrix<N3, N1>> estConsumer) {

        camera = new PhotonCamera(cameraName);
        this.cameraToRobot = cameraToRobot;
        this.cameraHeightMeters = cameraHeightMeters;
        this.cameraPitchRadians = cameraPitchRadians;
        this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagLayout,poseStrategy,cameraToRobot);
        this.estConsumer = estConsumer;
    }

    // Pipeline setters and getters, not sure if we will actually use these.
    public void setPipeline(int index) {
        camera.setPipelineIndex(index);
    }
    
    public int getPipeline() {
        return camera.getPipelineIndex();
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets()) {
            hasTarget = false;
            targetID = -1;
            return;
        }
        
        targets  = result.getTargets()
        target = result.getBestTarget();
        
        hasTarget = true;
        targetYaw = target.getYaw();
        
        targetPitch = target.getPitch();
        targetArea = target.getArea();
        targetSkew = target.getSkew();
        //targetCorners = target.getCorners(); DOESNT EXIST
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
    /*
    public double getTargetYaw(int targetID) {
        return hasTarget ? targetYaw : 0.0;
    }

    public double getTargetPitch(int targetID) {
        return hasTarget ? targetPitch : 0.0;
    }

    public double getTargetArea(int targetID) {
        return hasTarget ? targetArea : 0.0
    }

    public double getTargetSkew(int targetID) {
        return hasTarget ? targetSkew : 0.0;
    }
    */
    public List<TargetCorner> getTargetCorners(int targetID) {
        return hasTarget ? targetCorners : List.of();
    }   

    public int getTargetID() {
        return targetID;
    }

    // Checks how reliable the target info is based on poseAmbiguity
    public boolean hasReliablePose(double maxAmbiguity) {
        return hasTarget && poseAmbiguity >= 0 && poseAmbiguity < maxAmbiguity;
    }

    public Transform3d getBestCameraToTarget() {
        return hasTarget ? target.getBestCameraToTarget() : Transform3d.kZero;
    }

    /*
    public Optional<Transform3d> getAlternateCameraToTarget() {
        return hasTarget ? Optional.ofNullable(target.getAlternateCameraToTarget()) : Optional.empty();
    }
    */
    
    public Transform3d getBestRobotToTarget() {
        if (!hasTarget || cameraToRobot == null) {
            return Transform3d.kZero;
        }

        Transform3d cameraToTarget = target.getBestCameraToTarget();
        if (cameraToTarget == null) {
            return Transform3d.kZero;
        }

        robotToTarget = cameraToRobot.inverse().plus(cameraToTarget);
        return robotToTarget;
    }
    
    public double getX() {
        return robotToTarget != Transform3d.kZero ? robotToTarget.getX() : 0.0;
    }
    public double getY() {
        return robotToTarget != Transform3d.kZero ? robotToTarget.getY() : 0.0;
    }
    public double getZ() {
        return robotToTarget != Transform3d.kZero ? robotToTarget.getZ() : 0.0;
    }

    public double getRot() {
        return robotToTarget != Transform3d.kZero ? robotToTarget.getRotation().getZ() : 0.0;
    }
    

    public Optional<Pose3d> getFieldRelativePose() {
        if (!hasTarget || cameraToRobot == null) {
            return Optional.empty();
        }
        
        return aprilTagLayout.getTagPose(targetID).map(tagPose -> PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), tagPose, cameraToRobot));
    }
    
    //Fallback in case best target is ambiguous and we need to use a different one
    public Optional<Pose3d> getFieldRelativePose(PhotonTrackedTarget target1) {
        if (!hasTarget || cameraToRobot == null) {
            return Optional.empty();
        }
        
        // idk why but Kevin named it "aprilTagLayout" instead of "aprilTagFieldLayout" - David
        return aprilTagLayout.getTagPose(target1.getFiducialId()).map(tagPose -> PhotonUtils.estimateFieldToRobotAprilTag(target1.getBestCameraToTarget(), tagPose, cameraToRobot));
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
    // And then we can also calculate targetHeightMeters afterward.
    
    
    public double calcDistanceToTarget(double cameraHeightMeters, double targetHeightMeters, double cameraPitchRadians, double targetPitchRadians) {
        return PhotonUtils.calculateDistanceToTargetMeters(cameraHeightMeters, targetHeightMeters, cameraPitchRadians, targetPitchRadians);
    }

    // ChatGPT says we need to create the updateEstimationStdDevs method - David

    @SuppressWarnings("unused")
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

        /*
        * WPILib heuristic:
        *  - More tags → more trust
        *  - Closer tags → more trust
        */
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

    // ChatGPT also says that we need to define estConsumer - David
    /*
    public Optional<EstimatedRobotPose> estimateMultiTagPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        
        for (var result : camera.getAllUnreadResults()) {
            visionEst = photonPoseEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonPoseEstimator.estimateLowestAmbiguityPose(result);
            }

            updateEstimationStdDevs(visionEst, result.getTargets());
            
            //any part of the method past here may need some later reviewing
            //do we have something like Robot.isSimulation()?
            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est ->
                                getSimDebugField()
                                        .getObject("VisionEstimation")
                                        .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                        }
                );
            }

            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();

                        estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    }
            );
        }
        return visionEst;
    }
    */

    
}


// Define 
// isSimulation (in frc.robot.Robot, maybe in simulationInit or simulationPeriodic),
// getSimDebugField,   
// and estConsumer DONE
// also robotToTarget DONE
// ill be AFK for a few hours


