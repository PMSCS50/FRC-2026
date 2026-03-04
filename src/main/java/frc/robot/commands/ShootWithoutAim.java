package frc.robot.subsystems;

import frc.robot.subsystems.Shooter;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystem.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ShootWithoutAim extends Command {
    private Shooter shooter;
    private VisionSubsystem vision;

    private PhotonPipelineResult result;
    private PhotonTrackedTarget target;
    private Timer stopTimer;

    public ShootWithoutAim(Shooter shooter, VisionSubsystem vision) {
        this.shooter = shooter;
        this.vision = vision;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        PhotonPipelineResult result = vision.getLatestResult();
        targetOptional = result.getTargets().stream()
                            .filter(t -> t.getFiducialId() == ShooterConstants.HUB_TAG_ID)
                            .findFirst();
        
        stopTimer = new Timer();
    }

    @Override
    public void execute() {
        if (targetOptional.isPresent()) {
            var target = targetOptional.get();

            var translation = target.getBestCameraToTarget().inverse().getTranslation();
            double aprilTagToHub = 0.610816; 
            
            double dx = translation.getX() + aprilTagToHub; //forward distance to hub (RobotToApriltagX + AprilTagToHubX)
            double dy = translation.getY(); //horizontal distance to the hub
            
            double distance = Math.hypot(dx,dy);

            double velocity = shooter.velocityFromDistance(distance);

            shooter.setVelocityTo(velocity);

            if (shooter.atCorrectRPM()) {
                shooter.startKickerMotors();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return stopTimer.hasElapsed(2500);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        shooter.stopKickerMotors();
    }
}