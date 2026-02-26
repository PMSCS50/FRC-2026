package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import java.lang.Math;
import java.util.Optional;


public class AimAndShoot extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final Shooter shooter;

    private final PIDController rotController;
    private final DoubleSupplier xInput;
    private final DoubleSupplier yInput;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double speedLimiter = 0.5;
    private double directionFlipper = -1.0;

    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric();

    public AimAndShoot(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            Shooter shooter,
            DoubleSupplier xInput,
            DoubleSupplier yInput
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.shooter = shooter;
        this.xInput = xInput;
        this.yInput = yInput;

        rotController = new PIDController(
                Constants.BUCKET_AIM_P, 0, 0
        );

        rotController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain, shooter);
    }


    @Override
    public void initialize() {        
        rotController.setTolerance(0.05);
    }

    @Override
    public void execute() {
        double vx = xInput.getAsDouble() * MaxSpeed * speedLimiter * directionFlipper;
        double vy = yInput.getAsDouble() * MaxSpeed * speedLimiter * directionFlipper;
        if (vision.hasTargets()) {
            PhotonPipelineResult result = vision.getLatestResult();
            var targetOptional = result.getTargets().stream()
                            .filter(t -> t.getFiducialId() == 18)
                            .findFirst();

            if (targetOptional.isPresent()) {
                var target = targetOptional.get();

    
                // 1. Get Distance (Direct 3D vector, ignores height constants)
                var translation = target.getBestCameraToTarget().getTranslation();
                double aprilTagToHub = 0.725180; // May have to change this value
                
                double dx = translation.getX() + aprilTagToHub; //forward distance to hub (RobotToApriltagX + AprilTagToHubX)
                double dy = translation.getY(); //horizontal distance to the hub
                
                double distance = Math.hypot(dx,dy);
                double phi = Math.toRadians(70);

                double unitX = dx / distance;
                double unitY = dy / distance;
                double vStationary = shooter.velocityFromDistance(distance);

                double vHorizontal = vStationary * Math.cos(phi);

                double desiredVx = vHorizontal * unitX;
                double desiredVy = vHorizontal * unitY;

                var speeds = drivetrain.getSpeeds();

                var fieldSpeeds = 
                    ChassisSpeeds.fromRobotRelativeSpeeds(
                        speeds,
                        drivetrain.getState().Pose.getRotation()
                    );

                double vxField = fieldSpeeds.vxMetersPerSecond;
                double vyField = fieldSpeeds.vyMetersPerSecond;

                double correctedVx = desiredVx - vxField;
                double correctedVy = desiredVy - vyField;
                
                //2. get yaw for robot to turn (target.getYawRad() aims to apriltag, not hub.)
                double yaw = Math.atan2(dy,dx);
                double correctedYaw = Math.atan2(correctedVy,correctedVx); 
                double correction = MathUtil.angleModulus(correctedYaw - yaw);
                correction = MathUtil.clamp(correction, -0.2, 0.2); //safety clamp so shit doesnt get too crazy
                double bestYaw = yaw + correction;

                // 3. Control Loop
                rotController.setSetpoint(bestYaw);
                double rotSpeed = rotController.calculate(robotYaw);
                drivetrain.setControl(
                    drive.withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(rotSpeed));
    
                if (rotController.atSetpoint()) {
                    double correctedHorizontal = Math.hypot(correctedVx, correctedVy);
                    double correctedVelocity = correctedHorizontal / Math.cos(phi);

                    shooter.setVelocityTo(correctedVelocity);
                } else {
                    shooter.stop();
                }
                return; // Exit early since we found our target
            } else {
                drivetrain.setControl(
                    drive.withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(0)
                ); 
                shooter.stop();
            }
        } else {
            drivetrain.setControl(drive.
                withRotationalRate(0)
            );
            shooter.stop();
            return;
        }

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            drive.withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(0)
        );
        shooter.stop();
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}