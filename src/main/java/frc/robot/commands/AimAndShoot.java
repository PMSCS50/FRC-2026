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
                1, 0, 0
        );

        rotController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain, shooter);
    }


    @Override
    public void initialize() {        
        rotController.setTolerance(0.03);
    }

    @Override
    public void execute() {
        double vx = xInput.getAsDouble() * MaxSpeed * speedLimiter;
        double vy = yInput.getAsDouble() * MaxSpeed * speedLimiter;
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
                
                //2. get target and robot yaw
                double yaw = Math.atan2(dy,dx);
                double robotYaw = drivetrain.getPose().getRotation();
                
                //3. get field speeds
                var fieldSpeeds = 
                    ChassisSpeeds.fromRobotRelativeSpeeds(
                        speeds,
                        robotYaw
                    );

                double vxField = fieldSpeeds.vxMetersPerSecond;
                double vyField = fieldSpeeds.vyMetersPerSecond;

                //4. find and correct values of vv, vy (for shooter, not driving), and yaw
                double[] correctedValues = shooter.correctVandYaw(dx,dy,yaw, vxField, vyField);
                double correctedVx = correctedValues[0];
                double correctedVy = correctedValues[1];
                double bestYaw = correctedValues[2];

                // 5. Control Loop
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
            drivetrain.setControl(
                drive
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(0)
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