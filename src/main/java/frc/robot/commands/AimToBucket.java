package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;



public class AimToBucket extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final Shooter shooter;

    private final PIDController rotController;

    private final Timer dontSeeTargetTimer = new Timer();
    private final Timer onTargetTimer = new Timer();

    private final SwerveRequest.RobotCentric drive =
            new SwerveRequest.RobotCentric();

    public AimToBucket(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            Shooter shooter
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.shooter = shooter;

        rotController = new PIDController(
                Constants.BUCKET_AIM_P, 0, 0
        );

        addRequirements(drivetrain, shooter);
    }


    @Override
    public void initialize() {
        dontSeeTargetTimer.restart();
        onTargetTimer.restart();

        rotController.setSetpoint(0.0); // bucket centered
        rotController.setTolerance(Constants.BUCKET_AIM_TOLERANCE_DEG);
    }

    @Override
    public void execute() {
        if (vision.hasTargets()) {
            dontSeeTargetTimer.reset();

            var targetOptional = result.getTargets().stream()
                            .filter(t -> t.getFiducialId() == Constants.BUCKET_TAG_ID)
                            .findFirst();

            if (targetOptional.isPresent()) {
                var target = targetOptional.get();
                dontSeeTargetTimer.reset();
    
                // 1. Get Distance (Direct 3D vector, ignores height constants)
                var translation = target.getBestCameraToTarget().getTranslation();
                double dx = translation.getX(); //forward distance
                double dy = translation.getZ(); //vertical distance 
                
                //double distance = translation.getNorm(); 

                double yaw = target.getYaw(); //2. get yaw for tobot to turn

                // 3. Control Loop
                double rotSpeed = rotController.calculate(yaw);
                drivetrain.setControl(drive.withRotationalRate(rotSpeed));
    
                if (rotController.atSetpoint()) {
                    shooter.setBestAngle(shooter.bestAngleFromDistance(dx,dy));
                    shooter.setVelocityFromDistance(dx,dy);
                } else {
                    shooter.stop();
                }
                return; // Exit early since we found our target
            }
        } else {
            drivetrain.setControl(
                drive.withRotationalRate(0)
            );
            shooter.stop();
        }

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            drive.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
        shooter.stop();
    }


    @Override
    public boolean isFinished() {
        return dontSeeTargetTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME)
            || onTargetTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
    }
}
