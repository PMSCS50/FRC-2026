package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

public class FaceAprilTagRelative extends CommandBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    private final DoubleSupplier xInput;
    private final DoubleSupplier yInput;

    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
    PIDController xController,yController,rotController;

    // NOTE: This gain must be tuned. We convert yaw (degrees) -> radians before applying.
    private static final double kPRotate = 2.0;

    public FaceAprilTagRelative(
            CommandSwerveDrivetrain drivetrain,
            VisionSubsystem vision,
            DoubleSupplier xInput,
            DoubleSupplier yInput)
    {
        xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
        yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horizontal movement
        rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation

        this.drivetrain = drivetrain;
        this.vision = vision;
        this.xInput = xInput;
        this.yInput = yInput;

        // Declare subsystem requirement
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        // Nothing needed on start
    }

    @Override
    public void execute() {
        double vx = xInput.getAsDouble();
        double vy = yInput.getAsDouble();
        //requires elite ball knowledge. 
        double omegaShenron = 0;

        PhotonPipelineResult result = vision.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            // target.getYaw() is in degrees (PhotonVision). Convert to radians.
            double yawRad = Math.toRadians(target.getYaw());
            
            omegaShenron = - yawRad * kPRotate; // sign may need to be flipped depending on camera convention
            
        }

        drivetrain.setControl(
                drive.withVelocityX(vx)
                     .withVelocityY(vy)
                     .withRotationalRate(omegaShenron)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
                drive.withVelocityX(0)
                     .withVelocityY(0)
                     .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until toggled off
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}

