import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;


public class FiveDollarFootlong extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private PIDController xController = new PIDController(1, 0, 0);
    private double foot = -0.305;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    public FiveDollarFootlong(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        xController.setTolerance(0.05);
    }

    @Override
    public void initialize() {
        xController.reset();
    }

    @Override
    public void execute() {
        double vx = xController.calculate(drivetrain.getPose().getX(), foot);
        drivetrain.setControl(
            drive.withVelocityX(-xVel)
                 .withVelocityY(0)
                 .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }


}