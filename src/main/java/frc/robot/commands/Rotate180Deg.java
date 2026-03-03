import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import java.lang.Math;

public class Rotate180Deg extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private PIDController rotController = new PIDController(1, 0, 0);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

    public Rotate180Deg(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        rotController.setTolerance(0.05);
    }
    
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double omegaShenron = rotController.calculate(drivetrain.getPose().getRotation().getZ(), Math.PI);
        drivetrain.setControl(
            drive.with VelocityX(0)
                 .withVelocityY(0)
                 .withRotationalRate(omegaShenron)
        );
    }

    @Override
    public boolean isFinished() {
        return rotController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle());
    }


}

