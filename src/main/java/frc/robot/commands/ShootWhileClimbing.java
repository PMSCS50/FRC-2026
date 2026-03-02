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
import frc.robot.subsystems.L3Climb;
import frc.robot.subsystems.VisionSubsystem;
import java.lang.Math;
import java.util.Optional;

public class ShootWhileClimbing extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;

    public ShootWhileClimbing(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        //just in case robot angles itself (which it hopefully wont), i will factor that in shooting
        double robotPitch = drivetrain.getPose().getRotation().getY();
        //new shooterHeight
        double z = driveTrain.getPose().getTranslation().getZ();
        double distance = 3.543578; // distance from climb to hub. No angling required nor possible
        double velocity = shooter.velocityFromDistance();

        shooter.setVelocityTo(velocity);
    }

    @Override
    public void end() {
        shooter.stop();
    }

    @Override
    public void isFinished() {
        return false;
    }
}
