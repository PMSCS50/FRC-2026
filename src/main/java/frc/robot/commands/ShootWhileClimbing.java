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
import frc.robot.subsystems.Shooter;
import java.lang.Math;
import java.util.Optional;

public class ShootWhileClimbing extends Command {

    private final Shooter shooter;

    public ShootWhileClimbing( Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    //Dumping the mag when on L3. 
    //Note that z is just the height of the ladder at L3, and that we need to fix that to our robot height.
    @Override
    public void execute() {
        double z = 1.83134
        double distance = 3.543578; // distance from climb to hub. No angling required nor possible
        double velocity = shooter.velocityFromDistance(distance, z, robotPitch);

        shooter.setVelocityTo(velocity);

        if (shooter.atCorrectRPM()) {
                shooter.startKickerMotors();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public void isFinished() {
        return false;
    }
}
