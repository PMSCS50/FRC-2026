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
    private final VisionSubsystem vision;
    private final Shooter shooter;

    public ShootWhileClimbing(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        
    }
}
