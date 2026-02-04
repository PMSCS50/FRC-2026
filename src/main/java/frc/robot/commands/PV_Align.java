package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class PV_Align extends Command {

    private final PIDController xController =
        new PIDController(0.4, 0, 0);   // forward
    private final PIDController yController =
        new PIDController(0.4, 0, 0);   // sideways
    private final PIDController rotController =
        new PIDController(0.4, 0, 0);   // yaw

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;


    private final SwerveRequest.RobotCentric drive =
            new SwerveRequest.RobotCentric();

    private Timer dontSeeTagTimer, stopTimer, idleTimer;

    private static final int TARGET_TAG_ID = 18;

     


    public PV_Align(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        xController.setSetpoint(-.25);
        yController.setSetpoint(0.0);
            rotController.setSetpoint(-1.0);

        xController.setTolerance(0.02);
        yController.setTolerance(0.02);
        rotController.setTolerance(0.25);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        rotController.reset();

        stopTimer = new Timer();
        stopTimer.start();

        dontSeeTagTimer = new Timer();
        dontSeeTagTimer.start();

        idleTimer = new Timer();
        idleTimer.start();
    }

    @Override
    public void execute() {

        if (vision.hasTargets() && vision.getTargetID() == TARGET_TAG_ID) {

            dontSeeTagTimer.reset();

           
            SmartDashboard.putNumber("x", vision.getX());
            SmartDashboard.putNumber("y", vision.getY());
            SmartDashboard.putNumber("rot", vision.getRot());




            
            double xOut   = xController.calculate(vision.getX());
            double yOut   = -yController.calculate(vision.getY());
            double rotOut = rotController.calculate(vision.getRot());

            drivetrain.setControl(
                drive.withVelocityX(xOut)
                     .withVelocityY(yOut)
                     .withRotationalRate(rotOut)
            );
            if (!xController.atSetpoint()
             || !yController.atSetpoint()
             || !rotController.atSetpoint()) {
                stopTimer.reset();
            }
        }
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
        return dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME)
            || stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME)
            || idleTimer.hasElapsed(5);
    }
}
