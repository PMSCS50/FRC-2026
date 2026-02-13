package frc.robot.commands;


import java.util.Optional;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.wpilibj2.command.Command;


import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class UpdateFieldToRobot extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;

    public UpdateFieldToRobot(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;

        addRequirements(vision);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        Optional<EstimatedRobotPose> fieldToRobot = vision.estimateMultiTagPose();

        fieldToRobot.ifPresent(
            erp -> {
                Matrix<N3, N1> visionStdDevs = vision.getEstimationStdDevs();

                //SmartDashboard.putNumber("Estimated FieldToRobot", fieldToRobot)
                //SmartDashboard.putNumber("VisionStdDevX", xStd);
                //SmartDashboard.putNumber("VisionStdDevY", yStd);
                //SmartDashboard.putNumber("VisionStdDevTheta", thetaStd);

                drivetrain.addVisionMeasurement(erp.estimatedPose.toPose2d(), erp.timestampSeconds, visionStdDevs);
            }
        );

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}