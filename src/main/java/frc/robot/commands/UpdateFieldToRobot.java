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
        //We can do this. After all, we're Saiyans, we were born to fight!
        //Shut the fuck up Kakarot, save it until we're done!
    }

    @Override
    public void execute() {
        Optional<EstimatedRobotPose> fieldToRobot = vision.estimateMultiTagPose();

        fieldToRobot.ifPresent(
            erp -> {
                Matrix<N3, N1> visionStdDevs = vision.getEstimationStdDevs();
                drivetrain.addVisionMeasurement(erp.estimatedPose.toPose2d(), erp.timestampSeconds, visionStdDevs);
            }
        );

    }

    @Override
    public void end(boolean interrupted) {
        //People of the universe, LEND ME YOUR ENERGY!
        //*energy lending noises or something*
        //*Spirit Bomb completed*
        //Thank you everyone!
        //This is it! HAAAAAAAAAAAA!!!!!!!
        //Please Goku I need this! Your Spirit Bomb is too strong! I'm in no state to dodge it I need you to help me out!
        //Kakarot Im watching your final blow why you trying not to laugh bruh thats disrespecful as shit bruh
    }

    @Override
    public boolean isFinished() {
        
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}