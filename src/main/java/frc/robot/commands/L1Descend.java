package frc.robot.commands;

import frc.robot.subsystems.L3Climb;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class L1Descend extends Command {
    
    private L3Climb climb;

    public L1Descend(L3Climb climb) {
        this.climb = climb;

        addRequirements(climb);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (climb.getClimbLevel() == 1) {
            climb.pullOuterArms();
        } else if (climb.getSlideStatus() == "out") {
            climb.slideIn();
        } else {
            climb.pullInnerArmsHalfway();
        }
    }

    public void end(boolean interrupted) {
        return;
    }

    @Override
    public boolean isFinished() {
        return climb.getClimbLevel() == 0 && climb.getClimbStatus().equals("InnerArmsDoneHalfway");
    }

}