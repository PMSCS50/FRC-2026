package frc.robot.subsystems;

import frc.robot.subsystems.L3Climb;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class L1Ascend extends Command {
    
    private L3Climb climb;

    public L1Ascend(L3Climb climb) {
        this.climb = climb;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.slideOut();
    }

    @Override
    public void execute() {
        if (climb.getTopLimit()) {
            climb.pullOuterArms();
        } else {
            climb.pullInnerArmsHalfway();
        }
    }

    public void end() {

    }

    @Override
    public boolean isFinished() {
        return climb.getClimbStatus().equals("InnerArmsDoneHalfway");
    }

    @Override
    public boolean runsWhileDisabled() {
        return false;
    }
}