package frc.robot.subsystems;

import frc.robot.subsystems.L3Climb;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class L3Ascend extends Command {
    
    private L3Climb climb;

    public L3Ascend(L3Climb climb) {
        this.climb = climb;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.slideOut();
        climb.setStatus("AtBase");
    }

    @Override
    public void execute() {
        if (climb.getClimbLevel() < 3) {
            if (!climb.getClimbStatus().equals("OuterArmsDone")) {
                climb.pullOuterArms();
            } else {
                climb.pullInnerArms();
            }
        } else {
            climb.pullInnerArmsHalfway();
        }

    }

    @Override
    public void end(boolean interrupted) {
        climb.stopClimb();
    }

    @Override
    public boolean isFinished() {
        return climb.getClimbLevel() == 3 && climb.getClimbStatus().equals("InnerArmsDoneHalfway");
    }

    @Override
    public boolean runsWhileDisabled() {
        return false;
    }
}