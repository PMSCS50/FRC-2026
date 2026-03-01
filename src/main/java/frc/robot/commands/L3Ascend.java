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
    }

    @Override
    public void execute() {
        if (!climb.getClimbStatus().equals("OuterArmsDone")) {
            climb.pullOuterArms();
        } else {
            climb.pullInnerArms();
        }
    }

    @Override
    public void end() {
        climb.stopClimb();
    }

    @Override
    public boolean isFinished() {
        return !(climb.getTopLimit() || climb.getHookLimit());
    }

    @Override
    public boolean runsWhileDisabled() {
        return false;
    }
}