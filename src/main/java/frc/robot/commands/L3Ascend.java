package frc.robot.subsystems;

import frc.robot.subsystems.L3Climb;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class L3Ascend extends Command {
    
    private final L3Climb climb = new L3Climb();
    private int climbStatus = 0;

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
        if (climb.getClimbStatus() != "outerArmsDone") {
            climb.pullOuterArms();
        } else {
            climb.pullInnerArms();
        }
    }

    public void end() {
        return !(climb.getTopLimit() || climb.getHookLimit());
    }

    @Override
    public boolean isFinished() {
    }

    @Override
    public boolean runsWhileDisabled() {
        return false;
    }
}