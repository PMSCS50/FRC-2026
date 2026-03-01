package frc.robot.subsystems;

import frc.robot.subsystems.L3Climb;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class L1Descend extends Command {
    
    private final L3Climb climb = new L3Climb();

    public L1Descend(L3Climb climb) {
        this.climb = climb;

        addRequirements(climb);
    }

    @Override
    public void initialize() {

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
        return climb.getClimbStatus() == "InnerArmsDoneHalfway";
    }

    @Override
    public boolean runsWhileDisabled() {
        return false;
    }
}