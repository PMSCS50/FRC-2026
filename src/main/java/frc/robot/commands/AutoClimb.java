package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbPull extends Command {
    private final Climb climb;

    public ClimbPull(Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.spinRollers();
    }

    @Override
    public void execute() {
        climb.pull(); // winch logic already checks limit switch
    }

    @Override
    public boolean isFinished() {
        return !climb.getLimit(); // ends when switch is released
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopClimb();
        climb.stopRollers();
    }

    
}

