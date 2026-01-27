package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Shooter extends SubsystemBase {
    // Configuration for the shooter motor
    private final SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();


    private final SparkMax shooterMotor = new SparkMax(ShooterConstants.shooterMotorCanId, MotorType.kBrushless);
    //add more motors later

    // Current commanded output (interpreted as percent output by SparkMax.set)
    private double velocity = 0.0;

    public Shooter() {
        shooterMotorConfig
            // .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(20);

        shooterMotor.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {

    }

    private double velocityFromDistance(double meters) {
        return (meters + 4) * 100; // placeholder
    }

    public void setVelocityFromDistance(double meters) {
        this.setVelocity(velocityFromDistance(meters));
    }

    public void setVelocity(double newVelocity) {
        velocity = newVelocity;
        shooterMotor.set(velocity);
    }

    /** Stop the shooter. */
    public void stop() {
        this.setVelocity(0.0);
    }

    public double getVelocity() {
        return velocity;
    }

    /** Returns true if the shooter is currently running (non-zero velocity). */
    public boolean isShooting() {
        return velocity != 0.0;
    }
}


/*
public class Shooter extends SubsystemBase {
    //1 = left (facing forwards), 2 = right (facing forwards)
    private SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();

    private SparkMax shooterMotor = new SparkMax(CoralRollersConstants.coralRoller1CanId, MotorType.kBrushless);
    // private SparkMax shooterMotor2 = new SparkMax(CoralRollersConstants.coralRoller2CanId, MotorType.kBrushless);

    public double velocity = 0.0;

    public CoralRollers() {
        shooterMotorConfig
            //.inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(20);

        shooterMotor.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // shooterMotor.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {

    }

    public void setVelocity(double newVelocity) {
        velocity = newVelocity;{
        shooterMotor.set(velocity);
    }

    public void stop(){
        velocity = 0.0
        shooterMotor.set(velocity);
    }

    public boolean isShooting() {
        return velocity == 0.0;
    }
}
*/
