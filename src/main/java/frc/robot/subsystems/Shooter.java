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
    public double shooterAngle = 20.0; //angle
    private double shooterHeight = 0.1; //How high is the shooter from the ground?

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

    private double bestAngleFromDistance(double x, double y) {
        double minAngle = Math.toDegrees(Math.atan(2 * yDiff / x)); //safe, but not best angle.
        double phi_ideal = Math.atan(x/( Math.sqrt(x*x + (shooterHeight - y)*(shooterHeight - y)) + shooterHeight - y));
        //phi_ideal is angle that passes through (x,y) with the least velocity
        double phi = Math.max(minAngle, phi_ideal);
        phi = Math.min(55.0,Math.max(phi,20.0));
        return phi;
    }
    
    
    //Will calculate velocity for trajectory to hit (x,y); 
    private double velocityFromDistance(double x, double y) {
        double phi = shooterAngle;
        double v = Math.sqrt((9.807 * x * x) / (2 * Math.cos(phi) * Math.cos(phi) * (x*tan(phi) + shooterHeight - y)));       
        return v;
    }

    public void setVelocityFromDistance(double x, double y) {
        this.setVelocity(velocityFromDistance(x,y));
    }
    

    public void setVelocity(double newVelocity) {
        velocity = newVelocity;
        shooterMotor.set(velocity);
    }

    public void setBestAngle(double newAngle) {
        shooterAngle = newAngle;
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
