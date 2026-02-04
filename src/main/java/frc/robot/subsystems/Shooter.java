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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.MathUtil;

import java.lang.Math;

/*
TODO:
Initialize other motorrCanIDs and configure the kickerMotor
Initialize wheelRadius and find the max speed of a SparkMax with the FUEL inside.
Finish convertToRPM();

*/ 

public class Shooter extends SubsystemBase {
    // Configuration for the shooter motor
    private final SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig kickerMotorConfig = new SparkMaxConfig();

    final SparkMax shooterMotor = new SparkMax(Constants.ShooterConstants.shooterMotorCanId, MotorType.kBrushless);
    final SparkMax kickerMotor = new SparkMax(Constants.ShooterConstants.kickerMotorCanId, MotorType.kBrushless);
    

    // Current commanded output (interpreted as percent output by SparkMax.set)
    private double velocity = 0.0;
    public double shooterAngle = 70.0; //shooter angle
    private double shooterHeight = 0.508; //How high the shooter is from the ground (meters)

    public Shooter() {
        shooterMotorConfig
            // .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(20);

        shooterMotor.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        kickerMotorConfig
            // .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(20);

        kickerMotor.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {

    }    
    
    //Calculates velocity for trajectory to get in shooter given distance. Y value is fixed); 
    private double velocityFromDistance(double x) {
        double y = 1.8288 - shooterHeight; // y distance from shooter to hub. May have to change later
        double phi = Math.toRadians(shooterAngle);
        double v = Math.sqrt((9.807 * x * x) / (2 * Math.cos(phi) * Math.cos(phi) * (x*tan(phi) + shooterHeight - y)));       
        return v;
    }

    public void setVelocityFromDistance(double x) {
        this.setVelocity(velocityFromDistance(x));
    }
    

    public void setShootingVelocity(double newVelocity) {
        velocity = newVelocity;
        shooterMotor.set(convertToRPM(velocity));
    }

    
    //We need to finish this
    private double convertToRPM(double velocity) {

        double wheelRadius = 0.0508;
        double k = 1.1; //extra constant to try and account for energy loss
        
        double wheelRPM = k * (velocity * 60.0) / (2.0 * Math.PI * wheelRadius);
        //This is FREE maxwheelRPM. Replace with reduced one
        double maxWheelRPM = 5676.0;

        double percentOutput = MathUtil.clamp(wheelRPM / maxWheelRPM, 0.0, 1.0);
        return percentOutput;

    }


    /** Stop the shooter. */
    public void stop() {
        this.setShootingVelocity(0.0);
    }

    public double getVelocity() {
        return velocity;
    }

    /** Returns true if the shooter is currently running (non-zero velocity). */
    public boolean isShooting() {
        return velocity != 0.0;
    }
}


