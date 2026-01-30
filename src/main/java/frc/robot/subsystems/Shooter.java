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
Initialize other motorrCanIDs and hootEncoderDIO.
Also configure other motors
Initialize wheelRadius and max speed.


*/ 

public class Shooter extends SubsystemBase {
    // Configuration for the shooter motor
    private final SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();


    final SparkMax shooterMotor = new SparkMax(ShooterConstants.shooterMotorCanId, MotorType.kBrushless);
    final SparkMax kickerMotor = new SparkMax(ShooterConstants.kickbackMotorCanId, MotorType.kBrushless);
    final SparkMax hoodMotor = new SparkMax(ShooterConstants.hoodedMotorCanId, MotorType.kBrushless);

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
    }

    @Override
    public void periodic() {

    }    
    
    //Will calculate velocity for trajectory to hit (x,y); 
    private double velocityFromDistance(double x, double y) {
        double phi = Math.toRadians(shooterAngle);
        double v = Math.sqrt((9.807 * x * x) / (2 * Math.cos(phi) * Math.cos(phi) * (x*tan(phi) + shooterHeight - y)));       
        return v;
    }

    public void setVelocityFromDistance(double x, double y) {
        this.setVelocity(velocityFromDistance(x,y));
    }
    

    public void setVelocity(double newVelocity) {
        velocity = newVelocity;
        shooterMotor.set(convertToRPM(velocity));
    }

    private double convertToRPM(double velocity) {
        /*
        double wheelRadius = 0.0508; 
        
        double wheelRPM = (velocity * 60.0) / (2.0 * Math.PI * wheelRadius);

        double maxWheelRPM = 5676.0 / ShooterConstants.GEAR_RATIO;

        double percentOutput = MathUtil.clamp(wheelRPM / maxWheelRPM, 0.0, 1.0);

        return percentOutput;

        */
        return 0.0;
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
