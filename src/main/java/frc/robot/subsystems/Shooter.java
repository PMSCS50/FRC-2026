package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;



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
    private final TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();
    private final SparkMaxConfig kickerMotorConfig = new SparkMaxConfig();

    final TalonFX shooterMotor = new TalonFX(ShooterConstants.shooterMotorCanId);
    final SparkMax kickerMotor = new SparkMax(ShooterConstants.kickerMotorCanId, MotorType.kBrushless);
    
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);



    // Current commanded output (interpreted as percent output by SparkMax.set)
    private double velocity = 0.0;
    public double shooterAngle = 70.0; //shooter angle
    private double shooterHeight = 0.508; //How high the shooter is from the ground (meters)

    public Shooter() {
        //TalonFX shooterMotorConfig
        configureShooterMotor();

        shooterMotor.getConfigurator().apply(shooterMotorConfig);

        kickerMotorConfig
            // .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(20);

        kickerMotor.configure(shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureShooterMotor() {
        shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        shooterMotorconfig.Slot0.kP = 8.3; // Just a guess, do later
        shooterMotorconfig.Slot0.kI = 0;
        shooterMotorconfig.Slot0.kD = 0;
    }

    @Override
    public void periodic() {

    }    
    
    //Calculates velocity for trajectory to get in shooter given distance. Y value is fixed); 
    private double velocityFromDistance(double x) {
        double y = 1.8288 - shooterHeight; // y distance from shooter to hub. May have to change later
        double phi = Math.toRadians(shooterAngle);
        double v = Math.sqrt((9.807 * x * x) / (2 * Math.cos(phi) * Math.cos(phi) * (x*Math.tan(phi) + shooterHeight - y)));       
        double kp = v / 8;
        return kp * v; //Fuck air resistance
    }


    public void setVelocityTo(double newVelocity) {
        velocity = newVelocity;
        shooterMotor.setControl(velocityRequest.withVelocity(convertToRPM(rpm / 60.0)));
    }

    
    //We need to finish this
    private double convertToRPM(double velocity) {

        double wheelRadius = 0.0508;
        double kp = 1.1; //extra constant to try and account for energy loss
        
        double wheelRPM = kp * (velocity * 60.0) / (2.0 * Math.PI * wheelRadius);

        return wheelRPM;

    }

    /** Stop the shooter. */
    public void stop() {
        this.setVelocityTo(0.0);
    }


    public double getVelocity() {
        return velocity;
    }

    /** Returns true if the shooter is currently running (non-zero velocity). */
    public boolean isShooting() {
        return velocity != 0.0;
    }
}


