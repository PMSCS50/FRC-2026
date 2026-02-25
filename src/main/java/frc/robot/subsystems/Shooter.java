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
    private final TalonFXConfiguration shooterMotor1Config = new TalonFXConfiguration();
    private final TalonFXConfiguration shooterMotor2Config = new TalonFXConfiguration();
    private final SparkMaxConfig kickerMotor1Config = new SparkMaxConfig();
    private final SparkMaxConfig kickerMotor2Config = new SparkMaxConfig();

    final TalonFX shooterMotor1 = new TalonFX(ShooterConstants.shooterMotor1CanId);
    final TalonFX shooterMotor2 = new TalonFX(ShooterConstants.shooterMotor2CanId);
    final SparkMax kickerMotor1 = new SparkMax(ShooterConstants.kickerMotor1CanId, MotorType.kBrushless);
    final SparkMax kickerMotor2 = new SparkMax(ShooterConstants.kickerMotor2CanId, MotorType.kBrushless);


    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);



    private double velocity = 0.0;
    private double shooterAngle = 70.0; //shooter angle
    private double shooterHeight = 0.508; //How high the shooter is from the ground (meters)

    public Shooter() {
        //TalonFX shooterMotorConfig
        configureShooterMotor(shooterMotor1Config);
        configureShooterMotor(shooterMotor2Config);

        shooterMotor1.getConfigurator().apply(shooterMotor1Config);
        shooterMotor2.getConfigurator().apply(shooterMotor2Config);

        kickerMotor1Config
            // .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(20);

        kickerMotor1.configure(kickerMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        kickerMotor2Config
            // .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(20);

        kickerMotor2.configure(kickerMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureShooterMotor(TalonFXConfiguration shooterMotorConfig) {
        shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        shooterMotorConfig.Slot0.kP = 0.1;
        shooterMotorConfig.Slot0.kI = 0;
        shooterMotorConfig.Slot0.kD = 0;
    }

    @Override
    public void periodic() {

    }    
    
    //Calculates velocity for trajectory to get in shooter given distance; 
    public double velocityFromDistance(double x) {
        double y = 1.8288 - shooterHeight;
        double phi = Math.toRadians(shooterAngle);
        double v = Math.sqrt((9.807 * x * x) / (2 * Math.cos(phi) * Math.cos(phi) * (x * Math.tan(phi) + shooterHeight - y)));       
        double dragFactor = (1 + 0.015*x) * 1.04;
        return dragFactor * v;
    }


    public void setVelocityTo(double newVelocity) {
        velocity = newVelocity;
        shooterMotor1.setControl(velocityRequest.withVelocity(convertToRPM(velocity / 60.0)));
        shooterMotor2.setControl(velocityRequest.withVelocity(convertToRPM(velocity / 60.0)));
    }

    public void startKickerMotor() {
        kickerMotor1.set(ShooterConstants.kickerMotorPower);
        kickerMotor2.set(ShooterConstants.kickerMotorPower);
    }

    
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


