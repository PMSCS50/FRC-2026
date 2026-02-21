package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Intake extends SubsystemBase {
    //I am calling it the viagra motor because it turns the intake on
    //We will need this to be a tradition
    private final SparkMaxConfig viagraMotorConfig = new SparkMaxConfig();
    private final SparkMax viagraMotor = new SparkMax(IntakeConstants.viagraMotorCanID, MotorType.kBrushless);
    private final RelativeEncoder viagraEncoder = viagraMotor.getEncoder();
   
    private final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    private final SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeMotorCanID, MotorType.kBrushless);

    private final SparkMaxConfig beltMotorConfig = new SparkMaxConfig();
    private final SparkMax beltMotor = new SparkMax(IntakeConstants.beltMotorCanID, MotorType.kBrushless);
    
    public Intake() {
        viagraMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        viagraMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        intakeMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        beltMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        beltMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {

    }

    public void initIntake() {
        //replace 1/3 with (angle motor turns from start to bumper / 360)
        viagraEncoder.setPosition(1/4);
    }


    public void intakeIntake() {
        intakeMotor.set(0.4);
    }

    public void agitateAgitators() {
        beltMotor.set(0.3);
    }

    public void stopIntake() {
        intakeMotor.set(0.0);
    }

    public void stopAgitators() {
        beltMotor.set(0.0);
    }
}