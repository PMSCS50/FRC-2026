package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.SparkPIDController;
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

    //for starting the intake
    private final Timer initTimer = new Timer();
    private boolean initializing = false;
    
    public Intake() {
        viagraMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        viagraMotor.configure(viagraMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        intakeMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 
    }

    @Override
    public void periodic() {

    }

    public void initinitIntake() {
        initTimer.restart();
        initializing = true;
    }

    //This works because when the intake hits the bumper, the stall uses a huge amount of current.
    //So, we can just check if the current usage exceeds 30 amps and then stop the motor.
    public void initIntake() {
        if (!initializing) return;

        double viagraPower = 0.2     
        double ampThreshold = 30;  
        double timeoutSeconds = 2.0;

        double current = viagraMotor.getOutputCurrent();

        if (initTimer.hasElapsed(timeoutSeconds)) {
            viagraMotor.set(0);
            initializing = false;
            return;
        }

        if (current < ampThreshold) {
            intakePivotMotor.set(viagraPower);
        }

        else {
            intakePivotMotor.set(0);
            viagraEncoder.setPosition(0);
            initializing = false;
        }
    }

    //all power values are placeholders
    public void startIntake() {
        intakeMotor.set(0.4); 
    }

    public void stopIntake() {
        intakeMotor.set(0.0);
    }

 
}