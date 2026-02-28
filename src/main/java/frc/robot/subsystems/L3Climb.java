package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.L3ClimbConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DigitalInput;

public class Climb extends SubsystemBase {
    //1 = left (facing forwards), 2 = right (facing forwards)

    private final SparkMaxConfig climbMotor1Config = new SparkMaxConfig();
    private final SparkMaxConfig climbMotor2Config = new SparkMaxConfig();
    private final SparkMaxConfig slideMotor1Config = new SparkMaxConfig();
    private final SparkMaxConfig slideMotor2Config = new SparkMaxConfig();

    private final SparkMax climbMotor1 = new SparkMax(L3ClimbConstants.climbMotor1CanId, MotorType.kBrushless);
    private final SparkMax climbMotor2 = new SparkMax(L3ClimbConstants.climbMotor2CanId, MotorType.kBrushless);

    private final SparkMax slideMotor1 = new SparkMax(L3ClimbConstants.slideMotor1CanId, MotorType.kBrushless);
    private final SparkMax slideMotor2 = new SparkMax(L3ClimbConstants.slideMotor2CanId, MotorType.kBrushless);

    private RelativeEncoder climbEncoder = climbMotor1.getEncoder();
    private RelativeEncoder sliderEncoder = slideMotor1.getEncoder(); 

    private final DigitalInput limitSwitchHook = new DigitalInput(1);
    private final DigitalInput limitSwitchTop = new DigitalInput(2);
    private final DigitalInput limitSwitchBottom = new DigitalInput(3);

    private double ClimbStatus;

    public Climb() {
        
        SparkMaxConfig[] configs = {climbMotor1Config, climbMotor2Config, slideMotor1Config, slideMotor2Config};
        
        for (SparkMaxConfig climbMotorConfig : configs) {
            climbMotorConfig
                //.inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40);
        }
        climbMotor1.configure(climbMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climbMotor2.configure(climbMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        slideMotor3.configure(slideMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        slideMotor4.configure(slideMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        climbMotor2.follow(climbMotor1,true);
        slideMotor2.follow(slideMotor1,true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climb limit switch HOOK", limitSwitchHook.get())
        SmartDashboard.putBoolean("Climb limit switch TOP", limitSwitchTop.get());
        SmartDashboard.putBoolean("Climb limit switch BOTTOM", limitSwitchBottom.get());
        SmartDashboard.putNumber(" Climb position", getDistance());
    }

    public void pull(){
        if (limitSwitchHook.get()) {
            if (!limitSwitchTop.get()) {
                climbMotor1.set(0);
            } else {       
                climbMotor1.set(L3ClimbConstants.climbSpeed);
            }
        }
    }
    
    public void push() {
        if (limitSwitchBottom.get()) {
            climbMotor1.set(0);
        } else {
            climbMotor1.set(-L3ClimbConstants.climbSpeed);
        }
    }

    public void slideOut() {
        if (getSliderDistance() >= 4) {
            slideMotor1.set(0);
        } else {
            slideMotor1.set(L3ClimbConstants.slideSpeed);
        }
    }

    public void slideIn() {
        double current = slideMotor1.getOutputCurrent();
        if (current > 30) {
            slideMotor1.set(0);
        } else {
            slideMotor1.set(-L3ClimbConstants.slideSpeed);
        }
    }

    public void stopClimb() {
        climbMotor1.set(0);
    }

    public void reset(){
        climbMotor1.set(-L3ClimbConstants.climbSpeed);
    }
    
    public boolean getHookLimit() {
        return limitSwitchHook.get();
    }

    public boolean getBottomLimit() {
        return limitSwitchBottom.get();
    }

    public boolean getTopLimit() {
        return limitSwitchTop.get();
    }
    
    
    public double getDistance() {
        double climbMotorRadius = 0.689;
        double distance = climbEncoder.getPosition() * 2 * Math.PI * climbMotorRadius;
        return distance;
    }

    public double getSliderDistance() {
        double climbMotorRadius = 0.689;
        double distance = sliderEncoder.getPosition() * 2 * Math.PI * climbMotorRadius;
        return distance;
    }


}