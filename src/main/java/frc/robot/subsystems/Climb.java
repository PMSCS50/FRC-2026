package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Winch;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

public class Climb extends SubsystemBase {
    //1 = left (facing forwards), 2 = right (facing forwards)
    
    //private SparkMax CoralRoller2 = new SparkMax(CoralRollersConstants.coralRoller1CanId, MotorType.kBrushless);
    private RelativeEncoder climbEncoder = climbMotor.getEncoder(); 
    private final DigitalInput limitSwitchHook = new DigitalInput(1);
    private final DigitalInput limitSwitchTop = new DigitalInput(2);
    private final DigitalInput limitSwitchBottom = new DigitalInput(3);

    private final DigitalOutput LEDHook = new DigitalOutput(4);
    private final DigitalOutput LEDTop = new DigitalOutput(5);
    private final DigitalOutput LEDBottom = new DigitalOutput(6);
    

    private double ClimbStatus;

    public Climb() {
        climbMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        climbMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climb limit switch HOOK", limitSwitchHook.get())
        SmartDashboard.putBoolean("Climb limit switch TOP", limitSwitchTop.get());
        SmartDashboard.putBoolean("Climb limit switch BOTTOM", limitSwitchBottom.get());
        SmartDashboard.putNumber("Climb position", getDistance());

        LEDHook.set(limitSwitchHook.get());
        LEDTop.set(limitSwitchTop.get());
        LEDBottom.set(limitSwitchBottom.get());
    }

    public void pull(){
        if (limitSwitchHook.get()) {
            if (!limitSwitchTop.get()) {
                climbMotor.set(0);
            } else {       
                climbMotor.set(ClimbConstants.climbSpeed);
            }
        }
    }
    
    public void push() {
        if (limitSwitchBottom.get()) {
            climbMotor.set(0);
        } else {
            climbMotor.set(-ClimbConstants.climbSpeed);
        }
    }

    public void stopClimb() {
        climbMotor.set(0);
    }

    public void reset(){
        climbMotor.set(-ClimbConstants.climbSpeed);
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
        double climbMotorRadius = 0; //radius in inches
        double distance = climbEncoder.getPosition() * 2 * Math.PI * climbMotorRadius;
        return distance;
    }



}