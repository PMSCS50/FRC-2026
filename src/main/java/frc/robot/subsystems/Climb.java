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


public class Climb extends SubsystemBase {
    //1 = left (facing forwards), 2 = right (facing forwards)
    
    //private SparkMax CoralRoller2 = new SparkMax(CoralRollersConstants.coralRoller1CanId, MotorType.kBrushless);
    private RelativeEncoder climbEncoder = climbMotor.getEncoder(); 
    private final DigitalInput limitSwitchHook = new DigitalInput(1);
    private final DigitalInput limitSwitchTop = new DigitalInput(2);
    private final DigitalInput limitSwitchBottom = new DigitalInput(3);
    

    private double ClimbStatus;

    public Climb() {
        climbMotorConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        //ChuteMotor1.configure(coralRollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //ChuteMotor2.configure(coralRollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        climbMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //CoralRoller2.configure(coralRollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        limit.get();
        SmartDashboard.putBoolean("Climb limit switch HOOK", clawLimitSwitch.get())
        SmartDashboard.putBoolean("Climb limit switch TOP", limit2.get());
        SmartDashboard.putBoolean("Climb limit switch BOTTOM", limit3.get());
        SmartDashboard.putNumber("Climb position", climbEncoder.getPosition());
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
    
    public void getHookLimit() {
        return limitSwitchHook.get();
    }

    public void getBottomLimit() {
        return limitSwitchBottom.get();
    }

    public void getTopLimit() {
        return limitSwitchTop.get();
    }
    
    //Do later
    public double getDistance() {
        return 0;
    }



}