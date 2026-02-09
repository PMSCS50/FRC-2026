package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.ClimbConstants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Climb extends SubsystemBase {
    //1 = left (facing forwards), 2 = right (facing forwards)
    private SparkMaxConfig climbMotorConfig = new SparkMaxConfig();
    private SparkMax climbMotor = new SparkMax(ClimbConstants.climbMotorCanId, MotorType.kBrushless);
    //private SparkMax CoralRoller2 = new SparkMax(CoralRollersConstants.coralRoller1CanId, MotorType.kBrushless);
    private RelativeEncoder climbEncoder = climbMotor.getEncoder(); 
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
        SmartDashboard.putNumber("Climb position", climbEncoder.getPosition());
    }

    public void pull() {
        if(climbEncoder.getPosition() >= ClimbConstants.climbMax) {
            climbMotor.set(0);
        } else {
            climbMotor.set(ClimbConstants.climbSpeed);
        }
        climbMotor.set(ClimbConstants.climbSpeed);

    }

    public void stop(){
        climbMotor.set(0);
    }

    public void reset(){
        climbMotor.set(-ClimbConstants.climbSpeed);
    }

}