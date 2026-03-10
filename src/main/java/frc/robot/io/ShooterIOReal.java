package frc.robot.io;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOReal implements ShooterIO {

    // --- TalonFX shooter motors ---
    private final TalonFX shooterMotor1 = new TalonFX(ShooterConstants.shooterMotor1CanId);
    private final TalonFX shooterMotor2 = new TalonFX(ShooterConstants.shooterMotor2CanId);

    // --- SparkMax kicker motors ---
    private final SparkMax kickerMotor1 = new SparkMax(ShooterConstants.kickerMotor1CanId, MotorType.kBrushless);
    private final SparkMax kickerMotor2 = new SparkMax(ShooterConstants.kickerMotor2CanId, MotorType.kBrushless);

    // --- Cached status signals for efficient periodic reads ---
    private final StatusSignal<AngularVelocity> shooterVelocitySignal;
    private final StatusSignal<Voltage>          shooterVoltageSignal;
    private final StatusSignal<Current>          shooterCurrentSignal;
    private final StatusSignal<Temperature>      shooterTempSignal;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withSlot(0);

    public ShooterIOReal() {
        // --- Configure shooter motors ---
        TalonFXConfiguration motor1Config = new TalonFXConfiguration();
        TalonFXConfiguration motor2Config = new TalonFXConfiguration();

        configureShooterMotor(motor1Config);
        configureShooterMotor(motor2Config);
        motor1Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        shooterMotor1.getConfigurator().apply(motor1Config);
        shooterMotor2.getConfigurator().apply(motor2Config);
        shooterMotor2.setControl(new Follower(shooterMotor1.getDeviceID(), MotorAlignmentValue.Opposed));

        // --- Configure kicker motors ---
        SparkMaxConfig kicker1Config = new SparkMaxConfig();
        SparkMaxConfig kicker2Config = new SparkMaxConfig();

        kicker1Config.idleMode(IdleMode.kCoast).smartCurrentLimit(20);
        kicker2Config.idleMode(IdleMode.kCoast).smartCurrentLimit(20);
        kicker2Config.follow(kickerMotor1, true);

        kickerMotor1.configure(kicker1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kickerMotor2.configure(kicker2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- Cache status signals ---
        shooterVelocitySignal = shooterMotor1.getVelocity();
        shooterVoltageSignal  = shooterMotor1.getMotorVoltage();
        shooterCurrentSignal  = shooterMotor1.getSupplyCurrent();
        shooterTempSignal     = shooterMotor1.getDeviceTemp();

        // Refresh all signals together at the robot loop rate (50 Hz)
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                shooterVelocitySignal,
                shooterVoltageSignal,
                shooterCurrentSignal,
                shooterTempSignal);

        shooterMotor1.optimizeBusUtilization();
        shooterMotor2.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Refresh all signals in one bus call
        inputs.shooterMotorConnected =
                BaseStatusSignal.refreshAll(
                                shooterVelocitySignal,
                                shooterVoltageSignal,
                                shooterCurrentSignal,
                                shooterTempSignal)
                        .isOK();

        inputs.shooterVelocityRPS   = shooterVelocitySignal.getValueAsDouble();
        inputs.shooterAppliedVolts  = shooterVoltageSignal.getValueAsDouble();
        inputs.shooterCurrentAmps   = shooterCurrentSignal.getValueAsDouble();
        inputs.shooterTempCelsius   = shooterTempSignal.getValueAsDouble();

        inputs.kickerAppliedVolts   = kickerMotor1.getAppliedOutput() * kickerMotor1.getBusVoltage();
        inputs.kickerCurrentAmps    = kickerMotor1.getOutputCurrent();
    }

    @Override
    public void setShooterVelocity(double velocityRPS) {
        shooterMotor1.setControl(velocityRequest.withVelocity(velocityRPS));
    }

    @Override
    public void setKickerOutput(double output) {
        kickerMotor1.set(output);
    }

    @Override
    public void stop() {
        shooterMotor1.stopMotor();
        kickerMotor1.stopMotor();
    }

    // ---------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------

    private void configureShooterMotor(TalonFXConfiguration cfg) {
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.CurrentLimits.SupplyCurrentLimit = 40.0;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        cfg.Slot0.kS = 0.1;
        cfg.Slot0.kV = 0.12;
        cfg.Slot0.kP = 0.11;
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.0;
    }
}