package frc.robot.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {

    // Two Falcon 500s (TalonFX) geared 1:1 driving the flywheel
    private static final DCMotor SHOOTER_MOTOR = DCMotor.getFalcon500(2);
    private static final double  GEAR_RATIO    = 1.0;
    // Moment of inertia estimate for a shooter flywheel (kg·m²)
    private static final double  MOI           = 0.004;

    private final FlywheelSim shooterSim =
            new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(SHOOTER_MOTOR, MOI, GEAR_RATIO),
                    SHOOTER_MOTOR);

    // Simple first-order model for the kicker (SparkMax + NEO, ~0.001 kg·m²)
    private static final DCMotor KICKER_MOTOR    = DCMotor.getNEO(1);
    private static final double  KICKER_MOI      = 0.001;
    private static final double  KICKER_GEAR_RATIO = 1.0;

    private final FlywheelSim kickerSim =
            new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(KICKER_MOTOR, KICKER_MOI, KICKER_GEAR_RATIO),
                    KICKER_MOTOR);

    // Commanded outputs
    private double shooterVelocitySetpointRPS = 0.0;
    private double kickerOutput               = 0.0;

    // Simple proportional closed-loop voltage model (mimics onboard PID)
    private static final double kP_SIM    = 0.5;  // V / (RPS error)
    private static final double kV_SIM    = 0.12; // V·s / rotation (feed-forward)
    private static final double NOMINAL_V = 12.0;

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // --- Shooter ---
        double error        = shooterVelocitySetpointRPS - shooterSim.getAngularVelocityRPM() / 60.0;
        double shooterVolts = MathUtil.clamp(
                kV_SIM * shooterVelocitySetpointRPS + kP_SIM * error,
                -NOMINAL_V, NOMINAL_V);

        shooterSim.setInputVoltage(shooterVolts);
        shooterSim.update(0.02); // 20 ms robot loop

        inputs.shooterVelocityRPS  = shooterSim.getAngularVelocityRPM() / 60.0;
        inputs.shooterAppliedVolts = shooterVolts;
        inputs.shooterCurrentAmps  = shooterSim.getCurrentDrawAmps();
        inputs.shooterTempCelsius  = 25.0; // constant in sim
        inputs.shooterMotorConnected = true;

        // --- Kicker ---
        double kickerVolts = MathUtil.clamp(kickerOutput * NOMINAL_V, -NOMINAL_V, NOMINAL_V);
        kickerSim.setInputVoltage(kickerVolts);
        kickerSim.update(0.02);

        inputs.kickerAppliedVolts = kickerVolts;
        inputs.kickerCurrentAmps  = kickerSim.getCurrentDrawAmps();
    }

    @Override
    public void setShooterVelocity(double velocityRPS) {
        shooterVelocitySetpointRPS = velocityRPS;
    }

    @Override
    public void setKickerOutput(double output) {
        kickerOutput = MathUtil.clamp(output, -1.0, 1.0);
    }

    @Override
    public void stop() {
        shooterVelocitySetpointRPS = 0.0;
        kickerOutput               = 0.0;
    }
}