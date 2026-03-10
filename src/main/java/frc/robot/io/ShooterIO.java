package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    class ShooterIOInputs {
        public double shooterVelocityRPS = 0.0;      // Shooter wheel velocity in rotations per second
        public double shooterAppliedVolts = 0.0;     // Voltage applied to shooter motor
        public double shooterCurrentAmps = 0.0;      // Supply current draw of shooter motor
        public double shooterTempCelsius = 0.0;      // Motor temperature

        public double kickerAppliedVolts = 0.0;      // Voltage applied to kicker motor
        public double kickerCurrentAmps = 0.0;       // Supply current draw of kicker motor

        public boolean shooterMotorConnected = true; // Whether the shooter motor is reachable
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(ShooterIOInputs inputs) {}

    /** Run shooter at the given velocity in RPS. */
    default void setShooterVelocity(double velocityRPS) {}

    /** Run kicker motors at the given duty-cycle output (-1.0 to 1.0). */
    default void setKickerOutput(double output) {}

    /** Stop all motors. */
    default void stop() {}
}