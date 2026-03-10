package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.io.ShooterIO;
import frc.robot.io.ShooterIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

import java.lang.Math;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private double velocity        = 0.0; // m/s (used for trajectory math)
    private double shooterAngle    = 70.0;
    private double phi             = Math.toRadians(shooterAngle);
    private double shooterHeight   = 0.508;

    /** Construct with the appropriate IO implementation (real or sim). */
    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    // -------------------------------------------------------------------------
    // Velocity / trajectory helpers
    // -------------------------------------------------------------------------

    public double velocityFromDistance(double x) {
        double y = 1.8288 - shooterHeight;
        double v = Math.sqrt((9.807 * x * x) /
                (2 * Math.cos(phi) * Math.cos(phi) * (x * Math.tan(phi) + shooterHeight - y)));
        return (1 + 0.015 * x) * 1.04 * v;
    }

    public double velocityFromDistance(double x, double robotZ) {
        double y = 1.8288 - shooterHeight - robotZ;
        double v = Math.sqrt((9.807 * x * x) /
                (2 * Math.cos(phi) * Math.cos(phi) * (x * Math.tan(phi) + shooterHeight - y)));
        return (1 + 0.015 * x) * 1.04 * v;
    }

    // -------------------------------------------------------------------------
    // Motor commands
    // -------------------------------------------------------------------------

    public void setVelocityTo(double newVelocity) {
        velocity = newVelocity;
        io.setShooterVelocity(convertToRPS(newVelocity));
    }

    public void startKickerMotors() {
        io.setKickerOutput(ShooterConstants.kickerMotorPower);
    }

    public void stopKickerMotors() {
        io.setKickerOutput(0.0);
    }

    public void stop() {
        velocity = 0.0;
        io.stop();
    }

    // -------------------------------------------------------------------------
    // State queries
    // -------------------------------------------------------------------------

    public double getVelocity() {
        return velocity;
    }

    public boolean atCorrectRPM() {
        double measuredRPS = inputs.shooterVelocityRPS;
        double targetRPS   = convertToRPS(velocity);
        return Math.abs(measuredRPS - targetRPS) < 1.0; // within 1 RPS
    }

    public boolean isShooting() {
        return Math.abs(velocity) > 0.01;
    }

    // -------------------------------------------------------------------------
    // Velocity correction
    // -------------------------------------------------------------------------

    public double[] correctVandYaw(double dx, double dy, double yaw, double vxField, double vyField) {
        double distance    = Math.hypot(dx, dy);
        double unitX       = dx / distance;
        double unitY       = dy / distance;
        double vStationary = velocityFromDistance(distance);
        double vHorizontal = vStationary * Math.cos(phi);

        double correctedVx = vHorizontal * unitX;
        double correctedVy = vHorizontal * unitY;

        if (Math.hypot(vxField, vyField) > 0.01) {
            correctedVx -= vxField;
            correctedVy -= vyField;

            double correctedYaw = Math.atan2(correctedVy, correctedVx);
            double correction   = MathUtil.angleModulus(correctedYaw - yaw);
            correction          = MathUtil.clamp(correction, -0.2, 0.2);
            yaw                += correction;
        }

        return new double[]{correctedVx, correctedVy, yaw};
    }

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /** Convert a linear ball speed (m/s) to shooter wheel rotations per second. */
    private double convertToRPS(double velocity) {
        double wheelRadius = 0.0508;
        return velocity / (2.0 * Math.PI * wheelRadius);
    }
}