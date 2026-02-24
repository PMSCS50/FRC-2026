// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ElevatorConstants{
        public static final int elevator1CanID = 41;
        public static final int elevator2CanID = 42;
    
        public static final double rollerSpeed = 0.5;
        public static final double elevatorSpeed = 0.2;
    
        public static final double elevatorLowStop = 0;
        public static final double elevatorHighStop = 120;
    
        public static final double eP = 0.035;
        public static final double eI = 0.000;
        public static final double eD = 0.0;

        //public static final double loweredPos = 0;
        public static final double l1 = -0.5;
        public static final double l2 = 7.2;
        public static final double l3 = 18;
        public static final double l4 = 32.00;
        public static final double l3_5 = 24.17;
        public static final double l2_5 = 14.85;
      }

      public static final class IntakeConstants {
        public static final int viagraMotorCanId = 1;
        public static final int intakeMotorCanId = 2;
        public static final double viagraPower = 0.1;
        public static final double intakePower = 0.4;
      }

      public static final class ClimbConstants {
        public static final int climbMotorCanId = 3;
        public static final double climbSpeed = 0;
        public static final double climbMax = 0;
      }

      public static final class ShooterConstants {
        public static final int shooterMotorCanId = 4;
        public static final int kickerMotorCanId = 5;

        public static final double kickerMotorPower = 0.59;
        /*
        public static final int coralRoller2CanId = 22;
    
        public static final double chuteSpeed = .7;
        public static final double rollerSlowSpeed = 0.175;
        public static final double rollerFastSpeed = 0.4;
        public static final double rollerBack = -0.2;
        */
      }

      public static final class VisionConstants{
        public static final Pose3d cameraToRobot = new Pose3d(0.0,0.0,0.0, new Rotation3d(0.0,0.0,0.0));
        public static final double distanceToTag = 1;
        public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

      }

      public static final double BUCKET_AIM_TOLERANCE_DEG = 0.1;

      public static final double X_REEF_ALIGNMENT_P = 2;
      public static final double Y_REEF_ALIGNMENT_P = 3;
      public static final double ROT_REEF_ALIGNMENT_P = 0.1625;
    
      public static final double ROT_SETPOINT_REEF_ALIGNMENT = -1.0;  // Rotation
      public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0.25;
      public static final double X_SETPOINT_REEF_ALIGNMENT_L4 = -0.47;  // Vertical 
      public static final double X_SETPOINT_REEF_ALIGNMENT = -0.41;
      public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.005;
      public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.20;  // Horizontal pose
      public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.0025;
    
      public static final double DONT_SEE_TAG_WAIT_TIME = 0.5;
      public static final double POSE_VALIDATION_TIME = 1.0;
}
