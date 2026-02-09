// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.Debouncer;
//import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.*;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeRollers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralRollers;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.VisionSubsystem;


import edu.wpi.first.cameraserver.CameraServer;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double speedLimiter = 0.5;
    private double directionFlipper = -1.0;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 2% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    

    private final Telemetry logger = new Telemetry(MaxSpeed);
    //private PhotonCamera cam1 = new PhotonCamera("camera1_2585"); we dont need this

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController subjoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Elevator elevator = new Elevator();
    private final AlgaeRollers algaeroller = new AlgaeRollers();
    private final CoralRollers coralroller = new CoralRollers();
    private final Climb climb = new Climb();
    private final VisionSubsystem vision = new VisionSubsystem("meow");

    /* Path follower */
    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        CameraServer.startAutomaticCapture();
        // register commands to pathplanner
        
        /*
        NamedCommands.registerCommand("elevatorL2", new ElevatorPIDAutoL2(elevator, ElevatorConstants.l2));
        NamedCommands.registerCommand("elevatorL4Actual", new ElevatorPIDAuto(elevator, ElevatorConstants.l4));
        NamedCommands.registerCommand("elevatorL1", new ElevatorPIDAutoL1(elevator, ElevatorConstants.l1-0.5));
        NamedCommands.registerCommand("AlignReef", new AlignToReefTagRelative(drivetrain, true));
        NamedCommands.registerCommand("scoreOut", new CoralAutoScore(coralroller));
        NamedCommands.registerCommand("scoreOutFast", new CoralAutoScoreFast(coralroller));
        NamedCommands.registerCommand("preRoller", new CoralAutoPreroller(coralroller));
        NamedCommands.registerCommand("stopCoralRollers", new CoralAutoStop(coralroller));
        */

        NamedCommands.registerCommand("alignToReef", new PV_Align(drivetrain, vision));
        NamedCommands.registerCommand("faceAprilTag", new FaceAprilTagRelative(drivetrain, vision, xInput, yInput));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        
        
        joystick.x().onTrue(new PV_Align(drivetrain, vision));
        
        

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> 
                drive
                    .withVelocityX(joystick.getLeftY() * MaxSpeed * speedLimiter * directionFlipper)
                    .withVelocityY(joystick.getLeftX())
                    .withRotationalRate(joystick.getRightX() * MaxAngularRate)            
            )

        );
            SmartDashboard.putNumber("x", vision.getX());
            SmartDashboard.putNumber("y", vision.getY());
            SmartDashboard.putNumber("rot", vision.getRot());


        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double forward = joystick.getLeftY() * MaxSpeed * speedLimiter * directionFlipper;
                double translation = joystick.getLeftX() * MaxSpeed * speedLimiter * directionFlipper;
                double turn = joystick.getRightX() * MaxAngularRate * speedLimiter * directionFlipper * 2;

                if (joystick.a().getAsBoolean() && vision.hasTargets() && vision.getTargetID() == 18) {
                    double kp = 1.5;

                    turn = -kp * MaxAngularRate * Math.toRadians(vision.getTargetYaw());
                }
                return drive
                    .withVelocityX(forward)
                    .withVelocityY(translation)
                    .withRotationalRate(turn);
            })

        );
        joystick.b().whileTrue(new RunCommand(() -> this.flipDirection(1.0)));
        joystick.y().whileTrue(new RunCommand(() -> this.flipDirection(-1.0)));

        // ************************************************************************************
        // drivetrain controller | pov buttons | subsystem RunCommand()
        joystick.povUp().whileTrue(new RunCommand(() -> this.setSpeed(1.0)));
        joystick.povRight().whileTrue(new RunCommand(() -> this.setSpeed(0.400)));
        joystick.povLeft().whileTrue(new RunCommand(() -> this.setSpeed(0.200)));
        joystick.povDown().whileTrue(new RunCommand(() -> this.setSpeed(0.1)));
        // ************************************************************************************       
    }
    // changing drivetrain speed
    //   crawl, low, mid, high
    public void setSpeed(double spe){
        speedLimiter = spe;
        if(spe == 0.066) SmartDashboard.putString( "Swerve Speed", "CRAWL");
        if(spe == 0.2) SmartDashboard.putString("Swerve Speed", "LOW");
        if(spe == 0.5) SmartDashboard.putString("Swerve Speed", "MID");
        if(spe == 1.0) SmartDashboard.putString("Swerve Speed", "HIGH");
    }

    public void flipDirection(double newDir){
        this.directionFlipper = newDir;
    }
    //  setting the 
    

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        // return null;
        return autoChooser.getSelected();
    }
}
