// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CANdleSystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralHandlerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;




public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)*.5; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //controllers here
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController mechController = new CommandXboxController(1);


    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final CANdleSystem candle;
    private final CoralHandlerSubsystem coralHandler = new CoralHandlerSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(coralHandler);
    private final AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
    
    //Autonomous
    private final SendableChooser<Command> autoChooser;
    private final Command autonomousCommand = new PathPlannerAuto("AutoTest1");


    public RobotContainer() {

        candle = new CANdleSystem();
        
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();

        
        SmartDashboard.putData("Auto Chooser", autoChooser);

        //System.out.println("🔍 Available Paths: " + PathPlannerPath.getAllPathNames());

        SmartDashboard.putData("Run Auto", new InstantCommand(() -> {
            CommandScheduler.getInstance().schedule(getAutonomousCommand());
        }));


    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driveController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))
        ));

        //Add CANdle control: change LED animation when 'X' is pressed
        driveController.x().onTrue(Commands.runOnce(() -> candle.incrementAnimation()));


        // Run SysId routines when holding back/start and X/Y
        // Note that each routine should be run exactly once in a single log.
        driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        

        drivetrain.registerTelemetry(logger::telemeterize);

        //Coral
        mechController.a()
            .whileTrue(Commands.run(() -> coralHandler.startIntake()))
            .onFalse(Commands.run(() -> coralHandler.stopCoralHandler()));
        
        mechController.b()
            .whileTrue(Commands.run(() -> {
                coralHandler.startOuttake();
                algaeSubsystem.runAlgae();
            }))
            .onFalse(Commands.run(() -> {
                coralHandler.stopCoralHandler();
                algaeSubsystem.stopAlgae();
            }));

        mechController.y()
            .whileTrue(Commands.run(() -> algaeSubsystem.ejectAlgae()))
            .onFalse(Commands.run(() -> algaeSubsystem.stopEject()));

            
        // Manual Override: Hold Start + Back for 3 sec
         mechController.start()
         .and(mechController.back())
         .debounce(3.0) // Hold both buttons for 3 seconds
         .onTrue(new InstantCommand(() -> coralHandler.activateManualOverride()));
        
        mechController.leftBumper().onTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorPosition(ElevatorSubsystem.INTAKE_POSITION)));
        mechController.rightBumper().onTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorPosition(ElevatorSubsystem.L2_POSITION)));
        mechController.leftTrigger().onTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorPosition(ElevatorSubsystem.L3_POSITION)));
        mechController.rightTrigger().onTrue(new InstantCommand(() -> elevatorSubsystem.setElevatorPosition(ElevatorSubsystem.ALGAE_SCORE_POSITION)));
        
    }

    // Make sure CANdle updates every cycle
    public void periodic() {
        candle.periodic();
        
    }
    public Command getAutonomousCommand() {
        System.out.println("Running Autnomous Path: AutoTest1");
        return autonomousCommand;
        
        //return autoChooser.getSelected();
        //return Commands.print("No autonomous command configured");
    }

    //Expose CANdleSystem for use in Robot.java
    public CANdleSystem getCaNdleSystem() {
        return candle;
    }
}
