// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.ConstantsOther.ArmConstants;
import frc.robot.ConstantsOther.ElevatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;




public class RobotContainer {

    private final Boolean DriveModeRobotCentric = true;
    private final Boolean DriveModeFieldCentric = true;


    //start swerve added stuff----------------------------------------------------
    private final double MAXSPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity




    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MAXSPEED * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MAXSPEED * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MAXSPEED);

    private final CommandXboxController joystickDriver = new CommandXboxController(0);
    private final CommandXboxController joystickAssistant = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    // end swerve added stuff

    Climber mClimber =  null; //new Climber();
    Elevator mElevator = null;//new Elevator(ElevatorConstants.leftID, ElevatorConstants.rightID);
    Arm mArm = new Arm(ArmConstants.armId);





    public RobotContainer() {
        configureBindings();
        SignalLogger.enableAutoLogging(false);
    }

    private void configureBindings() {
        System.out.println("testing!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1");
        //begin swerve added stuff-----------------------------------------------
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                fieldCentricDrive.withVelocityX(-joystickDriver.getLeftY() * MAXSPEED) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystickDriver.getLeftX() * MAXSPEED) // Drive left with negative X (left)
                    .withRotationalRate(-joystickDriver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystickDriver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystickDriver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystickDriver.getLeftY(), -joystickDriver.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
     /*    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
*/
        // reset the field-centric heading on left bumper press
        joystickDriver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        //end swerve added stuff
        joystickDriver.rightBumper().onTrue(new InstantCommand(()->setDriveMode(ConstantsOther.DriveMode.ROBOTCENTRIC)));
        joystickDriver.rightBumper().onFalse(new InstantCommand(()->setDriveMode(ConstantsOther.DriveMode.FIELDCENTRIC)));

        if(mClimber != null) {
            //POV 0 = up, 90=right, 180=down, 270=left 
            joystickDriver.leftBumper().onTrue(new InstantCommand(()->mClimber.leftUp()));
            joystickDriver.leftBumper().onFalse(new InstantCommand(()->mClimber.stop()));

            joystickDriver.leftTrigger().onTrue(new InstantCommand(()->mClimber.leftDown()));
            joystickDriver.leftTrigger().onFalse(new InstantCommand(()->mClimber.stop()));

            joystickDriver.rightBumper().onTrue(new InstantCommand(()->mClimber.rightUp()));
            joystickDriver.rightBumper().onFalse(new InstantCommand(()->mClimber.stop()));

            joystickDriver.rightTrigger().onTrue(new InstantCommand(()->mClimber.rightDown()));
            joystickDriver.rightTrigger().onFalse(new InstantCommand(()->mClimber.stop()));

//            joystick.leftBumper().onFalse(new InstantCommand(()->mClimber.stop()));
  //          joystick.leftBumper().onFalse(new InstantCommand(()->mClimber.stop()));
//                   new JoystickButton(driver, 4).onTrue(new InstantCommand(() -> mClimber.up()));
  //               new JoystickButton(driver, 5).onTrue(new InstantCommand(() -> mClimber.down()));
        }
        if(mElevator != null) {
            joystickDriver.leftTrigger().onTrue(new InstantCommand(()->mElevator.bumpUp()));
            joystickDriver.rightTrigger().onTrue(new InstantCommand(()->mElevator.bumpDown()));
        }

        if(mArm != null) {
            
            joystickDriver.povDown().onTrue(new InstantCommand(()->mArm.runToPosition(SmartDashboard.getNumber("arm pos", MAXSPEED))));
            joystickDriver.povLeft().onTrue(new InstantCommand(()->mArm.runToPosition(ArmConstants.Arm_Level1)));
            joystickDriver.povUp().onTrue(new InstantCommand(()->mArm.runToPosition(ArmConstants.Arm_Level2)));  
            joystickDriver.povRight().onTrue(new InstantCommand(()->mArm.runToPosition(ArmConstants.Arm_Level3)));
            joystickDriver.leftTrigger().onTrue(new InstantCommand(()->mArm.bumpUp()));
            joystickDriver.rightTrigger().onTrue(new InstantCommand(()->mArm.bumpDown()));
        }

 

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void setDriveMode(int driveMode) {
        if (driveMode == ConstantsOther.DriveMode.ROBOTCENTRIC) {
            drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                    robotCentricDrive.withVelocityX(-joystickDriver.getLeftY() * MAXSPEED) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystickDriver.getLeftX() * MAXSPEED) // Drive left with negative X (left)
                    .withRotationalRate(-joystickDriver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
            );
            SmartDashboard.putString("DriveMode","RobotCentric");
        }
        else {   // driveMode == ConstantsOther.DriveMode.FIELDCENTRIC;
            drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                fieldCentricDrive.withVelocityX(-joystickDriver.getLeftY() * MAXSPEED) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystickDriver.getLeftX() * MAXSPEED) // Drive left with negative X (left)
                    .withRotationalRate(-joystickDriver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
            );
            SmartDashboard.putString("DriveMode","FieldCentric");
        }
    }
}
