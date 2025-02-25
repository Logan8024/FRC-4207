// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class RunElevatorOpenLoop extends Command {
  private Elevator elevator;
  private double supplier;
  /** Creates a new RunArmOpenLoop. */
  public RunElevatorOpenLoop(Elevator elev, double input) {
    supplier = input;
    elevator = elev;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elev);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("CMD init - running arm");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.runOpenLoop(supplier);
    System.out.println("¡ARM! ¡EXCERCISE CAUTION!");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}