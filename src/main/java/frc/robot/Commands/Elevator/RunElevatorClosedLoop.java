// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ConstantsOther.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.Utils.ToolKit;;

public class RunElevatorClosedLoop extends Command {
  private Elevator Elevator;
  private double setpoint;
  private boolean end;
  private TrapezoidProfile.State state;
  /** Creates a new RunElevatorClosedLoop. */
  public RunElevatorClosedLoop(Elevator ar, double target) {
    Elevator = ar;
    setpoint = target;
    end = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("CMD init - running Elevator");
    end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ToolKit.isInTolarance(Elevator.getPos(), setpoint, ElevatorConstants.kTolearance)) {
      System.out.println("Elevator At Setpoint");
      end = true;
    }
    else {
      state = new State(setpoint, 0);
      Elevator.runToPosition(state);
      end = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}