// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.ConstantsOther;
import frc.robot.ConstantsOther.ArmConstants;
import frc.robot.ConstantsOther.ElevatorConstants;
import frc.robot.Utils.TargetPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Elevator_Arm_Movement extends ParallelCommandGroup {
  /** Creates a new Elevator_Arm_Movement. */
  Elevator mElevator = new Elevator(ElevatorConstants.leftID, ElevatorConstants.rightID);
  Arm mArm = new Arm(ArmConstants.armId);
  public Elevator_Arm_Movement(TargetPosition position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (ConstantsOther.side == "left") {
      addCommands(
      new InstantCommand(()->mElevator.runToPosition(position.elevatorPos)),
      new InstantCommand(()->mArm.runToPosition(position.angleLeft))
    );
    }
    else {
      addCommands(
      new InstantCommand(()->mElevator.runToPosition(position.elevatorPos)),
      new InstantCommand(()->mArm.runToPosition(position.angleRight))
    );
    }
    
  }
}
