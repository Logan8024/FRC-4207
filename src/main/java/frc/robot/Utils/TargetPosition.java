// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

/** Add your docs here. */
public class TargetPosition {
    public double elevatorPos;
    public String name;
    public double angleLeft;
    public double angleRight;
    public int targetID;

    public TargetPosition(int targetID, String name, double angleLeft, double angleRight, double ElevatorPosition) {
      this.elevatorPos = ElevatorPosition;
      this.name = name;
      this.angleLeft = angleLeft;
      this.angleRight = angleRight;
      this.targetID = targetID;
    }
}
