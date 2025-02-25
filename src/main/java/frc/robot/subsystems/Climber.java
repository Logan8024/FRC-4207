// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConstantsOther;

public class Climber extends SubsystemBase {
  private TalonFX leftMotor = new TalonFX(ConstantsOther.MOTOR_CLIMBER_LEFT);
  private TalonFX rightMotor = new TalonFX(ConstantsOther.MOTOR_CLIMBER_RIGHT);

  //private Servo s_Right = new Servo(1);
  //private Servo s_Left = new Servo(2);

  private final double speedUp = .25;
  private final double speedDown = -.25;

  public Climber() {
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
    
    var currentConfigs = new MotorOutputConfigs();
            // The left motor is CCW+  ?????? this should maybe be right motor.
            currentConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
            leftMotor.getConfigurator().apply(currentConfigs);

  }

  /*public void lock() {
    s_Left.setAngle(ConstantsOther.Climber.angleServo);
    s_Right.setAngle(ConstantsOther.Climber.angleServo);
  }
*/
  public void leftUp() {
    leftMotor.set(speedUp);
  }
  public void leftDown() {
    leftMotor.set(speedDown);
  }
  public void rightUp() {
    rightMotor.set(speedUp);
  }
  public void rightDown() {
    rightMotor.set(speedDown);
  }


  public void up() {
    leftMotor.set(speedUp);
    rightMotor.set(speedUp);
  }
  public void down() {
    leftMotor.set(speedDown);
    rightMotor.set(speedDown);
  }
  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
