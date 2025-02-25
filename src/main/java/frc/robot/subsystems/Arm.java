// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.ConstantsOther;
import frc.robot.ConstantsOther.ArmConstants;

public class Arm extends SubsystemBase{

    /*We need methods to run the arm to postions using a rev throughbore encoder in absolute mode.
     * It is needed to read arm position and to move the arm.
     */
    public SparkMax arm;
    public SparkClosedLoopController armPID;
    public AbsoluteEncoder armEncoder;
    private SparkMaxConfig motorConfig;


    double curTargetSetPoint = 0;
    private final ArmFeedforward ff =
        new ArmFeedforward(
        ArmConstants.kSVolts, ArmConstants.kGVolts,
        ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

    public Arm (int armID) { 
    arm = new SparkMax(armID, MotorType.kBrushless);
    armPID = arm.getClosedLoopController();
    armEncoder = arm.getAbsoluteEncoder();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();

    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig.absoluteEncoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0)
        .i(0)
        .d(0)
        .outputRange(-.2, .2);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
        arm.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void runOpenLoop(double supplier) {
        System.out.println("GOT HERE IN run open loop==========================");
        if(getPos() >= ArmConstants.kUpperLimit) {
            arm.set(supplier);
            System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
        }
        else if(getPos() <= ArmConstants.kLowerLimit) {
            arm.set(supplier);
            System.out.println("¡TOO LOW! ¡LOWER LIMIT!");
        }
        else {
            arm.set(supplier);
        }
    }

  
    public void hold(TrapezoidProfile.State setpoint) {
        System.out.println("GOT HERE IN HOLD==========================");
        double feedforward = ff.calculate(setpoint.position*2*Math.PI, setpoint.velocity);
        armPID.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0/*, feedforward*/);
        curTargetSetPoint = getPos();
    }
//double speed = 0;
    public void bumpUp() {
  //      speed+=.1;
    //    arm.set(speed);
      //  System.out.println(speed);
        runToPosition(new TrapezoidProfile.State(getPos()+.1,0.0));
       // runToPosition(new TrapezoidProfile.State(curTargetSetPoint+.01,0));
}
    public void bumpDown() {
        //arm.set(0);
        runToPosition(new TrapezoidProfile.State(getPos() - .1,0.0));
       // runToPosition(new TrapezoidProfile.State(curTargetSetPoint-.01,0));
    }

    public void runToPosition(TrapezoidProfile.State setpoint) {
        System.out.println("GOT HERE IN RUN TO POSITION = " + setpoint.position + " " + arm.getAbsoluteEncoder().getPosition());
        //these are included safety measures. not necessary, but useful
        if(getPos() >= ArmConstants.kUpperLimit) {
            arm.set(0);
            System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
        }
        else if(getPos() <= ArmConstants.kLowerLimit) {
            arm.set(0);
            System.out.println("¡TOO LOW! ¡LOWER LIMIT! " + getPos());
        } 
        else{
            //double feedforward = ff.calculate(setpoint.position*2*Math.PI, setpoint.velocity);
            armPID.setReference(setpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0/*, feedforward*/);
            curTargetSetPoint = setpoint.position;
            
        }
    }

    public void runToPosition(double setpoint) {
        System.out.println("GOT HERE IN RUN TO POSITION = " + setpoint + " " + arm.getAbsoluteEncoder().getPosition());
        //these are included safety measures. not necessary, but useful
        if(getPos() >= ArmConstants.kUpperLimit) {
            arm.set(0);
            System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
        }
        else if(getPos() <= ArmConstants.kLowerLimit) {
            arm.set(0);
            System.out.println("¡TOO LOW! ¡LOWER LIMIT! " + getPos());
        } 
        else{
            //double feedforward = ff.calculate(setpoint.position*2*Math.PI, setpoint.velocity);
            armPID.setReference(setpoint, ControlType.kPosition, ClosedLoopSlot.kSlot0/*, feedforward*/);
            curTargetSetPoint = setpoint;
            
        }
    }

    public double getPos() {
        return armEncoder.getPosition();
    }

    public double[] getTemp() {
       double result[] = {arm.getMotorTemperature()};
       return result;
    }
    
    public double[] getCurrent() {
        double result[] = {arm.getOutputCurrent()};
        return result;
    }

    public static void toggleSide() {
        if (ConstantsOther.side == "left") {
            ConstantsOther.side = "right";
        }
        else {
            ConstantsOther.side = "left";
        }
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        SmartDashboard.putNumber("Arm Position", getPos());
        System.out.println("Curent " + getCurrent()[0]);
        SmartDashboard.putNumber("Output", arm.getAppliedOutput());
     //   SmartDashboard.putString("hi", armPID.);
    }
}