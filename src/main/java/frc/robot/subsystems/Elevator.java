package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConstantsOther;
import frc.robot.ConstantsOther.ElevatorConstants;
import frc.robot.Utils.TargetPosition;


public class Elevator extends SubsystemBase{

  /*We need methods to run the arm to postions using a rev throughbore encoder in absolute mode.
   * It is needed to read arm position and to move the arm.
   */
  private SparkMax m_Left;
  private SparkMax m_Right;
  private RelativeEncoder encoder;
  private SparkClosedLoopController elevatorPID;
  private SparkMaxConfig leftConfig;
  private SparkMaxConfig rightConfig;



  private final ArmFeedforward ff =
      new ArmFeedforward(
      ElevatorConstants.kSVolts, ElevatorConstants.kGVolts,
      ElevatorConstants.kVVoltSecondPerRad, ElevatorConstants.kAVoltSecondSquaredPerRad);

  public Elevator (int leftID, int rightID) {
    m_Left = new SparkMax(leftID, MotorType.kBrushless);
    m_Right = new SparkMax(rightID, MotorType.kBrushless);
    elevatorPID = m_Left.getClosedLoopController();
    leftConfig = new SparkMaxConfig();
    rightConfig = new SparkMaxConfig();
    encoder = m_Left.getEncoder();

    leftConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);


    leftConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.4)
        .i(0)
        .d(0)
        .outputRange(-.3, .3);

    rightConfig
    .follow(leftID);
    
  
    m_Left.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_Right.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void runOpenLoop(double supplier) {
      if(getPos() >= ElevatorConstants.kUpperLimit) {
          m_Left.set(supplier);
          System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
      }
      else if(getPos() <= ElevatorConstants.kLowerLimit) {
          m_Left.set(supplier);
          System.out.println("¡TOO LOW! ¡LOWER LIMIT!");
      }
      else {
          m_Left.set(supplier);
      }
  }


  public void hold(TrapezoidProfile.State setpoint) {
      double feedforward = ff.calculate(setpoint.position*2*Math.PI, setpoint.velocity);
      elevatorPID.setReference(setpoint.position, ControlType.kPosition);
  }

  public void holdCurrent() {
    runToPosition(new TrapezoidProfile.State(getPos(),0.0));
  }

  public void runToPosition(TrapezoidProfile.State setpoint) {
      //these are included safety measures. not necessary, but useful
      /*if(getPos() >= ElevatorConstants.kUpperLimit) {
          m_Left.set(0);
          System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
      }
      else if(getPos() <= ElevatorConstants.kLowerLimit) {
          m_Left.set(0);
          System.out.println("¡TOO LOW! ¡LOWER LIMIT! " + getPos());
      }*/
      //else{
          //double feedforward = ff.calculate(setpoint.position*2*Math.PI, setpoint.velocity);
          elevatorPID.setReference(setpoint.position, ControlType.kPosition);
          SmartDashboard.putNumber("position trying", setpoint.position);
      //}
  }
  public void runToPosition(double setpoint) {
    //these are included safety measures. not necessary, but useful
    /*if(getPos() >= ElevatorConstants.kUpperLimit) {
        m_Left.set(0);
        System.out.println("¡TOO HIGH! ¡UPPER LIMIT!");
    }
    else if(getPos() <= ElevatorConstants.kLowerLimit) {
        m_Left.set(0);
        System.out.println("¡TOO LOW! ¡LOWER LIMIT! " + getPos());
    }*/
    //else{
        //double feedforward = ff.calculate(setpoint.position*2*Math.PI, setpoint.velocity);
        elevatorPID.setReference(setpoint, ControlType.kPosition);
        SmartDashboard.putNumber("position trying", setpoint);
    //}
}

  //test function
 

  public double getPos() {
      return encoder.getPosition();
  }

  public double[] getTemp() {
     double result[] = {m_Left.getMotorTemperature(), m_Right.getMotorTemperature()};
     return result;
  }
  public double[] getCurrent() {
      double result[] = {m_Left.getOutputCurrent(), m_Right.getOutputCurrent()};
      return result;
  }
  public void bumpUp() {
    runToPosition(new TrapezoidProfile.State(-4,0.0));
//    runToPosition(new TrapezoidProfile.State(getPos()+.1,0.0));
  }
  public void bumpDown() {
    runToPosition(new TrapezoidProfile.State(getPos() - .1,0.0));
  }
  @Override
  public void periodic() {
      // TODO Auto-generated method stub
      super.periodic();
      SmartDashboard.putNumber("Elevator Position", getPos());
      System.out.println("Elevator Curent " + getCurrent()[0]);
    }
}