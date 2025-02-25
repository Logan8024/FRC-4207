package frc.robot;

import javax.sound.sampled.TargetDataLine;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utils.TargetPosition;

public final class ConstantsOther {
    public static final int ERROR_FLAGINT = -4207;
    public static final double ERROR_FLAGDOUBLE= -4207.0;
    
    //motor constants
   // public static final int MOTOR_INTAKE_FRONTROLLER = 36;//16;
    public static final int MOTOR_CLIMBER_LEFT = 13;
    public static final int MOTOR_CLIMBER_RIGHT = 26;
    public static final int MOTOR_ARM_ROTATE = -1;
    public static final int MOTOR_ELEVATOR_LEFT = 1;
    public static final int MOTOR_ELEVATOR_RIGHT = 5;

    public static String side = "left";


    
    public final class DriveMode {
        public static final int ROBOTCENTRIC=0;
        public static final int FIELDCENTRIC=1;
    }

    public static final class Climber {
        public static final int angleServo = 75;
    }

    

    //Elevator Constants
    public static final class ElevatorConstants {


        public static final int leftID = 5;
        public static final int rightID = 1;

        public static final int Start = 0;
        public static final int Intake = 0;
        public static final int Level1 = 0;
        public static final int Level2 = 0;
        public static final int Level3 = 0;
        public static final int Level4 = 0;

        public static final TargetPosition[] target = {
        new TargetPosition(Start, "Start", 0, 0, 0),
        new TargetPosition(Intake, "Intake", 0, 0, 0),
        new TargetPosition(Level1, "Level1", 0, 0, 0),
        new TargetPosition(Level2, "Level2", 0, 0, 0),
        new TargetPosition(Level3, "Level3", 0, 0, 0),
        new TargetPosition(Level4, "Level4", 0, 0, 0)
        };
        

        public static final TrapezoidProfile.State Elevator_Start = new TrapezoidProfile.State(20,0.05);
        public static final TrapezoidProfile.State Elevator_Intake = new TrapezoidProfile.State(5,0.05);
        public static final TrapezoidProfile.State Elevator_Level1 = new TrapezoidProfile.State(5,0.05);
        public static final TrapezoidProfile.State Elevator_Level2 = new TrapezoidProfile.State(5,0.05);
        public static final TrapezoidProfile.State Elevator_Level3 = new TrapezoidProfile.State(5,0.05);
        public static final TrapezoidProfile.State Elevator_Level4 = new TrapezoidProfile.State(5,0.05);
        
        //  public static final int kLeftID = 6;
        //  public static final int kRightID = 7;
        //  public static final double kStowPos = 0.26;
        public static final double kDefaultPos = 0.032;
        public static final double k3mPos = 0.064;

        public static final double kShuttlePos = 0.07;
        public static final double kUpperLimit = 0.35;
        public static final double kLowerLimit = 0.023;
        public static final double kOverrunLimit = 0;

        public static final double kP = 0.01; 
        public static final double kI = 0;
        public static final double kD = 0; 
        public static final double kIz = 0; 
        public static final double kFF = 0; 
        public static final double kMaxOutput = 0.7; 
        public static final double kMinOutput = -0.7;
        public static final double kMaxAccel = 0.18;
        public static final double kMaxVel = 0.85;

        public static final double kTolearance = 0.002;
        public static final double kManualSpeed = 0.3;

        public static final double kSVolts = 0;
        public static final double kGVolts = 0.33;
        public static final double kVVoltSecondPerRad = 6.24;
        public static final double kAVoltSecondSquaredPerRad = 0.04;

    }
    
    public static final class ArmConstants {
        //motor
        public static final int armId = 2;

        //Positions
        public static  TrapezoidProfile.State Arm_Start = new TrapezoidProfile.State(.5,0);
        public static  TrapezoidProfile.State Arm_Intake = new TrapezoidProfile.State(1,0);
        public static  TrapezoidProfile.State Arm_Level1 = new TrapezoidProfile.State(1.5,0);
        public static  TrapezoidProfile.State Arm_Level2 = new TrapezoidProfile.State(2,0);
        public static  TrapezoidProfile.State Arm_Level3 = new TrapezoidProfile.State(2.5,0);
        public static final TrapezoidProfile.State Arm_Level4 = new TrapezoidProfile.State(3,0);

      //  public static final int kLeftID = 6;
     //   public static final int kRightID = 7;
    //    public static final double kStowPos = 0.26;
        public static final double kDefaultPos = 0.6;
        // intake go lower
    //    public static final double kIntakePos = 0.0232;
    //    public static final double kSubwooferPos = 0.037;
     //   public static final double kAnglePos = 0.06;
    
     //   public static final double kFrontAmpPos = 0.18;
    
        // 3m position
    //    public static final double k3mPos = 0.064;
    
        public static final double kShuttlePos = 0.07;
     //   public static final double kBackAmpPos = 0.28;
        public static final double kUpperLimit = 1;
        public static final double kLowerLimit = 0;
        public static final double kOverrunLimit = 0;
    
        public static final double kP = 6; 
        public static final double kI = 0;
        public static final double kD = 0; 
        public static final double kIz = 0; 
        public static final double kFF = 0; 
        public static final double kMaxOutput =  0.7; 
        public static final double kMinOutput = -0.7;
       // public static final double kMaxAccel = 0.18;
       // public static final double kMaxVel = 0.85;
    
        public static final double kTolearance = 0.002;
//        public static final double kManualSpeed = 0.3;
    
        public static final double kSVolts = 0.05;
        public static final double kGVolts = 0.33;
        public static final double kVVoltSecondPerRad = 6.24;
        public static final double kAVoltSecondSquaredPerRad = 0.04;
    }
    
    //driver joystick



    //  Buttons  X style   triggers as axis
    //   1 = A
    //   2 = B
    //   3 = X
    //   4 = Y
    //   5 = LB
    //   6 = RB
    //   7 = Back
    //   8 = Start
    //   9 = Left stick Down
    //   10 = Right Stick Down


    //  Buttons  D  (dual action)   triggers are buttons
    //   1 = X
    //   2 = A
    //   3 = B
    //   4 = Y
    //   5 = LB
    //   6 = RB
    //   7 = Left Trigger
    //   8 = RIght Trigger
    //   9 = Back
    //   10 = Start

/*     public static int POV_SPEAKER_MODE = 0;
    public static int POV_PODIUM_MODE = 270;
    public static int POV_AMP_MODE =180;
    public static int POV_INTAKE_MODE =90;

    public static int POV_CLIMBER_MODE = 180;
    public static int POV_PODIUM_OFFSET = 0;
    public static int POV_MID_COURT = 90;

//    public static final int BTN_SPEAKER_SHOOT = 4;
//    public static final int BTN_AMP_SHOOT = 5;
//    public static final int BTN_PODIUM_SHOOT = 6;
    

/*
 * 
    new JoystickButton(driver, 3).whileTrue(new AprilStrafe(s_Swerve, m_Photon,9));
        new JoystickButton(driver, 4).onTrue(new InstantCommand(() -> m_Shooter.SpeakerShoot()));
        new JoystickButton(driver, 5).onTrue(new InstantCommand(() -> m_Shooter.AmpShoot()));
        new JoystickButton(driver, 6).onTrue(new InstantCommand(() -> m_Shooter.PodiumShoot()));

 */


}