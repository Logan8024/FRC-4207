package frc.robot.Commands.Arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class HoldArm extends Command {
  private final Arm arm;
  private double pos;
  private TrapezoidProfile.State state;
  /** Creates a new HoldArm. */
  public HoldArm(Arm ar) {
    arm = ar;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pos = arm.getPos();
    state = new State(pos, 0);
    System.out.println("holding arm");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.hold(state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Hold interupt");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}