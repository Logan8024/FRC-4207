package frc.robot.Commands.Elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class HoldElevator extends Command {
  private final Elevator elevator;
  private double pos;
  private TrapezoidProfile.State state;
  /** Creates a new HoldArm. */
  public HoldElevator(Elevator elev) {
    elevator = elev;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elev);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pos = elevator.getPos();
    state = new State(pos, 0);
    System.out.println("holding arm");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.hold(state);
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