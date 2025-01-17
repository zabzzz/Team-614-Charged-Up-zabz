package frc.robot.commands.Autonomous.TimedBasedAuto.Timed;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import frc.robot.RobotContainer;

public class TimedTilt extends CommandBase{

  Timer TimedTiltTimer = null;
  double localRunTime;

    public double tiltSpeed;
    public TimedTilt (double tiltspeed, double runtime) {
        addRequirements(RobotContainer.elevatorSubsystem);

        TimedTiltTimer = new Timer();

        tiltSpeed = tiltspeed;
        localRunTime = runtime;
    }
    @Override
  public void initialize() {
    TimedTiltTimer.reset();
    TimedTiltTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (TimedTiltTimer.get() <= localRunTime)
    RobotContainer.elevatorSubsystem.set(tiltSpeed);
  }
  @Override
  public void end(boolean interrupted) {
    RobotContainer.elevatorSubsystem.set(Constants.MOTOR_ZERO_SPEED);
    TimedTiltTimer.stop();
    TimedTiltTimer.reset();
  }
  //gets returned true when the command ends
  @Override
  public boolean isFinished() {
    return TimedTiltTimer.get() <= localRunTime;
  }
}
