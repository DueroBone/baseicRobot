package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class TeleDriveCommand extends Command {
  public TeleDriveCommand() {
    addRequirements(Robot.DriveTrain);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    Robot.DriveTrain.doTankDrive(Robot.Primary.getLeftY(), Robot.Primary.getRightY());
  }

  @Override
  public void end(boolean interrupted) {
    Robot.DriveTrain.stop();
  }

  @Override
  public boolean isFinished() {
    return DriverStation.isDisabled();
  }
}
