package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;

public class Robot extends TimedRobot {
  public static RobotContainer m_robotContainer;
  public static Command AutonomousCommand;
  public static DriveTrain DriveTrain;
  

  @Override
  public void robotInit() {
    log("Robot Init");
    m_robotContainer = new RobotContainer();
    AutonomousCommand = m_robotContainer.getAutonomousCommand();
    DriveTrain = new DriveTrain();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    log("Robot Disabled");
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    log("Robot Autonomous");
    AutonomousCommand = m_robotContainer.getAutonomousCommand();
    if (AutonomousCommand != null) {
      AutonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    log("Robot Teleop");
    if (AutonomousCommand != null) {
      AutonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    log("Robot Test");
    CommandScheduler.getInstance().cancelAll();
    // Home all motors and sensors
    // spin up shooter
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
    log("Robot Simulation");
  }

  @Override
  public void simulationPeriodic() {
  }

  // ============================ //

  private void log(String msg) {
    System.out.println(msg);
  }
}
