package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.SyncedCommands.Controllers;
import frc.robot.SyncedCommands.Controllers.ControllerBase;
import frc.robot.subsystems.DriveTrain;

public class Robot extends TimedRobot {
  public static RobotContainer m_robotContainer;
  public static Command AutonomousCommand;
  public static DriveTrain DriveTrain;

  private static Controllers m_controllers = new Controllers(Constants.DriveConstants.controllerJoystickDeadband, Constants.DriveConstants.controllerTriggerDeadband);
  /** Driver */
  public static ControllerBase Zero = m_controllers.Zero;
  /** Copiolot */
  public static ControllerBase One = m_controllers.One;
  /** One-man show */
  public static ControllerBase Two = m_controllers.Two;
  /** Guest controller */
  public static ControllerBase Three = m_controllers.Three;
  /** UNUSED */
  public static ControllerBase Four = m_controllers.Four;
  /** UNUSED */
  public static ControllerBase Five = m_controllers.Five;

  /** Person controlling driving */
  public static ControllerBase Primary = m_controllers.Primary;
  /** Person controlling shooting */
  public static ControllerBase Secondary = m_controllers.Secondary;

  @Override
  public void robotInit() {
    System.out.println("Robot Init");

    m_robotContainer = new RobotContainer();
    AutonomousCommand = m_robotContainer.getAutonomousCommand();
    DriveTrain = new DriveTrain();

    m_controllers.fullUpdate();
    Three.setJoystickMultiplier(0.5);
    m_controllers.addControllers(m_controllers.primaryControllerSelector, Zero, Two, Three);
    m_controllers.addControllers(m_controllers.secondaryControllerSelector, One, Two, Three);
    m_controllers.fullUpdate();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_controllers.updateAutoControllers();
  }

  @Override
  public void disabledInit() {
    System.out.println("Robot Disabled");
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    System.out.println("Robot Autonomous");

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
    System.out.println("Robot Teleop");

    m_controllers.fullUpdate();
    if (AutonomousCommand != null) {
      AutonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    System.out.println("Robot Test");

    CommandScheduler.getInstance().cancelAll();
    m_controllers.fullUpdate();
    Robot.DriveTrain.resetAll();
    // Home all motors and sensors
    // spin up shooter
    // turn on intake
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
    System.out.println("Robot Simulation");

    m_controllers.fullUpdate();
  }

  @Override
  public void simulationPeriodic() {
  }
}