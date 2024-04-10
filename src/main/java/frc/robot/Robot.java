package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.SyncedLibraries.AutoControllerSelector;
import frc.robot.SyncedLibraries.BasicFunctions;
import frc.robot.SyncedLibraries.Controllers;
import frc.robot.SyncedLibraries.Controllers.ControllerBase;
import frc.robot.SyncedLibraries.SystemBases.DriveTrainBase;
import frc.robot.SyncedLibraries.SystemBases.LedBase;
import frc.robot.SyncedLibraries.SystemBases.LimelightBase;
import frc.robot.SyncedLibraries.SystemBases.SwerveDriveBase;
import frc.robot.SyncedLibraries.SystemBases.TeleDriveCommandBase;
import frc.robot.SyncedLibraries.RobotState.*;

public class Robot extends TimedRobot {
  public static RobotContainer m_robotContainer;
  public static Command AutonomousCommand;
  public static DriveTrainBase DriveTrain;
  public static SwerveDriveBase SwerveDrive;
  public static LimelightBase Limelight;

  /**
   * The current state of the robot
   * <p>
   * KEEP UPDATED
   */
  public static RobotStateEnum robotState = RobotStateEnum.Disabled;
  public static ManipulatorStateEnum manipulatorState = ManipulatorStateEnum.Empty;

  private static Controllers m_controllers = new Controllers(Constants.DriveConstants.controllerJoystickDeadband,
      Constants.DriveConstants.controllerTriggerDeadband);
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
  public static ControllerBase Primary = Zero;
  /** Person controlling shooting */
  // public static ControllerBase Secondary = m_controllers.Secondary;

  LedBase leds;
  public static TeleDriveCommandBase teleDriveCommandBase;

  PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);

  private int counter = 0;

  @Override
  public void robotInit() {
    System.out.println("Robot Init");

    m_controllers.fullUpdate();
    // Three.setJoystickMultiplier(0.5);
    // m_controllers.addControllers(m_controllers.primaryControllerSelector, Zero,
    // Two, Three);
    // m_controllers.addControllers(m_controllers.secondaryControllerSelector, One,
    // Two, Three);
    m_controllers.fullUpdate();

    m_robotContainer = new RobotContainer();
    AutonomousCommand = m_robotContainer.getAutonomousCommand();
    /*
     * DriveTrain = new DriveTrainBase(new CANSparkMax[] {
     * new CANSparkMax(1, MotorType.kBrushless),
     * new CANSparkMax(2, MotorType.kBrushless) },
     * new CANSparkMax[] {
     * new CANSparkMax(3, MotorType.kBrushless),
     * new CANSparkMax(4, MotorType.kBrushless) },
     * null, new int[0], false, 1, 0, 0, kDefaultPeriod,
     * kDefaultPeriod, true);
     */
    SwerveDrive = new SwerveDriveBase(
        // turnMotors
        new CANSparkMax[] {
            new CANSparkMax(1, MotorType.kBrushless),
            new CANSparkMax(2, MotorType.kBrushless),
            new CANSparkMax(3, MotorType.kBrushless),
            new CANSparkMax(4, MotorType.kBrushless) },
        // absoluteEncoders
        new CANcoder[] {
            new CANcoder(1),
            new CANcoder(2),
            new CANcoder(3),
            new CANcoder(4) },
        // absoluteTurnOffset
        new double[] { 0, 0, 0, 0 }, (double) 150 / 7,
        // driveMotors
        new CANSparkMax[] {
            new CANSparkMax(9, MotorType.kBrushless),
            new CANSparkMax(10, MotorType.kBrushless),
            new CANSparkMax(11, MotorType.kBrushless),
            new CANSparkMax(12, MotorType.kBrushless) },
        1,
        // amps
        new int[] { 10, 10 }, 1, 0, 0, 1);

    // DriveTrain.invertAll();
    Limelight = new LimelightBase();
    leds = new LedBase(1, 60, 90);
    leds.sections[0].init(05, 3, Color.kRed, new Color(0, 255, 0), Color.kBlue).doMoveBackward();
    leds.sections[1].init(10, 0.00001, Color.kRed, new Color(0, 255, 0),
        Color.kBlue).doRainbow();
    // teleDriveCommandBase = new TeleDriveCommandBase(DriveTrain, false, Zero);
    teleDriveCommandBase = new TeleDriveCommandBase(SwerveDrive,
        new AutoControllerSelector(m_controllers).addController(0));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // m_controllers.updateAutoControllers();
    if (counter++ % 500 == 0) {
      System.out.println(pdp.getVoltage());
    }
  }

  @Override
  public void disabledInit() {
    System.out.println("Robot Disabled");
    if (DriverStation.isEStopped()) {
      BasicFunctions.KILLIT(DriveTrain);
    }
  }

  @Override
  public void disabledPeriodic() {
    robotState = RobotStateEnum.Disabled;
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

    // start main driving command
    CommandScheduler.getInstance().schedule(teleDriveCommandBase);
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
