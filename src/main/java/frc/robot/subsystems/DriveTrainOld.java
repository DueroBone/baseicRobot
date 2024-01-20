// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.ArrayList;

public class DriveTrainOld extends SubsystemBase {

  //// ----- Motor Controllers ----- /////
  // There are 6 separate motor controllers with 1 pwm channel per controller
  public final CANSparkMax motorDriveLeft1 = new CANSparkMax(DriveConstants.leftDrive1Id, CANSparkLowLevel.MotorType.kBrushless);
  public final CANSparkMax motorDriveLeft2 = new CANSparkMax(DriveConstants.leftDrive2Id, CANSparkLowLevel.MotorType.kBrushless);
  public final CANSparkMax motorDriveLeft3 = new CANSparkMax(DriveConstants.leftDrive3Id, CANSparkLowLevel.MotorType.kBrushless);
  public final CANSparkMax motorDriveRight1 = new CANSparkMax(DriveConstants.rightDrive1Id, CANSparkLowLevel.MotorType.kBrushless);
  public final CANSparkMax motorDriveRight2 = new CANSparkMax(DriveConstants.rightDrive2Id, CANSparkLowLevel.MotorType.kBrushless);
  public final CANSparkMax motorDriveRight3 = new CANSparkMax(DriveConstants.rightDrive3Id, CANSparkLowLevel.MotorType.kBrushless);
  private ArrayList<CANSparkMax> leftMotors = new ArrayList<CANSparkMax>(); // define in constructor
  private ArrayList<CANSparkMax> rightMotors = new ArrayList<CANSparkMax>(); // define in constructor
  private ArrayList<CANSparkMax> motors = new ArrayList<CANSparkMax>(); // define in constructor

  // define Speed Controller Groups and Differential Drive for use in drive train
  private final CANSparkMax driveMainLeft = motorDriveLeft1;
  private final CANSparkMax driveMainRight = motorDriveRight1;
  private final DifferentialDrive differentialDrive = new DifferentialDrive(driveMainLeft, driveMainRight);

  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;

  // navX Gyro on RoboRIO 2.0
  public AHRS m_Gyro;

  private final boolean kSkipGyro = false;
  private int counter = 0; // for limiting display
  private double speedMultiplier = DriveConstants.drivingMax;
  public boolean isHighGear = false;
  public boolean isSlowMode = false;

  private DoubleSolenoid gearChanger;

  public DriveTrainOld() {
    System.out.print("Instatntiating drivetrain");
    leftMotors.add(motorDriveLeft1);
    leftMotors.add(motorDriveLeft2);
    leftMotors.add(motorDriveLeft3);
    rightMotors.add(motorDriveRight1);
    rightMotors.add(motorDriveRight2);
    rightMotors.add(motorDriveRight3);
    motors.addAll(leftMotors);
    motors.addAll(rightMotors);

    // Sync side motors
    for (CANSparkMax motor : leftMotors) {
      if (motor != motorDriveLeft1) {
        motor.follow(motorDriveLeft1);
      }
    }
    for (CANSparkMax motor : rightMotors) {
      if (motor != motorDriveRight1) {
        motor.follow(motorDriveRight1);
      }
    }
    for (CANSparkMax motor : motors) {
      motor.restoreFactoryDefaults();
      motor.setSmartCurrentLimit(DriveConstants.driveAmpsMax);
      motor.setClosedLoopRampRate(DriveConstants.drivingRamp);
      motor.setOpenLoopRampRate(DriveConstants.drivingRamp);
      motor.setIdleMode(IdleMode.kCoast);
      motor.enableVoltageCompensation(11);
    }

    // Invert 1 side of robot so will drive forward
    driveMainLeft.setInverted(true);

    differentialDrive.setSafetyEnabled(false);

    m_leftEncoder = motorDriveLeft1.getEncoder();
    m_rightEncoder = motorDriveRight1.getEncoder();

    // Initialize the solenoids
    gearChanger = new DoubleSolenoid(5, PneumaticsModuleType.REVPH, DriveConstants.GearChangeUp,
        DriveConstants.GearChangeDown);
    gearChanger.set(DoubleSolenoid.Value.kOff);

    if (kSkipGyro) {
      m_Gyro = null;
    } else {
      // navX-MXP Gyro instantiation
      try {
        m_Gyro = new AHRS(SPI.Port.kMXP);
      } catch (RuntimeException ex) {
        DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
      }
      while (m_Gyro.isCalibrating()) {
        try {
          Thread.sleep(500);
        } catch (Exception e) {
          System.out.println(e);
        } // sleep in milliseconds
        System.out.print("**gyro isCalibrating . . .");
      }
      // SmartDashboard.putBoolean("gyro connected", m_Gyro.isConnected());
      System.out.print("gyro connected " + m_Gyro.isConnected());
    }
    System.out.println(" ... Done");
  }

  @Override
  public void periodic() {
  }

  /**
   * Tank style driving for the DriveTrain.
   *
   * @param left  Speed in range [-1,1]
   * @param right Speed in range [-1,1]
   */
  public void doTankDrive(double leftDrivePercent, double rightDrivePercent) {
    leftDrivePercent *= speedMultiplier;
    rightDrivePercent *= speedMultiplier;

    if (counter++ % 100 == 0) {
      System.out.println("**driveTrain power L/R: " + leftDrivePercent + " | " + rightDrivePercent);
    }
    if (Math.abs(leftDrivePercent) > 0.01) {
      driveMainLeft.set(rightDrivePercent);
    } else {
      driveMainLeft.stopMotor();
    }
    if (Math.abs(rightDrivePercent) > 0.01) {
      driveMainRight.set(rightDrivePercent);
    } else {
      driveMainRight.stopMotor();
    }
  }

  /**
   * Arcade style driving for the DriveTrain.
   *
   * @param speed    Speed in range [-1,1]
   * @param rotation Rotation in range [-1,1]
   */
  public void doArcadeDrive(double speed, double rotation) {
    // if (counter++ % 100 == 0) { System.out.println("**arcadeDrive power
    // speed/rotation: " + speed+"-"rotation); }
    differentialDrive.arcadeDrive(speed * speedMultiplier, rotation);
  }

  public double getHeadingAngle() {
    return m_Gyro.getAngle();
  }

  public double getYaw() {
    return m_Gyro.getYaw();
  }

  public void resetGyro() {
    m_Gyro.reset();
    m_Gyro.zeroYaw();
  }

  // public void resetEncoder() {
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public double getLeftEncoderCount() {
    return m_leftEncoder.getPosition();
    // return 0;
  }

  public double getRightEncoderCount() {
    return m_rightEncoder.getPosition();
    // return 0;
  }

  public double getLeftDistanceInch() {
    return Math.PI * DriveConstants.WHEEL_DIAMETER * (getLeftEncoderCount() /
    DriveConstants.PULSES_PER_REVOLUTION);
    // return 0.0;
  }

  public double getRightDistanceInch() {
    return Math.PI * DriveConstants.WHEEL_DIAMETER * (getRightEncoderCount() /
    DriveConstants.PULSES_PER_REVOLUTION);
    // return 0.0;
  }

  public double getAveDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
    // return 0.0;
  }

  // Function to set the solenoids
  public void doHighGear(boolean fast) {
    isHighGear = fast;
    if (fast) {
      gearChanger.set(DoubleSolenoid.Value.kReverse);
      System.out.println("Gear shifter set to High Speed Mode");
    } else {
      gearChanger.set(DoubleSolenoid.Value.kForward);
      System.out.println("Gear shifter set to High Torque Mode");
    }
  }

  /** Artificial speed limit, 1/3 */
  public void doSlowMode(boolean slow) {
    isSlowMode = slow;
    if (slow) {
      speedMultiplier = 0.3;
    } else {
      speedMultiplier = DriveConstants.drivingMax;
    }
  }

  public void stop() {
    System.out.println("Drivetrain stop");
    doTankDrive(0.0, 0.0);
  }

  public double getLeftSpeed() {
    return driveMainLeft.get();
  }

  public double getRightSpeed() {
    return driveMainRight.get();
  }

  public double getAveCurrent() {
    double current = 0;
    for (CANSparkMax motor : motors) {
      current += motor.getOutputCurrent();
    }
    return current / motors.size();
  }

  public void setBrakeMode(boolean brake) {
    if (brake) {
      for (CANSparkMax motor : motors) {
        motor.setIdleMode(IdleMode.kBrake);
      }
    } else {
      for (CANSparkMax motor : motors) {
        motor.setIdleMode(IdleMode.kCoast);
      }
    }
  }

  public void resetAll() {
    System.out.println("Resetting DriveTrain");
    stop();
    resetEncoders();
    resetGyro();
    doHighGear(false);
    doSlowMode(false);
    // setBrakeMode(true);
  }
}