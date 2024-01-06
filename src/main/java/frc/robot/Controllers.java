// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Where all of the controller objects are stored. */
public class Controllers {

  // effects bad if port-bound controllers are not xbox

  /** Generic controller */

  public static class ControllerBase {
    public CommandXboxController commObjectX;
    public XboxController objectX;
    public CommandPS4Controller commObjectPS4;
    public PS4Controller objectPS4;
    public Trigger A;
    public Trigger B;
    public Trigger X;
    public Trigger Y;
    public Trigger LeftBumper;
    public Trigger RightBumper;
    public Trigger Share;
    public Trigger Options;
    public Trigger LeftTrigger;
    public Trigger RightTrigger;
    public Trigger PovUp;
    public Trigger PovUpLeft;
    public Trigger PovUpRight;
    public Trigger PovDown;
    public Trigger PovDownLeft;
    public Trigger PovDownRight;
    public Trigger PovLeft;
    public Trigger PovRight;
    public Trigger LeftStickPress;
    public Trigger RightStickPress;

    public ControllerBase(int port, boolean isPS4) {
      if (!isPS4) {
        this.commObjectX = new CommandXboxController(port);
        this.objectX = commObjectX.getHID();
        this.A = commObjectX.a();
        this.B = commObjectX.b();
        this.X = commObjectX.x();
        this.Y = commObjectX.y();
        this.LeftBumper = commObjectX.leftBumper();
        this.RightBumper = commObjectX.rightBumper();
        this.Share = commObjectX.start();
        this.Options = commObjectX.back();
        this.LeftTrigger = commObjectX.leftTrigger();
        this.RightTrigger = commObjectX.rightTrigger();
        this.PovUp = commObjectX.povUp();
        this.PovUpLeft = commObjectX.povUpLeft();
        this.PovUpRight = commObjectX.povUpRight();
        this.PovDown = commObjectX.povDown();
        this.PovDownLeft = commObjectX.povDownLeft();
        this.PovDownRight = commObjectX.povDownRight();
        this.PovLeft = commObjectX.povLeft();
        this.PovRight = commObjectX.povRight();
        this.LeftStickPress = commObjectX.leftStick();
        this.RightStickPress = commObjectX.rightStick();
      } else {
        this.commObjectPS4 = new CommandPS4Controller(port);
        this.objectPS4 = commObjectPS4.getHID();
        this.A = commObjectPS4.cross();
        this.B = commObjectPS4.circle();
        this.X = commObjectPS4.square();
        this.Y = commObjectPS4.triangle();
        this.LeftBumper = commObjectPS4.L1();
        this.RightBumper = commObjectPS4.R1();
        this.Share = commObjectPS4.share();
        this.Options = commObjectPS4.options();
        this.LeftTrigger = commObjectPS4.L2();
        this.RightTrigger = commObjectPS4.R2();
        this.PovUp = commObjectPS4.povUp();
        this.PovUpLeft = commObjectPS4.povUpLeft();
        this.PovUpRight = commObjectPS4.povUpRight();
        this.PovDown = commObjectPS4.povDown();
        this.PovDownLeft = commObjectPS4.povDownLeft();
        this.PovDownRight = commObjectPS4.povDownRight();
        this.PovLeft = commObjectPS4.povLeft();
        this.PovRight = commObjectPS4.povRight();
        this.LeftStickPress = commObjectPS4.L3();
        this.RightStickPress = commObjectPS4.L3();
      }
    }
  }

  public static ControllerBase Zero = new ControllerBase(0, false);
  public static ControllerBase One = new ControllerBase(1, false);
  public static ControllerBase Two = new ControllerBase(2, false);
  public static ControllerBase Three = new ControllerBase(3, false);
  public static ControllerBase Four = new ControllerBase(4, false);
  public static ControllerBase Five = new ControllerBase(5, false);
}