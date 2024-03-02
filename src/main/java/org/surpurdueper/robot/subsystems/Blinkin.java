// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot.subsystems;

import org.littletonrobotics.util.VirtualSubsystem;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Blinkin extends VirtualSubsystem {

  private final Spark lights = new Spark(1);

  public static final double orange = 0.65;
  public static final double strobeGold = -0.07;
  public static final double black = .99;
  public static final double rainbow = -0.99;

  /** Creates a new blinkin. */
  public Command setLightsOrange() {
    return Commands.runOnce(() -> lights.set(orange));
  }

  public Command setLightsStrobeGold() {
    return Commands.runOnce(() -> lights.set(orange));
  }

 public Command setLightsOff() {
    return Commands.runOnce(() -> lights.set(orange));
 }

  public Command setLightsRainbow() {
    return Commands.runOnce(() -> lights.set(rainbow));
  }
 

  public Blinkin() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
