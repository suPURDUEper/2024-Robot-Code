// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.util.VirtualSubsystem;

public class Blinkin extends VirtualSubsystem {

  private final Spark lights;

  public static final double orange = 0.65;
  public static final double strobeGold = -0.07;
  public static final double black = .99;
  public static final double rainbow = -0.99;

  private double currentLights = 0;

  public Blinkin() {
    super();
    lights = new Spark(9);
  }

  /** Creates a new blinkin. */
  public Command setLightsOrange() {
    return Commands.runOnce(() -> currentLights = orange);
  }

  public Command setLightsStrobeGold() {
    return Commands.runOnce(() -> currentLights = strobeGold);
  }

  public Command setLightsOff() {
    return Commands.runOnce(() -> currentLights = black);
  }

  public Command setLightsRainbow() {
    return Commands.runOnce(() -> currentLights = rainbow);
  }

  @Override
  public void periodic() {
    lights.set(currentLights);
  }
}
