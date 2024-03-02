// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase {

  private final Spark lights = new Spark(1);

  public static final double orange = 0.65;
  public static final double strobeGold = -0.07;
  public static final double black = .99;

  /** Creates a new blinkin. */
  public void setLightsOrange() {
    lights.set(orange);
  }

  public void setLightsStrobeGold() {
    lights.set(strobeGold);
  }

  public void setLightsOff() {
    lights.set(black);
  }

  public Blinkin() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
