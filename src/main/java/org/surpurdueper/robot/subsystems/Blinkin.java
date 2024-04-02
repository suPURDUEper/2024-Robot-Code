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

  public static final double kOrange = 0.65;
  public static final double kStrobeGold = -0.07;
  public static final double kBlack = .99;
  public static final double kRainbow = -0.99;
  public static final double kRed = 0.61;
  public static final double kGreen = 0.73;
  public Intake intake;

  public double currentLights = 0;

  public Blinkin(Intake intake) {
    super();
    lights = new Spark(9);
    this.intake = intake;
  }

  public Command setLightsTo(double lights) {
    return Commands.runOnce(() -> currentLights = lights);
  }

  public void SetLightsGreen() {
    lights.set(kGreen);
  }

  public void SetLightsRed() {
    lights.set(kRed);
  }

  @Override
  public void periodic() {
    if (intake.hasDisk()) {
      currentLights = kGreen;
    } else {
      currentLights = kRed;
    }
    lights.set(currentLights);
  }
}
