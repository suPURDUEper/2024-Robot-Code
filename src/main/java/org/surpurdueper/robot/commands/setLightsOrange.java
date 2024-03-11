// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.surpurdueper.robot.subsystems.Blinkin;

public class setLightsOrange extends Command {
  public final Blinkin blinkin;

  public setLightsOrange(Blinkin blinkin) {
    this.blinkin = blinkin;
  }

  @Override
  public void initialize() {
    blinkin.setLightsOrange();
  }
}
