// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import org.surpurdueper.robot.subsystems.Blinkin;
import org.surpurdueper.robot.subsystems.Intake;

public class SetLightsGreen extends Command {
  private Blinkin blinkin;
  private Intake intake;

  public SetLightsGreen(Blinkin blinkin, Intake intake) {
    if (intake.isFeederBreakBeamTriggered() == true) {
      blinkin.SetLightsGreen();
    }
  }
}
