// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Add your docs here. */
public class TunnableNumber {
  double prev;
  private LoggedNetworkNumber value;
  public boolean updateConfig = false;

  public TunnableNumber(String key, double defaultvalue) {
    value = new LoggedNetworkNumber(key, defaultvalue);
    prev = value.get();
  }

  public void periodic() {
    if (value.get() != prev) {
      prev = value.get();
      updateConfig = true;
    } else {
      updateConfig = false;
    }
  }

  public double get() {
    return value.get();
  }
}
