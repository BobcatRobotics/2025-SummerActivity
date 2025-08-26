package frc.robot.util;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TunableDouble {
private final String key;
  private boolean configUpdate = false;
  private LoggedNetworkNumber value;
  private double last;

  public TunableDouble(String key, double defaultVal) {
    value = new LoggedNetworkNumber(key, defaultVal);
    this.key = key;
    last = defaultVal;
  }

  public void periodic() {
    double current = value.get();
    if (current != last) {
      last = value.get();
      configUpdate = true;
    } else {
      configUpdate = false;
    }
  }

  public double get() {
    return value.get();
  }

  public boolean check() {
    return configUpdate;
  }
}