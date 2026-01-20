package frc.robot;

/**
 * Enumeration for selecting which mechanism to characterize with SysId.
 * Used in test mode to run characterization routines.
 */
public enum SysIdMechanism {
  INTAKE_DEPLOY("Intake Deploy"),
  INTAKE_SPINNER("Intake Spinner"),
  SHOOTER_FLYWHEEL("Shooter Flywheel"),
  HOPPER_AGITATOR("Hopper Agitator"),
  HOPPER_FEEDER("Hopper Feeder");

  private final String name;

  SysIdMechanism(String name) {
    this.name = name;
  }

  @Override
  public String toString() {
    return name;
  }
}
