package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class HoodIOSim implements HoodIO {
    private Angle setpointAngle = Degrees.zero();

    @Override
    public void updateInputs(HoodInputs inputs) {
        inputs.position = setpointAngle;
    }

    @Override
    public void runPosition(Angle angle) {
        setpointAngle = angle;
    }
}
