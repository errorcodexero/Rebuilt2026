package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;

public class HoodIOServo implements HoodIO {
    
    private Servo hoodLeft;
    private Servo hoodRight;

    @Override
    public void updateInputs(HoodInputs inputs) {
        inputs.position = Degrees.of(hoodLeft.getAngle());
    }

    @Override
    public void runPosition(Angle angle) {
        double degrees = angle.in(Degrees);
        hoodLeft.setAngle(degrees);
        hoodRight.setAngle(degrees);
    }
}
