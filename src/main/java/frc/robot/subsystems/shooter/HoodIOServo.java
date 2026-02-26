package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;

public class HoodIOServo implements HoodIO {
    private double leftOffset = 8.0;
    private double rightOffset = 0.0;
    
    private Servo hoodLeft;
    private Servo hoodRight;

    public HoodIOServo() {
        hoodLeft = new Servo(ShooterConstants.HoodPWMs.hoodLeftPWMPort);
        hoodRight = new Servo(ShooterConstants.HoodPWMs.hoodRightPWMPort);
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        inputs.position = Degrees.of(hoodLeft.getAngle());
    }

    @Override
    public void goToAngle(Angle angle) {
        if (angle.lt(ShooterConstants.hoodMinAngle) || angle.gt(ShooterConstants.hoodMaxAngle)) {
            return;
        }

        double degrees = angle.in(Degrees);
        hoodLeft.set((degrees + leftOffset) / 300.0);
        hoodRight.set(1.0 - (degrees - rightOffset) / 300.0);
    }

    @Override
    public void applyCalibration(double leftOffset, double rightOffset) {
        this.leftOffset = leftOffset;
        this.rightOffset = rightOffset;
    }
}
