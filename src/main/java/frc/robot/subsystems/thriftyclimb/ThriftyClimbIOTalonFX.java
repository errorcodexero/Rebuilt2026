package frc.robot.subsystems.thriftyclimb;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.Constants.RobotType;

public class ThriftyClimbIOTalonFX implements ThriftyClimbIO {
    protected final TalonFX climb_;

    public ThriftyClimbIOTalonFX(){
        climb_ = new TalonFX(ThriftyClimbConstants.thriftyClimbId);

        TalonFXConfiguration motorOneConfiguration = new TalonFXConfiguration();

        // Not using a remote cancoder.
        // motorOneConfiguration.Feedback.FeedbackRemoteSensorID = 3;
        // motorOneConfiguration.Feedback.SensorToMechanismRatio = 1;

        if (Constants.getRobot() != RobotType.SIMBOT) {
            motorOneConfiguration.Slot0
                .withKP(5)
                .withKI(0)
                .withKD(0);
        } else {
            motorOneConfiguration.Slot0
                .withKP(15)
                .withKI(0)
                .withKD(0);
        }

        motorOneConfiguration.MotionMagic
            .withMotionMagicCruiseVelocity(RevolutionsPerSecond.of(2000))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(100));
            // .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(10));

        tryUntilOk(5, () -> climb_.getConfigurator().apply(motorOneConfiguration, 0.25));
        climb_.setPosition(Radians.of(ThriftyClimbConstants.thriftyStowedHeight.in(Inches) * ThriftyClimbConstants.thriftyGearRatio * Math.PI * ThriftyClimbConstants.thriftyClimbSpoolDiameter.in(Inches)));
    }

    public void updateInputs(ThriftyClimbInputs inputs) {
        inputs.position = Inches.of(climb_.getPosition().getValue().in(Revolutions) / (ThriftyClimbConstants.thriftyGearRatio * Math.PI * ThriftyClimbConstants.thriftyClimbSpoolDiameter.in(Inches)));
        inputs.velocity = InchesPerSecond.of(climb_.getVelocity().getValue().in(RevolutionsPerSecond) / (ThriftyClimbConstants.thriftyGearRatio * Math.PI * ThriftyClimbConstants.thriftyClimbSpoolDiameter.in(Inches)));;
        inputs.current = climb_.getStatorCurrent().getValue();
    }

    public void applyOutputs(ThriftyClimbOutputs outputs) {
        climb_.setControl(new MotionMagicVoltage(Revolutions.of(outputs.setpoint.in(Inches) * ThriftyClimbConstants.thriftyGearRatio * Math.PI * ThriftyClimbConstants.thriftyClimbSpoolDiameter.in(Inches))));
    }
}