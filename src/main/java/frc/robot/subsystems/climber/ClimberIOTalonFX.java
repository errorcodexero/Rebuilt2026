package frc.robot.subsystems.climber;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;

public class ClimberIOTalonFX implements ClimberIO {

    private final TalonFX motorOne;
    private final TalonFX motorTwo;

    private final StatusSignal<Angle> motorOnePosition;

    private final StatusSignal<Angle> motorTwoPosition;

    private final List<BaseStatusSignal> signalsToUpdate;

    public ClimberIOTalonFX() {
        motorOne = new TalonFX(ClimberConstants.motorOneId);
        motorTwo = new TalonFX(ClimberConstants.motorTwoId);

        motorOnePosition = motorOne.getPosition();

        // Example configuration
        TalonFXConfiguration motorOneConfiguration = new TalonFXConfiguration();
        motorOneConfiguration.Feedback.FeedbackRemoteSensorID = 3;
        motorOneConfiguration.Feedback.SensorToMechanismRatio = 1;
        tryUntilOk(5, () -> motorOne.getConfigurator().apply(motorOneConfiguration, 0.25));

        motorTwoPosition = motorTwo.getPosition();

        signalsToUpdate = List.of(
            motorOnePosition,

            motorTwoPosition
        );
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        BaseStatusSignal.refreshAll(signalsToUpdate);
        BaseStatusSignal.setUpdateFrequencyForAll(50, signalsToUpdate);

        inputs.position = motorOnePosition.getValue();
    }
}
