package org.team498.C2023.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.*;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import org.team498.lib.drivers.TalonFX;
import org.team498.lib.util.Falcon500Conversions;

import static org.team498.C2023.Constants.ElevatorConstants.*;
import static org.team498.C2023.Ports.Elevator.*;

public class ElevatorIOCompRobot implements ElevatorIO {
    private final TalonFX leader;
    private final TalonFX follower;
    private final DutyCycle encoder;

    private final ElevatorFeedforward FF = new ElevatorFeedforward(0, 0, 0);

    public ElevatorIOCompRobot() {
        leader = new TalonFX(F_ELEVATOR_ID);
        follower = new TalonFX(B_ELEVATOR_ID);

        leader.configFactoryDefault();
        follower.configFactoryDefault();

        leader.setNeutralMode(NeutralMode.Brake);
        follower.setNeutralMode(NeutralMode.Brake);

        leader.configVoltageCompSaturation(12.0);
        follower.configVoltageCompSaturation(12.0);

        leader.enableVoltageCompensation(true);
        follower.enableVoltageCompensation(true);

        leader.setInverted(true);
        follower.setInverted(true);

        follower.follow(leader, FollowerType.PercentOutput);

        encoder = new DutyCycle(new DigitalInput(ENCODER_PORT));
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = (leader.getSelectedSensorPosition() / 2048) / MOTOR_ROTATION_TO_METERS;
        inputs.velocityMetersPerSecond = (Falcon500Conversions.falconToRPM(leader.getSelectedSensorVelocity(), 1) / 60) / MOTOR_ROTATION_TO_METERS;
        inputs.temperatureCelsius = new double[] { leader.getTemperature(), follower.getTemperature() };
    }

    @Override
    public void setSpeed(double speed) {
        leader.set(TalonFXControlMode.PercentOutput, speed);
    }

    @Override
    public void setInitialPosition(boolean inAutoPose) {
        double angle;
        if (inAutoPose) {
            angle = encoder.getOutput() + 0.5;
            if (angle < 1)
                angle += 1;

            setEncoderPosition(angle - 0.0);
        } else {
            angle = encoder.getOutput() + 0.25;
            if (angle < 1)
                angle += 1;

            setEncoderPosition(angle - 0.744160 - 0.25);
        }
    }

    @Override
    public void setPosition(double position, double feedforward) {
        leader.set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, feedforward);
    }
    private void setEncoderPosition(double position) {
        leader.setSelectedSensorPosition(position * 2048);
    }

    @Override
    public void configPID(double kP, double kI, double kD) {
        leader.config_kP(0, kP);
        leader.config_kI(0, kI);
        leader.config_kD(0, kD);
    }


}
