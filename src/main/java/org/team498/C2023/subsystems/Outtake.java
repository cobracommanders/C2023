package org.team498.C2023.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.Compressor;

import static org.team498.C2023.Ports.Outtake.*;

public class Outtake extends SubsystemBase {
    private final CANSparkMax topRollers;
    private final CANSparkMax bottomRollers;
    private final DigitalInput beamBreak;
    private final DoubleSolenoid solenoid;

    public enum Position {
        SHOOTING(Value.kForward),
        INTAKING(Value.kReverse);

        private final Value value;

        Position(Value value) {
            this.value = value;
        }
    }

    public enum State {
        SHOOT_CONE(Position.SHOOTING, -0.75, 0),
        SHOOT_CUBE(Position.INTAKING, -0.45, 0.45),
        INTAKE_CONE(Position.INTAKING, 1, 0),
        INTAKE_CUBE(Position.INTAKING, 0.5, -0.5),
        IDLE(Position.INTAKING, 0, 0);

        private final Position position;
        private final double bottomRollerSpeed;
        private final double topRollerSpeed;

        State(Position position, double bottomRollerSpeed, double topRollerSpeed) {
            this.position = position;
            this.bottomRollerSpeed = bottomRollerSpeed;
            this.topRollerSpeed = topRollerSpeed;
        }
    }

    private Outtake() {
        topRollers = new CANSparkMax(OUTTAKE_TOP, MotorType.kBrushless);
        bottomRollers = new CANSparkMax(OUTTAKE_BOTTOM, MotorType.kBrushless);

        topRollers.restoreFactoryDefaults();
        bottomRollers.restoreFactoryDefaults();

        topRollers.setIdleMode(IdleMode.kBrake);
        bottomRollers.setIdleMode(IdleMode.kBrake);

        solenoid = Compressor.getInstance().createDoubleSolenoid(SOLENOID_FORWARDS, SOLENOID_REVERSE);
        beamBreak = new DigitalInput(BEAM_BREAK);
    }

    public boolean isTopStalling() {
        return topRollers.getOutputCurrent() > 20;
    }

    public boolean isBottomStalling() {
        return bottomRollers.getOutputCurrent() > 25;
    }

    public boolean hasGamePiece() {
        return !beamBreak.get();
    }

    public FunctionalCommand setOuttake(State state) {
        return new InstantCommand(() -> {
            setTopRollers(state.topRollerSpeed);
            setBottomRollers(state.bottomRollerSpeed);
            setPosition(state.position);
        });
    }

    public void setTopRollers(double speed) {
        topRollers.set(speed);
    }

    public void setBottomRollers(double speed) {
        bottomRollers.set(speed);
    }

    public void setPosition(Position position) {
        solenoid.set(position.value);
    }

    public void stop() {
        setTopRollers(0);
        setBottomRollers(0);
    }

    private static Outtake instance;

    public static Outtake getInstance() {
        if (instance == null) {
            instance = new Outtake();
        }

        return instance;
    }
}
