package org.team498.C2023.subsystems;

import org.team498.C2023.State;
import org.team498.C2023.Constants.ElevatorConstants;
import org.team498.lib.drivers.LazyTalonFX;
import org.team498.lib.wpilib.PController;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    //motors
    private final LazyTalonFX topMotor; // Declaration for a TalonFX / Falcon500 brushless motor
    private final LazyTalonFX bottomMotor;
    private final DutyCycle encoder;

    private final PController pController;
    private final ElevatorFeedforward feedforward;

    private State.Elevator currentState;
    private double manualSpeed;

    private boolean isManual = false;


    public Elevator() {
        topMotor = new LazyTalonFX(0);
        bottomMotor = new LazyTalonFX(0);
        encoder = new DutyCycle(new DigitalInput(0));

        pController = new PController(ElevatorConstants.P);
        feedforward = new ElevatorFeedforward(ElevatorConstants.S, ElevatorConstants.G, ElevatorConstants.V);

        currentState = State.Elevator.IDLE;
    }

    @Override
    public void periodic() {
        double speed;
        if (isManual) {
            speed = manualSpeed;
        } else {
            double position = currentState.height;
            speed = pController.calculate(position, topMotor.getSelectedSensorPosition() / ElevatorConstants.MOTOR_ROTATION_TO_METERS);
        }
        speed = feedforward.calculate(speed);
        set(speed);
    }

    public State.Elevator getState() {
        return currentState;
    }

    public void setState(State.Elevator state) {
        currentState = state;
    }

    public void setManual(boolean isManual) {
        this.isManual = isManual;
    }

    public void setManualSpeed(double speed) {
        this.manualSpeed = speed;
    }

    private void set(double speed) {
        topMotor.set(ControlMode.PercentOutput, speed);
        bottomMotor.set(ControlMode.PercentOutput, speed);
    }

    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null) instance = new Elevator();
        return instance;
    }
}
