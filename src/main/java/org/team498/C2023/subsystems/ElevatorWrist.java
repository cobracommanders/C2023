package org.team498.C2023.subsystems;

import org.team498.C2023.State;
import org.team498.lib.drivers.LazySparkMax;
import org.team498.lib.wpilib.PController;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.team498.C2023.Constants;
import org.team498.C2023.Ports;

/*
 * uses 1 neo mounted on the elevator
 * geared to the wrist to move up and down
 * Has rev through-bore encoder
 */
public class ElevatorWrist extends SubsystemBase {
    private final LazySparkMax motor;
    private final DutyCycle encoder;

    private final PController pController;

    private double setpoint;
    private State.ElevatorWrist currentState;

    private boolean isManual = false;
    private double manualSpeed;

    public ElevatorWrist(){
        motor = new LazySparkMax(Ports.ElevatorWrist.WRIST, MotorType.kBrushless);
        encoder = new DutyCycle(new DigitalInput(Ports.ElevatorWrist.ENCODER));

        pController = new PController(Constants.ElevatorWrist.P);

        motor.restoreFactoryDefaults();

        setpoint = 0;
        currentState = State.ElevatorWrist.IDLE_CUBE;
        manualSpeed = 0;
    }

    @Override
    public void periodic() {
        double speed;
        if(isManual){
            speed = manualSpeed;
        }  else {
            speed = pController.calculate(encoder.getOutput() -.22);
        }
        if(Elevator.getInstance().isSafe()) set(speed);

        SmartDashboard.putNumber("Wrist speed", speed);
        SmartDashboard.putNumber("Setpoint", setpoint);
        SmartDashboard.putNumber("Encoder", encoder.getOutput() -.22);
    }

    public boolean atSetpoint() {
        return pController.atSetpoint(0.01);
    }

    public State.ElevatorWrist getState(){
        return currentState;
    }
    
    public void setState(State.ElevatorWrist state){
        currentState = state;
        setpoint = state.angle;
        pController.setSetpoint(setpoint);
    }

    public void setManual(boolean isManual) {
        this.isManual = isManual;
    }

    public void setManualSpeed(double speed) {
        this.manualSpeed = speed;
    }

    public void set(double speed){
        motor.set(-speed);
    }

    private static ElevatorWrist instance;

    public static ElevatorWrist getInstance(){
        if(instance == null) instance = new ElevatorWrist();
        return instance;
    }
}
