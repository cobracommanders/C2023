package org.team498.C2023.subsystems;

import org.team498.C2023.Constants;
import org.team498.C2023.Ports;
import org.team498.C2023.State;
import org.team498.lib.drivers.LazySparkMax;
import org.team498.lib.wpilib.PController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWrist extends SubsystemBase {
    private final LazySparkMax lmotor;
    private final LazySparkMax rmotor;
    private final DutyCycle encoder;

    private final PController pController;
    private double setpoint;
    private State.IntakeWrist currentState;

    private boolean isManual = false;
    private double manualSpeed;

    public IntakeWrist(){
        lmotor = new LazySparkMax(Ports.IntakeWrist.L_WRIST, MotorType.kBrushless);
        rmotor = new LazySparkMax(Ports.IntakeWrist.R_WRIST, MotorType.kBrushless);
        encoder = new DutyCycle(new DigitalInput(Ports.IntakeWrist.ENCODER));

        pController = new PController(Constants.IntakeWristConstants.P);

        lmotor.restoreFactoryDefaults();
        rmotor.restoreFactoryDefaults();

        currentState = State.IntakeWrist.IDLE_IN;
        
        setpoint = currentState.position;
        manualSpeed = 0;
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Wrist Angle", getAngle());
        double speed;
        SmartDashboard.putNumber("Intake Wrist Setpoint", setpoint);
        if(isManual){
            speed = manualSpeed;
        }  else {
            speed = pController.calculate(getAngle());
        }
        SmartDashboard.putNumber("Intake Wrist Speed", speed);
        set(speed);
    }

    public boolean atSetpoint(){
        return pController.atSetpoint(.05);
    }

    public State.IntakeWrist getState(){
        return currentState;
    }
    
    public void setState(State.IntakeWrist state){
        currentState = state;
        setpoint = state.position;
        pController.setSetpoint(setpoint);
    }

    public double getAngle() {
        double result = encoder.getOutput();
        result -= Math.floor(result);
        return -(result - 0.739);
    }

    public void setManual(boolean isManual) {
        this.isManual = isManual;
    }

    public void setManualSpeed(double speed) {
        this.manualSpeed = speed;
    }

    public void set(double speed){
        lmotor.set(-speed);
        rmotor.set(speed);
    }

    private static IntakeWrist instance;

    public static IntakeWrist getInstance(){
        if(instance == null) instance = new IntakeWrist();
        return instance;
    }
}