package org.team498.C2023.subsystems;

import org.team498.C2023.Ports;
import org.team498.C2023.State;
import org.team498.lib.drivers.LazyTalonFX;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase{
    private final LazyTalonFX leftMotor;
    private final LazyTalonFX rightMotor;
    private SlewRateLimiter limiter;
    private double currentSpeed;
    private State.Manipulator currentState;

    public Manipulator(){
        leftMotor = new LazyTalonFX(Ports.Manipulator.L_ROLLERS);
        rightMotor = new LazyTalonFX(Ports.Manipulator.R_ROLLERS);
        limiter = new SlewRateLimiter(6.5, -1000, 0);
    
        currentState = State.Manipulator.IDLE;
        currentSpeed = 0;

        leftMotor.configFactoryDefault();
        rightMotor.configFactoryDefault();

    }

    @Override
    public void periodic() {
        double speed = limiter.calculate(currentSpeed);
        set(speed);
    }

    public void set(double speed){
        leftMotor.set(ControlMode.PercentOutput, speed);
        rightMotor.set(ControlMode.PercentOutput, -speed);
    }

    public void setState(State.Manipulator state){
        double setpoint = Math.abs(state.setpoint) * 11;
        if (setpoint == 0) setpoint = 9999;
        limiter = new SlewRateLimiter(setpoint, -9999, currentSpeed);
        if (state.setpoint > 0) currentSpeed = 1;
        else if(state.setpoint < 0) currentSpeed = -1;
        else if(state.setpoint == 0) currentSpeed = 0;
        currentState = state;
    }

    public State.Manipulator getState(){
        return currentState;
    }

    private static Manipulator instance;

    public static Manipulator getInstance(){
        if(instance == null) instance = new Manipulator();
        return instance;
    }

}
