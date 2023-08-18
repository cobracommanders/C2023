package org.team498.C2023.subsystems;

import java.util.Set;

import org.team498.C2023.Ports;
import org.team498.C2023.State;
import org.team498.lib.drivers.LazyTalonFX;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRollers extends SubsystemBase {
    private final LazyTalonFX topMotor;
    private final LazyTalonFX bottomMotor;
    private final LazyTalonFX thirdMotor;

    private State.IntakeRollers currentState;

    public IntakeRollers(){
        topMotor = new LazyTalonFX(Ports.IntakeRollers.TOP_ROLLER);
        bottomMotor = new LazyTalonFX(Ports.IntakeRollers.BOTTOM_ROLLER);
        thirdMotor = new LazyTalonFX(Ports.IntakeRollers.THIRD_ROLLER);

        topMotor.configFactoryDefault();
        bottomMotor.configFactoryDefault();
        thirdMotor.configFactoryDefault();

        currentState = State.IntakeRollers.IDLE;

    }

    @Override
    public void periodic() {
        set(currentState.topRollerSpeed, currentState.bottomRollerSpeed, currentState.thirdRollerSpeed);
    }

    public void set(double topSpeed, double bottomSpeed, double thirdSpeed){
        topMotor.set(ControlMode.PercentOutput, topSpeed);
        bottomMotor.set(ControlMode.PercentOutput, bottomSpeed);
        thirdMotor.set(ControlMode.PercentOutput, thirdSpeed);
    }

    public State.IntakeRollers getState(){
        return currentState;
    }

    public void setState(State.IntakeRollers state){
        currentState = state;
    } 

    private static IntakeRollers instance;

    public static IntakeRollers getInstance(){
        if(instance == null) instance = new IntakeRollers();
        return instance;
    }

}
