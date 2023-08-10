package org.team498.C2023.subsystems;

import org.team498.C2023.Ports;
import org.team498.C2023.State;
import org.team498.C2023.Constants.FlywheelConstants;
import org.team498.lib.drivers.LazySparkMax;
import org.team498.lib.wpilib.PController;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/*
 * This is an example Flywheel subsystem that can be used for reference while writing your own subsystems :)
 * The mechanism uses two NEO brushless motors controlled by Spark Maxes
 * They are mounted opposite each other and power a single-wheel flywheel with velocity control
 * While this code is for a Flywheel, the ideas here can be used for any velocity-based subsystem
 */
public class Flywheel extends SubsystemBase {
    // all variable/object declaration goes at the top of the class. 
    // They can be instantiated (given values) later, but they must be declared here

    // Motors will almost always be private because they will only be controlled using public methods. There should be NO global use where motors 
    private final LazySparkMax lMotor; // Declaration for a NEO or NEO550 brushless motor
    private final LazySparkMax rMotor; // We use left / right descriptors to make identification easier in testing and communication
    private final RelativeEncoder encoder; //Declaration for a Built-in NEO/NEO550 encoder

    private final PController pController; //Declaration for a P Controller
    private final SimpleMotorFeedforward feedforward;

    // Variables will store the current properties of the subsystem
    private double setpoint;
    private State.Flywheel currentState;
    
    // Constructor: Configure Motor Controller settings and  
    // Instantiate all objects (assign values to every variable and object)
    public Flywheel() {
        lMotor = new LazySparkMax(Ports.Flywheel.L_MOTOR, MotorType.kBrushless);
        rMotor = new LazySparkMax(Ports.Flywheel.R_MOTOR, MotorType.kBrushless);
        encoder = lMotor.getEncoder(); //this can be left or right motor, whichever is most convenient

        // Use the subsystems constants to instantiate PID and Feedforward
        pController = new PController(FlywheelConstants.P);
        feedforward = new SimpleMotorFeedforward(FlywheelConstants.S, FlywheelConstants.V, FlywheelConstants.A);

        // reset motor defaults to ensure all settings are clear
        lMotor.restoreFactoryDefaults();
        rMotor.restoreFactoryDefaults();

        // Instantiate variables to intitial values
        setpoint = 0;
        currentState = State.Flywheel.IDLE;
    }

    // This method will run every 10-20 milliseconds (about 50-100 times in one second)
    @Override
    public void periodic() {
        // This condition will reduce CPU utilization when the motor is not meant to run and save power because 
        // it will not actively deccelerate the wheel
        if (setpoint == 0) return; //don't run the motor or calculate anything if the setpoint is 0

        double speed; // We will use this variable to keep track of our desired speed
        speed = feedforward.calculate(setpoint); // adjust the setpoint to account for physical motor properties using feedforward
        speed = pController.calculate(encoder.getVelocity(), speed); // adjust for feedback error using proportional gain
        set(speed / FlywheelConstants.MAX_RPM); // set the motor behavior using set() to interact with the controllers
        // We divide by MAX_RPM to scale to {-1, 1}
    }

    // Getter method to retrieve current State
    public State.Flywheel getState() {
        return currentState;
    }

    // Every subsystem has a setState() method that configures local properties to match the desired state
    public void setState(State.Flywheel state) {
        currentState = state; // update state
        setpoint = state.setpoint; // update setpoint
        pController.setSetpoint(setpoint); // update pController
    }

    // We do NOT use the preset methods for following and inverting motors in case of flash failure 
    // (Ask Caleb about that if you're curious)
    // Use a method to define motor control in relevant groups
    public void set(double speed) {
        lMotor.set(speed);
        rMotor.set(-speed); // invert speed on right side (assuming the motor is facing opposite the left)
    }
    
    // Using static instances to reference the flywheel object ensures that we only use ONE FLywheel throughout the code 
    // This makes it very easy to access the flywheel object
    private static Flywheel instance;

    public static Flywheel getInstance() {
        if (instance == null) instance = new Flywheel(); // Make sure there is an instance (this will only run once)
        return instance;
    }
}
