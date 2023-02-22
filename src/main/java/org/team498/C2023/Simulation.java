package org.team498.C2023;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team498.C2023.subsystems.Elevator;
import org.team498.C2023.subsystems.Wrist;

public class Simulation {
    private final Elevator elevator = Elevator.getInstance();
    private final Wrist wrist = Wrist.getInstance();
    private final Mechanism2d mechanism2d = new Mechanism2d(Units.inchesToMeters(35), Units.inchesToMeters(65));
    private final MechanismRoot2d root = mechanism2d.getRoot("Root", Units.inchesToMeters(1), Units.inchesToMeters(1));

    private final MechanismLigament2d drivetrain = root.append(new MechanismLigament2d("Drivetrain", Units.inchesToMeters(28), 0));
    private final MechanismLigament2d elevatorBase = root.append(new MechanismLigament2d("Elevator Base", Units.inchesToMeters(3), 90));
    private final MechanismLigament2d elevatorBase2 = elevatorBase.append(new MechanismLigament2d("Elevator", Units.inchesToMeters(4), -30));


    /* ELEVATOR */
    private final ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getNEO(2),
                                                            5,
                                                            5.36781211,
                                                            Units.inchesToMeters(1.273),
                                                            0,
                                                            1.5,
                                                            false,
                                                            VecBuilder.fill(0.00)
    );
    private final MechanismLigament2d elevatorMechanism = elevatorBase2.append(new MechanismLigament2d("Elevator",
                                                                                                       elevatorSim.getPositionMeters(),
                                                                                                       0
    ));


    /* WRIST */
    private final SingleJointedArmSim wristSim = new SingleJointedArmSim(DCMotor.getNEO(1),
                                                                         200,
                                                                         SingleJointedArmSim.estimateMOI(10.75, 2),
                                                                         10.75,
                                                                         Math.toRadians(-30),
                                                                         Math.toRadians(200),
                                                                         true,
                                                                         VecBuilder.fill(0.00)
    );
    private final MechanismLigament2d wristMechanism = elevatorMechanism.append(new MechanismLigament2d("Wrist",
                                                                                                        Units.inchesToMeters(10.75),
                                                                                                        wristSim.getAngleRads()
    ));


    public Simulation() {
        SmartDashboard.putData("Elevator Sim", mechanism2d);
    }

    public void update() {
        elevatorSim.setInput(elevator.getPower() * RobotController.getBatteryVoltage());
        elevatorSim.update(Robot.kDefaultPeriod);
        elevator.setEncoderPosition(elevatorSim.getPositionMeters() * Constants.ElevatorConstants.MOTOR_ROTATION_TO_METERS);
        elevatorMechanism.setLength(elevator.getPosition());

        wristSim.setInput(wrist.getPower() * RobotController.getBatteryVoltage());
        wristSim.update(Robot.kDefaultPeriod);
        wrist.setSimAngle(Math.toDegrees(wristSim.getAngleRads()));
        wristMechanism.setAngle(wrist.getAngle() * 360 - 60);
    }


    private static Simulation instance;

    public static Simulation getInstance() {
        if (instance == null) {
            instance = new Simulation();
        }
        return instance;
    }
}
