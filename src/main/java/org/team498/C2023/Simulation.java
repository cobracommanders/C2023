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
import org.team498.C2023.subsystems.ElevatorWrist;
import org.team498.C2023.subsystems.IntakeWrist;
import org.team498.C2023.subsystems.elevator.Elevator;

public class Simulation {
    private final Elevator elevator = Elevator.getInstance();
    private final ElevatorWrist elevatorWrist = ElevatorWrist.getInstance();
    private final IntakeWrist intakeWrist = IntakeWrist.getInstance();
    private final Mechanism2d mechanism2d = new Mechanism2d(Units.inchesToMeters(35), Units.inchesToMeters(65));
    private final MechanismRoot2d root = mechanism2d.getRoot("Root", Units.inchesToMeters(1), Units.inchesToMeters(1));

    private final MechanismLigament2d drivetrain = root.append(new MechanismLigament2d("Drivetrain", Units.inchesToMeters(28), 0));
    private final MechanismLigament2d elevatorBase = root.append(new MechanismLigament2d("Elevator Base", Units.inchesToMeters(3), 90));
    private final MechanismLigament2d elevatorBase2 = elevatorBase.append(new MechanismLigament2d("Elevator", Units.inchesToMeters(4), -30));

    /* ELEVATOR */
    private final ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getFalcon500(2),
                                                            5,
                                                            5.36781211,
                                                            Units.inchesToMeters(1.273),
                                                            0,
                                                            1.5,
                                                            true,
                                                            VecBuilder.fill(0.00)
    );
    private final MechanismLigament2d elevatorMechanism = elevatorBase2.append(new MechanismLigament2d("Elevator",
                                                                                                       elevatorSim.getPositionMeters(),
                                                                                                       0
    ));


    /* ELEVATOR WRIST */
    private final SingleJointedArmSim elevatorWristSim = new SingleJointedArmSim(DCMotor.getNEO(1),
                                                                                 200,
                                                                                 SingleJointedArmSim.estimateMOI(10.75, 2),
                                                                                 10.75,
                                                                                 Math.toRadians(-30),
                                                                                 Math.toRadians(270),
                                                                                 true,
                                                                                 VecBuilder.fill(0.00)
    );
    private final MechanismLigament2d elevatorWristMechanism = elevatorMechanism.append(new MechanismLigament2d("Elevator Wrist",
                                                                                                                Units.inchesToMeters(15),
                                                                                                                90
    ));

    /* INTAKE WRIST */
    private final SingleJointedArmSim intakeWristSim = new SingleJointedArmSim(DCMotor.getNEO(2),
                                                                               100,
                                                                               SingleJointedArmSim.estimateMOI(1, 2),
                                                                               10.75,
                                                                               Math.toRadians(-20000),
                                                                               Math.toRadians(20000),
                                                                               false,
                                                                               VecBuilder.fill(0.00)
    );
    private final MechanismLigament2d intakeWristMechanism = drivetrain.append(new MechanismLigament2d("Intake Wrist",
                                                                                              Units.inchesToMeters(18),
                                                                                              intakeWristSim.getAngleRads()
    ));


    public Simulation() {
        SmartDashboard.putData("Elevator Sim", mechanism2d);
    }

    public void update() {
        // elevatorSim.setInput(elevator.getPower() * RobotController.getBatteryVoltage());
        // elevatorSim.update(Robot.DEFAULT_PERIOD);
        // elevator.setEncoderPosition(elevatorSim.getPositionMeters() * Constants.ElevatorConstants.MOTOR_ROTATION_TO_METERS);
        // elevatorMechanism.setLength(elevator.getPosition());

        elevatorWristSim.setInput(elevatorWrist.getPower() * RobotController.getBatteryVoltage());
        elevatorWristSim.update(Robot.DEFAULT_PERIOD);
        elevatorWrist.setSimAngle(Math.toDegrees(elevatorWristSim.getAngleRads()));
        elevatorWristMechanism.setAngle(elevatorWrist.getAngle() * 360 - 60);

        intakeWristSim.setInput(-intakeWrist.getPower() * RobotController.getBatteryVoltage());
        intakeWristSim.update(Robot.DEFAULT_PERIOD);
        intakeWrist.setSimAngle(Math.toDegrees(intakeWristSim.getAngleRads()));
        intakeWristMechanism.setAngle(intakeWrist.getAngle() * 360 - 20);
    }


    private static Simulation instance;

    public static Simulation getInstance() {
        if (instance == null) {
            instance = new Simulation();
        }
        return instance;
    }
}
