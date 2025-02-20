package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.AnalogInput;


public class AlgaeSubsystem extends SubsystemBase {

    // Constants
    private static final int ALGAE_MOTOR_CAN_ID = 11; // Placeholder
    private static final int ALGAE_DISTANCE_SENSOR_PORT = 0; // Placeholder
    private static final double HOLD_POWER = 0.1; // Minimal power to hold algae
    private static final double INTAKE_POWER = 0.5; // Power to pull algae in
    private static final double EJECT_POWER = -0.5; // Power to eject algae
    private static final double HOLD_DISTANCE_CM = 6.0; // Distance to hold algae

    // Hardware
    private final SparkMax algaeMotor;
    private final AnalogInput algaeDistanceSensor;

    private boolean isEjecting = false;

    public AlgaeSubsystem() {
        algaeMotor = new SparkMax(ALGAE_MOTOR_CAN_ID, MotorType.kBrushless);
        algaeDistanceSensor = new AnalogInput(ALGAE_DISTANCE_SENSOR_PORT);

        SparkMaxConfig algaeMotorConfig = new SparkMaxConfig();
        algaeMotorConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(false);

        algaeMotor.configure(algaeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        double distance = getDistanceCM();

        if (isEjecting) {
            algaeMotor.set(EJECT_POWER);
            System.out.println("Ejecting Algae");
        } else if (distance <= HOLD_DISTANCE_CM) {
            algaeMotor.set(HOLD_POWER);
            System.out.println("Holding Algae at " + distance + " cm");
        } 
    }

    // Run the algae motor (used during scoring)
    public void runAlgae() {
        if (!isEjecting) {
            algaeMotor.set(INTAKE_POWER);
            System.out.println("Running Algae Intake");
        }
    }

    // Stop the algae motor
    public void stopAlgae() {
        if (!isEjecting) {
            algaeMotor.set(0);
            System.out.println("Stopping Algae Motor");
        }
    }

    // Eject algae on command
    public void ejectAlgae() {
        isEjecting = true;
        System.out.println("Ejecting Algae");
    }

    // Stop ejection
    public void stopEject() {
        isEjecting = false;
        algaeMotor.set(0);
    }

    // Get distance in cm from sensor
    private double getDistanceCM() {
        // Placeholder conversion; adjust based on your REV Distance Sensor specs
        return algaeDistanceSensor.getVoltage() * 100;
    }
}