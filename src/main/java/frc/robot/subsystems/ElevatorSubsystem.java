package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private static final int LEADER_MOTOR_ID = 9;
    private static final int FOLLOWER_MOTOR_ID = 10;
    private static final int ENCODER_DIO_PORT = 0; // Update

    // Elevator Setpoints (in encoder units)
    public static final double INTAKE_POSITION = 0.0;
    public static final double L2_POSITION = 10.0;
    public static final double L3_POSITION = 20.0;
    public static final double ALGAE_SCORE_POSITION = 30.0;

    private final SparkMax leaderMotor;
    private final SparkMax followerMotor;
    private final DigitalInput throughBoreEncoder;
    private final PIDController pidController;

    private double targetPosition = INTAKE_POSITION;



    public ElevatorSubsystem() {
        leaderMotor = new SparkMax(LEADER_MOTOR_ID, MotorType.kBrushless);
        followerMotor = new SparkMax(FOLLOWER_MOTOR_ID, MotorType.kBrushless);
        throughBoreEncoder = new DigitalInput(ENCODER_DIO_PORT);

    /*
     * Create new SPARK MAX configuration objects. These will store the
     * configuration parameters for the SPARK MAXes that we will set below.
     */
    SparkMaxConfig elevatorLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();

    /*
     * Set parameters that will apply to all SPARKs. We will also use this as
     * the elevator leader config.
     */
    elevatorLeaderConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake)
        .inverted(false);

    elevatorFollowerConfig
        .apply(elevatorLeaderConfig)
        .follow(LEADER_MOTOR_ID)
        .inverted(true);

    //Apply configurations to the motors

    leaderMotor.configure(elevatorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(elevatorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



        // PID Controller (Tune these values)
        pidController = new PIDController(0.05, 0.0, 0.0); // Adjust for better control
        pidController.setTolerance(0.5); // Set allowable error range


    }

    /**
     * Sets the target elevator position.
     */
    public void setElevatorPosition(double position) {
        targetPosition = position;
    }

    /**
     * Reads the encoder position.
     */
    private double getElevatorPosition() {
        return throughBoreEncoder.get() ? 1.0 : 0.0; // This is a placeholder, use real PWM decoding if necessary
    }

    @Override
    public void periodic() {
        double currentPosition = getElevatorPosition();
        double power = pidController.calculate(currentPosition, targetPosition);

        leaderMotor.set(power); // Apply PID output to leader motor

        // Debug Output
        System.out.println("🚀 Elevator Position: " + currentPosition + " | Target: " + targetPosition + " | Power: " + power);
    }
}
