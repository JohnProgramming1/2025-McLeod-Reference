package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class CoralHandlerSubsystem extends SubsystemBase {

    // Placeholder DIO Ports
    private static final int HOPPER_BEAM_BREAK_PORT = 0;
    private static final int INTAKE_BEAM_BREAK_PORT = 1;
    private static final int OUTTAKE_BEAM_BREAK_PORT = 2;

    // Placeholder CAN ID
    private static final int CORAL_HANDLER_MOTOR_CAN_ID = 9;

    // Beam Break Sensors
    private final DigitalInput hopperBeamBreak = new DigitalInput(HOPPER_BEAM_BREAK_PORT);
    private final DigitalInput intakeBeamBreak = new DigitalInput(INTAKE_BEAM_BREAK_PORT);
    private final DigitalInput outtakeBeamBreak = new DigitalInput(OUTTAKE_BEAM_BREAK_PORT);

    // Coral Handler Motor
    private final SparkMax coralHandlerMotor = new SparkMax(CORAL_HANDLER_MOTOR_CAN_ID, MotorType.kBrushless);

    // Elevator Movement Lock
    private boolean elevatorLocked = false;

    // Failsafe Timer
    private final Timer failsafeTimer = new Timer();
    private final Timer reverseTimer = new Timer(); // Timer for reverse action in case of failsafe
    private static final double FAILSAFE_TIMEOUT = 3.0; // 3-second timeout
    private static final double REVERSE_DURATION = 1.0; // Reverse for 1 second

    // Track if coral is passing through
    private boolean isCoralInProcess = false;
    private boolean isReversing = false;


    public CoralHandlerSubsystem() {
        coralHandlerMotor.set(0); // Ensure motor starts off
    }

    //Start Intake//
    public void startIntake() {
        coralHandlerMotor.set(0.5);
        System.out.println("Coral Handler Intake Active");
    }

    //Start Outtake//
    public void startOuttake() {
        coralHandlerMotor.set(-0.5);
        System.out.println("Coral Handler Outtake Active");
    }

    public void activateManualOverride() {
        elevatorLocked = false; // Unlock the elevator manually
        System.out.println("Manual Override Activated: Elevator Unlocked");
    }


    @Override
    public void periodic() {
        // Beam break logic (Beam break is triggered when FALSE)
        boolean hopperBroken = !hopperBeamBreak.get();
        boolean intakeBroken = !intakeBeamBreak.get();
        boolean outtakeBroken = !outtakeBeamBreak.get();

        // Hopper Beam Break → Start Intake
        if (hopperBroken && !isCoralInProcess && !isReversing) {
            coralHandlerMotor.set(0.5); // Start motor
            elevatorLocked = true;
            isCoralInProcess = true;
            failsafeTimer.reset();
            failsafeTimer.start();
            setLEDColor("BLUE"); // Intake Active
            System.out.println("Coral detected at hopper. Intake started, elevator locked.");
        }

        // Intake Beam Break Cleared → Stop Intake
        if (isCoralInProcess && !intakeBroken && !isReversing) {
            coralHandlerMotor.set(0); // Stop motor
            elevatorLocked = false;
            isCoralInProcess = false;
            failsafeTimer.stop();
            setLEDColor("YELLOW"); // Coral ready for scoring
            System.out.println("Coral cleared intake. Elevator unlocked.");
        }

        // Outtake Beam Break → Coral Fully Ejected
        if (outtakeBroken) {
            setLEDColor("RED"); // Ejecting
            System.out.println("Coral fully ejected.");
        }

        // Failsafe Timeout → Trigger Reverse
        if (isCoralInProcess && failsafeTimer.get() > FAILSAFE_TIMEOUT && !isReversing) {
            isReversing = true;
            reverseTimer.reset();
            reverseTimer.start();
            coralHandlerMotor.set(-0.5); // Reverse motor
            setLEDColor("PURPLE");
            System.out.println("FAILSAFE: Reversing motor for " + REVERSE_DURATION + " second to clear jam.");
        }

        // Complete Reverse → Resume Intake
        if (isReversing && reverseTimer.get() > REVERSE_DURATION) {
            reverseTimer.stop();
            coralHandlerMotor.set(0.5); // Resume normal intake
            failsafeTimer.reset(); // Reset the failsafe timer
            failsafeTimer.start();
            isReversing = false;
            System.out.println("Reverse complete. Resuming normal intake.");
            setLEDColor("BLUE"); // Resume intake
        }
    }

    public boolean isElevatorLocked() {
        return elevatorLocked;
    }

    private void setLEDColor(String color) {
        // Placeholder for LED control
        System.out.println("LED Color: " + color);
    }

    public void stopCoralHandler() {
        coralHandlerMotor.set(0);
        setLEDColor("GREEN"); // Idle
        System.out.println("Coral Handler Stopped");
    }
}