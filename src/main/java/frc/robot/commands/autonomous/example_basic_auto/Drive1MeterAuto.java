package frc.robot.commands.autonomous.example_basic_auto;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class Drive1MeterAuto extends TimedRobot {
    // Declare TalonFX motor controller
    private TalonFX talonFX;

    // Speed setting for the motors
    private double motorSpeed = 0.5;

    @Override
    public void robotInit() {
        // Initialize the TalonFX motor controller (CAN ID = 1 for example)
        talonFX = new TalonFX(1);
    }

    @Override
    public void autonomousInit() {
        // Reset motor controllers to start from a known state
        talonFX.setSelectedSensorPosition(0); // Optional: reset encoder if you had one
    }

    @Override
    public void autonomousPeriodic() {
        // Move the motor at a constant speed
        talonFX.set(motorSpeed);

        // Set a timer or some logic to control how long it should move forward
        Timer.delay(0.1); // Just to avoid a tight loop (you can adjust this)
    }

    @Override
    public void teleopPeriodic() {
        // Teleop code (if needed, not affecting autonomous movement)
    }

    @Override
    public void testPeriodic() {
        // Test mode code, not needed here
    }
}