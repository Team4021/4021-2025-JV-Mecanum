package frc.robot.commands.autonomous.example_basic_auto;



import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class MecanumAuto extends TimedRobot {
    // Declare TalonFX motor controller
    private TalonFX talonFX;

    // Speed setting for the motors
    private double motorSpeed = 0.5;
    private double motorStop = 0.0;

    @Override
    public void robotInit() {
        // Initialize the TalonFX motor controller (CAN ID = 1 for example)
        talonFX = new TalonFX(1);
    }

    @Override
    public void autonomousPeriodic() {
        // Move the motor at a constant speed
        talonFX.set(motorSpeed);

        // Set a timer or some logic to control how long it should move forward
        Timer.delay(1.5); // Just to avoid a tight loop (you can adjust this)

        talonFX.set(motorStop);
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