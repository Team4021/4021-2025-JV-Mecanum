// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import frc.robot.commands.autonomous.example_basic_auto.Drive1MeterAuto;
import frc.robot.subsystems.DriveSubsystem;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.SensorCollection;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  Command m_autonomousCommand;


UsbCamera LiftCam;

	SendableChooser<Command> autonChooser = new SendableChooser<Command>(); // Create a chooser to select an autonomous command

  public static boolean manualDriveControl = true;

  public static final GenericHID controller = new GenericHID(Constants.CONTROLLER_USB_PORT_ID); // Instantiate our controller at the specified USB port

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem(); // Drivetrain subsystem
  // public static final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(); // Intake subsystem
  // public static final CoralElevatorSubsystem m_CoralElevatorSubsystem = new CoralElevatorSubsystem(); // Elevator subsystem
  // public static final PowerSubsystem m_powerSubsystem = new PowerSubsystem(); // Power subsystem for interacting with the Rev PDH
  // public static final VisionSubsystem m_visionSubsystem = new VisionSubsystem(); // Subsystem for interacting with Photonvision
  // public static final LEDSubsystem m_LEDSubsystem = new LEDSubsystem(); // Subsytem for controlling the REV Blinkin LED module
  
  double goalAngle;
  double start;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    configureButtonBindings(); // Bind our commands to physical buttons on a controller

    // Add our Autonomous Routines to the chooser //
		autonChooser.setDefaultOption("Do Nothing", new InstantCommand());
    //autonChooser.addOption("Drive 1 Meter", new Drive1MeterAuto());
    autonChooser.addOption("Square Autonomous", new WaitCommand(1.0));
		SmartDashboard.putData("Auto Mode", autonChooser);

    // Zero the gyroscope and reset the drive encoders
    //m_driveSubsystem.zeroGyro();
    m_driveSubsystem.resetEncoders();
    //m_LEDSubsystem.setLEDMode(LEDMode.DISABLED);

LiftCam = CameraServer.startAutomaticCapture(0);

  }




  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods. This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //SmartDashboard.putNumber("Gyroscope Pitch", m_driveSubsystem.getPitch());
    //SmartDashboard.putNumber("Gyroscope Yaw", m_driveSubsystem.getYaw());
    //SmartDashboard.putNumber("Gyroscope Roll", m_driveSubsystem.getRoll());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    System.out.println("ROBOT DISABLED");
    //m_LEDSubsystem.setLEDMode(LEDMode.DISABLED);
  }

  /** This function is called continuously after the robot enters Disabled mode. */
  @Override
  public void disabledPeriodic() {

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //System.out.println("AUTONOMOUS MODE STARTED");
    start = Timer.getFPGATimestamp();
   // m_autonomousCommand = autonChooser.getSelected();
    
    // Zero the gyrodcope and reset the drive encoders
    //m_driveSubsystem.zeroGyro();
    //m_driveSubsystem.resetEncoders();

    // schedule the selected autonomous command
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }

    // Set the LED pattern for autonomous mode
    //m_LEDSubsystem.setLEDMode(LEDMode.AUTO);

    // Set Elevator/End Effector inital preset
    // m_CoralElevatorSubsystem.climbNeutral();
    // m_CoralElevatorSubsystem.armInitial();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (Timer.getFPGATimestamp() - start<3.0) {
      m_driveSubsystem.drive(0.1, 0, 0);
  
    }else {
      m_driveSubsystem.stop();
    }
  }

  @Override
  public void teleopInit() {
    System.out.println("TELEOP MODE STARTED");

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this if statement or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Zero the gyroscope and reset the drive encoders
    //m_driveSubsystem.zeroGyro();
    m_driveSubsystem.resetEncoders();

    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        // Set the LED pattern for teleop mode
      //m_LEDSubsystem.setLEDMode(LEDMode.TELEOPRED);

      }
      if (ally.get() == Alliance.Blue) {
        //m_LEDSubsystem.setLEDMode(LEDMode.TELEOPBLUE);
      }
    }

    //goalAngle = m_driveSubsystem.getGyroAngle();

    // // Set Elevator/End Effector inital preset
    // m_CoralElevatorSubsystem.climbNeutral();
    // m_CoralElevatorSubsystem.armInitial();

    // m_intakeSubsystem.setDefaultCommand(new IntakeManualControl());
    //m_CoralElevatorSubsystem.setDefaultCommand(new CoralElevatorMoveCommand());
  }

 // private final int BUTTON_NUMBER = 1; // Define the button number you want to use
  //private TalonFX motor;
  TalonFX motor = new TalonFX(4); // Shooter?
  TalonFX climber = new TalonFX(41); //climber
  
  
  public void teleopPeriodic() {
    double moveForward = MathUtil.applyDeadband(-controller.getRawAxis(1), 0.25) * 0.6; // Get forward/backward input, put negative before controller after driving
    double strafeRight = MathUtil.applyDeadband(controller.getRawAxis(0), 0.25) * 0.6; // Get strafing input
    double rotate = MathUtil.applyDeadband(controller.getRawAxis(4), 0.25) * 0.6; // Get rotation input
   m_driveSubsystem.drive(moveForward, strafeRight, rotate);
  

   if (controller.getRawButton(6)) { // 6 = right bumper
    // Move the motor forward when the button is pressed
    motor.set( -0.1); // Adjust the speed (0.0 to 1.0) as needed
} else {
    // Stop the motor when the button is released
    motor.set( 0.0);
}

if (controller.getRawButton(1)) { 
  // Move the motor backward when button 1 is pressed
  climber.set(-0.4); // Adjust the speed (0.0 to 1.0) as needed
} else if (controller.getRawButton(2)) { 
  // Move the motor forward when button 2 is pressed
  climber.set(0.4); // Adjust the speed (0.0 to -1.0) as needed
} else {
  // Stop the motor when neither button is pressed
  climber.set(0.0);
}


  // class robot {
  //   private TalonFX motor;
  //   private static final double TICKS_PER_REVOLUTION = 2048; // TalonFX encoder resolution (typically 2048 ticks per revolution)
  //   private static final double MAX_RPM = 5000; // Max RPM for motor (adjust according to your motor specs)
    
  //   public char TalonFXRotation(int motorID) {
  //       // Initialize TalonFX motor
  //       motor = new TalonFX(41);

  //       // Set up the motor's sensor for position control
  //       motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
  //       motor.setSensorPhase(true);  // Set the sensor phase (may need adjustment depending on wiring)
  //   }

//     // Method to perform a set number of rotations
//     public void rotateTurns(double numberOfTurns) {
//         // Convert the number of turns to ticks
//         double targetPosition = .5 * 2048;
        
//         // Set the motor to position control mode and set target position
//         motor.set(ControlMode.Position, targetPosition);
//     }

//     public void main(String[] args) {
//         TalonFXRotation robotMotor = new TalonFXRotation(1); // Motor ID 1 as an example
        
//         // Rotate the motor by 5 turns
//         robotMotor.rotateTurns(5);
//     }
// }


    // Log controller inputs to SmartDashboard
    // SmartDashboard.putNumber("Controller: Right Trigger", controller.getRawAxis(Constants.RIGHT_TRIGGER_AXIS));
    // SmartDashboard.putNumber("Controller: Left Trigger", controller.getRawAxis(Constants.LEFT_TRIGGER_AXIS));
    // SmartDashboard.putBoolean("Controller: Right Bumper", controller.getRawButton(Constants.RIGHT_BUMPER));
    // SmartDashboard.putBoolean("Controller: Left Bumper", controller.getRawButton(Constants.LEFT_BUMPER));
    // SmartDashboard.putBoolean("Controller: X Button", controller.getRawButton(Constants.X_BUTTON));
    // SmartDashboard.putBoolean("Controller: Y Button", controller.getRawButton(Constants.Y_BUTTON));
    // SmartDashboard.putBoolean("Controller: B Button", controller.getRawButton(Constants.B_BUTTON));
    // SmartDashboard.putBoolean("Controller: A Button", controller.getRawButton(Constants.A_BUTTON));
    // SmartDashboard.putNumber("Controller: Left Joystick X Axis", controller.getRawAxis(Constants.LEFT_HORIZONTAL_JOYSTICK_AXIS));
    // SmartDashboard.putNumber("Controller: Left Joystick Y Axis", controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS));
    // SmartDashboard.putNumber("Controller: Right Joystick X Axis", controller.getRawAxis(Constants.RIGHT_HORIZONTAL_JOYSTICK_AXIS));
    // SmartDashboard.putNumber("Controller: Right Joystick Y Axis", controller.getRawAxis(Constants.RIGHT_VERTICAL_JOYSTICK_AXIS));

  // if (Robot.manualDriveControl) {
  //   double ySpeed = controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS);
  //   double xSpeed = -controller.getRawAxis(Constants.LEFT_HORIZONTAL_JOYSTICK_AXIS);
  //   double zSpeed = -controller.getRawAxis(Constants.RIGHT_HORIZONTAL_JOYSTICK_AXIS);
    
  //   // Speed limits
  //   ySpeed = Math.max(Math.min(ySpeed, 0.4), -0.4);
  //   xSpeed = Math.max(Math.min(xSpeed, 0.4), -0.4);
  //   zSpeed = Math.max(Math.min(zSpeed, 0.4), -0.4);

  //   if (Math.abs(zSpeed) > 0.01) { // If we are telling the robot to rotate, then let it rotate
	// 		// m_driveSubsystem.driveCartesian(ySpeed, xSpeed, zSpeed, m_driveSubsystem.getRotation2d()); // field-relative
  //     m_driveSubsystem.driveCartesian(ySpeed, xSpeed, zSpeed); // robot-relative
	// 		//goalAngle = m_driveSubsystem.getGyroAngle();
	// 	}
	// 	else { // Otherwise, use the gyro to maintain our current angle
	// 		//double error = m_driveSubsystem.getGyroAngle() - goalAngle;
			
	// 	//double correction = Constants.GYRO_TURN_KP * error;
  //     //if (Math.abs(correction) > Constants.MAX_POWER_GYRO) { // Maximum value we want
  //       //correction = Math.copySign(Constants.MAX_POWER_GYRO, correction);
  //     }
			
	// 		// m_driveSubsystem.driveCartesian(ySpeed, xSpeed, -1 * correction, m_driveSubsystem.getRotation2d()); // field-relative
  //     //m_driveSubsystem.driveCartesian(ySpeed, xSpeed, -1 * correction); // robot-relative
  //     }
  //   {  //put else before this bracket if we use gyro
  //     //goalAngle = m_driveSubsystem.getGyroAngle();
		}
  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    System.out.println("TEST MODE STARTED");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or JEDI LIKES BBC onse of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} 
   * or {@link XboxController}), and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.Trigger}.
   */
  private void configureButtonBindings() {
    // Intake Controls //
    // new Trigger(() -> controller.getRawButton(Constants.RIGHT_BUMPER)).whileTrue(new IntakeSetBarPowerCommand(Constants.INTAKE_BAR_SPEED)); // Intake 
    // new Trigger(() -> controller.getRawButton(Constants.LEFT_BUMPER)).whileTrue(new IntakeSetBarPowerCommand(-Constants.INTAKE_BAR_SPEED)); // Outake 

    // // new Trigger(() -> controller.getRawButton(Constants.A_BUTTON)).onTrue(new IntakeSetArmPositionCommand(Constants.HOLD_ALGAE_POSITION)); // Set arm position
    // // new Trigger(() -> controller.getRawButton(Constants.B_BUTTON)).onTrue(new IntakeSetArmPositionCommand(Constants.HOLD_CORAL_POSITION)); // Set arm position
    // // new Trigger(() -> controller.getRawButton(Constants.Y_BUTTON)).onTrue(new IntakeSetArmPositionCommand(Constants.PICK_UP_ALGAE_POSITION)); // Set arm position
    // // new Trigger(() -> controller.getRawButton(Constants.X_BUTTON)).onTrue(new IntakeSetArmPositionCommand(Constants.PICK_UP_CORAL_POSITION)); // Set arm position

    // new Trigger(() -> controller.getRawButton(Constants.RIGHT_BUMPER)).whileTrue(new IntakePickUpAlgaeCommand()); // Pick Up Algae
    // new Trigger(() -> controller.getRawButton(Constants.RIGHT_BUMPER)).onFalse(new IntakeSetArmPositionCommand(Constants.HOLD_ALGAE_POSITION));
    // new Trigger(() -> controller.getRawButton(Constants.RIGHT_TRIGGER_BUTTON)).whileTrue(new IntakePickUpCoralCommand()); // Pick Up Coral
    // new Trigger(() -> controller.getRawButton(Constants.RIGHT_TRIGGER_BUTTON)).onFalse(new IntakeSetArmPositionCommand(Constants.HOLD_ALGAE_POSITION));

    // // Coral Elevator Controls //
    // new Trigger(() -> controller.getRawButton(Constants.PREV_BUTTON)).whileTrue(new CoralElevatorWheelMoveCommand(-Constants.WHEEL_SPEED)); // Wheel Outtake Manual
    // new Trigger(() -> controller.getRawButton(Constants.START_BUTTON)).whileTrue(new CoralElevatorWheelMoveCommand(Constants.WHEEL_SPEED)); // Weel Intake Manual
    // //new POVButton(controller, 0).onTrue(new CoralElevatorSetPositionArmCommand(m_CoralElevatorSubsystem.arm_max)); // Score Mid Preset
    // new POVButton(controller, 90).onTrue(new CoralElevatorSetPositionArmCommand(21.33)); // Score High Preset
    // new POVButton(controller, 180).onTrue(new CoralElevatorSetPositionArmCommand(29.4)); //  Intake Preset
    // //new POVButton(controller, 270).onTrue(new CoralElevatorSetPositionArmCommand(m_CoralElevatorSubsystem.arm_max)); // Score Low Preset

    // // Test Controls //
    // new Trigger(() -> controller.getRawButton(Constants.A_BUTTON)).whileTrue(new DriveToTrackedTargetCommand(1)); // Track AprilTag
  }

}



