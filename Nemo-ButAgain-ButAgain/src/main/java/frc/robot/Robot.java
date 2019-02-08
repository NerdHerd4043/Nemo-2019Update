/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.*;

// import jaci.pathfinder.Pathfinder;
// import jaci.pathfinder.PathfinderFRC;
// import jaci.pathfinder.Trajectory;
// import jaci.pathfinder.followers.EncoderFollower;

import frc.robot.subsystems.*;

import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // private static final int k_ticks_per_rev = 20;
  // private static final double k_wheel_diameter = 6.0;
  // private static final double k_max_velocity = 10;  

  // private static final String k_path_name = "Straight";

  public static AHRS ahrs;
  double last_world_linear_accel_x;
  double last_world_linear_accel_y;

  final static double kCollisionThreshold_DeltaG = 0.4f;

  ShuffleboardTab tab;

  public static DriveTrain driveTrain;
	public static Intake intake;
	public static Hopper hopper;
  
  public static boolean driveMode;
	public static boolean ballColorLeft;
	public static boolean ballColorRight;
	
	public static String selectedProfile;
	
	public static boolean teamColor;
	
	public static boolean upDown = true;
	
  public int distanceInTicks = 568 * 8;

  public static AnalogInput lineFollowLeft;
  public static AnalogInput lineFollowRight;

  int rawReadLeft;
  int rawReadRight;
  int rightThreshold = 1700;
  int leftThreshold = 2600;
  boolean leftTriggered = false;
  boolean rightTriggered = false;
  
  Preferences prefs;

  public static NetworkTableEntry arcadeSpeed;
  public static NetworkTableEntry arcadeTurn;
  public static NetworkTableEntry tankLeft;
  public static NetworkTableEntry tankRight;
  public static NetworkTableEntry squaredIn;
  public static NetworkTableEntry arcadeDrive;

  public static OI m_oi;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() { 

    maxJerkX = 0;
    maxJerkY = 0;

    ahrs = new AHRS(SPI.Port.kMXP); /* Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP or SerialPort.Port.kUSB */

    prefs = Preferences.getInstance();
    prefs.putString("DriverName", "nick");
    selectedProfile = prefs.getString("DriverName", "ethan");

    ShuffleboardTab shuffTab = Shuffleboard.getTab("Drive");

    // shuffTab
    //   .add("Max Speed", 1)
    //   .withWidget(BuiltInWidgets.kNumberSlider)
    //   .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
    //   .getEntry();

    arcadeSpeed = shuffTab
      .getLayout("ArcadeTuner", BuiltInLayouts.kList)
      .withSize(2, 2)
      .withPosition(0, 0)
      .withProperties(Map.of("label position", "TOP"))
      .add("Max Speed", 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1))
      .getEntry();

    arcadeTurn = shuffTab
      .getLayout("ArcadeTuner", BuiltInLayouts.kList)
      .withSize(2, 2)
      .withPosition(0, 0)
      .withProperties(Map.of("label position", "TOP"))
      .add("Max Turn", 1) 
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1))
      .getEntry();

    tankLeft = shuffTab
      .getLayout("TankTuner", BuiltInLayouts.kList)
      .withSize(2, 2)
      .withPosition(2, 0)
      .withProperties(Map.of("label position", "TOP"))
      .add("Max Left", 1) 
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1))
      .getEntry();

    tankRight = shuffTab  
      .getLayout("TankTuner", BuiltInLayouts.kList)
      .withSize(2, 2)
      .withPosition(2, 0)
      .withProperties(Map.of("label position", "TOP"))
      .add("Max Right", 1) 
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1))
      .getEntry();

    squaredIn = shuffTab
      .getLayout("Settings", BuiltInLayouts.kList)
      .withSize(2, 1)
      .withPosition(0, 2)
      .withProperties(Map.of("labelposition", "HIDDEN"))
      .add("Squared Inputs", true)
      .withWidget(BuiltInWidgets.kToggleButton)
      .getEntry();

      arcadeDrive = shuffTab
      .getLayout("Settings", BuiltInLayouts.kList)
      .withSize(2, 1)
      .withPosition(0, 2)
      .withProperties(Map.of("label position", "HIDDEN"))
      .add("Arcade Drive", true) 
      .withWidget(BuiltInWidgets.kToggleButton)
      .getEntry();

    driveMode = arcadeDrive.getBoolean(true);

    CameraServer.getInstance().startAutomaticCapture();

    // tab = Shuffleboard.getTab("SmartDashboard");
    // NetworkTableEntry maxSpeed =
    //   tab.add("Max Speed", 1)
    //      .getEntry(); 

    lineFollowLeft = new AnalogInput(2);
    lineFollowRight = new AnalogInput(1);

		driveTrain = new DriveTrain();
		intake = new Intake(); 
		hopper = new Hopper();

    m_oi = new OI();

    // System.out.println(selectedProfile);

    RobotMap.motorBL.setSelectedSensorPosition(0, 0, 10);
    RobotMap.motorBR.setSelectedSensorPosition(0, 0, 10);
    

    // m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    //SmartDashboard.putData("Auto mode", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    if (driveMode != arcadeDrive.getBoolean(true)) {
      driveMode = arcadeDrive.getBoolean(true);
      Robot.driveTrain.setDriveMode(driveMode);
      System.out.println("ArcadeDrive = " + driveMode);
    }

    SmartDashboard.putData("Gyro", ahrs);
    SmartDashboard.putNumber("Left Encoder", RobotMap.motorBL.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Right Encoder", RobotMap.motorBR.getSelectedSensorPosition(0));
   
  }

  /** 
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    RobotMap.motorBL.setSelectedSensorPosition(0, 0, 10);
		RobotMap.motorBR.setSelectedSensorPosition(0, 0, 10);
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    // Trajectory left_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".left");
    // Trajectory right_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".right");

    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }

    RobotMap.motorBR.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder, 0, 10);
		RobotMap.motorBL.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder, 0, 10);
		RobotMap.motorBR.setSelectedSensorPosition(0, 0, 10);
		RobotMap.motorBL.setSelectedSensorPosition(0, 0, 10);
  }

  void autoRoutineStraight() {
		/* for (i in 1..25) {
		 * 	Drive forward 1 crate length
		 * 	Dump ball in crate
		 * }
		 */
		System.out.println("Starting autoRoutine");
		for (int i = 1; i < 2; i++) {
			driveTrain.diffDrive.arcadeDrive(0, 0);
			//intake.intakeUp();
			hopper.dumpLeft();
			hopper.dumpRight();
			System.out.println("Dump");
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				System.out.println("INTERRUPTED");
			}
			hopper.closeLeft();
			hopper.closeRight();
			System.out.println("Close");
			System.out.println("Inside for loop");
//			while(RobotMap.motorBR.getSelectedSensorPosition(0) < distanceInTicks * i) {
//				System.out.println("Inside while loop");
//				double diff = (RobotMap.motorBR.getSelectedSensorPosition(0) + RobotMap.motorBL.getSelectedSensorPosition(0)) / 300d;
//				driveTrain.diffDrive.arcadeDrive(.5, diff);
//			}
		}
  }
  
  void autoRoutineLineFollow() {
    rawReadLeft = lineFollowLeft.getValue();
    rawReadRight = lineFollowRight.getValue();


    if (leftTriggered && rightTriggered) {
      if (rawReadRight > rightThreshold) {
        driveTrain.diffDrive.arcadeDrive(.5, .25);
        System.out.println("Turning right");
      } else if (rawReadLeft > leftThreshold) {
        System.out.println("Turning left");
        driveTrain.diffDrive.arcadeDrive(.5, -.55);
      } else {
        System.out.println("AAAAAAAAA");
        driveTrain.diffDrive.arcadeDrive(.4, -.15);
      }
    }
    else if (rawReadRight > rightThreshold) {
      rightTriggered = true;
      driveTrain.diffDrive.arcadeDrive(.5, -.15);
    } if (rawReadLeft > leftThreshold) {
      leftTriggered = true;
      driveTrain.diffDrive.arcadeDrive(.5, -.15);
    } else {
      driveTrain.diffDrive.arcadeDrive(.5, -.15);
    }

    // System.out.print("L: " + rawReadLeft);
    // System.out.println("R: " + rawReadRight);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    intake.stopAll();
    //autoRoutineStraight();
    autoRoutineLineFollow();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    prefs = Preferences.getInstance();
		selectedProfile = prefs.getString("DriverName", "ethan");
		teamColor = prefs.getBoolean("ColorPicker", true);
		
		// System.out.println(selectedProfile);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
		
//		if (teamColor) {				//If the wanted color is red
//			if (ballColorLeft){			//If the actual left color is red
//				sorter.leftUnYeet();	//Keep the left sorter going up
//			} else if (!ballColorLeft) {//If the actual left color is blue
//				sorter.leftYeet(); 		//yeet the left side
//			}
//			
//			if (ballColorRight){		//If the actual right color is red
//				sorter.rightUnYeet();	//Keep the right sorter going up
//			} else if (!ballColorRight) {//If the actual right color is blue
//				sorter.rightYeet(); 	//yeet the right side
//			}
//		}
		
		if (OI.driveStick.getRawAxis(2) != 0){
			intake.intakeDown();
			upDown = true;
		} else if (OI.driveStick.getRawAxis(3) != 0){
			intake.intakeUp();
			upDown = true;
		} else if (upDown){
			intake.slowDown();
		} else {
			intake.stopAll();
		}
  }

  /**
   * This function is called periodically during test mode.
   */
double maxJerkX = 0;
double maxJerkY = 0;

  @Override
  public void testPeriodic() {
    boolean collisionDetected = false;

    double curr_world_linear_accel_x = ahrs.getWorldLinearAccelX();
    double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
    last_world_linear_accel_x = curr_world_linear_accel_x;
    double curr_world_linear_accel_y = ahrs.getWorldLinearAccelY();
    double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
    last_world_linear_accel_y = curr_world_linear_accel_y;
    
    if ( ( Math.abs(currentJerkX) > kCollisionThreshold_DeltaG ) ||
          ( Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) ) {
        collisionDetected = true;
    }

    if(currentJerkX > maxJerkX) {
      maxJerkX = currentJerkX;
    }
    if (currentJerkY > maxJerkY) {
      maxJerkY = currentJerkY;
    }
    System.out.print(maxJerkX + "  ");
    System.out.println(maxJerkY);
  }
}
