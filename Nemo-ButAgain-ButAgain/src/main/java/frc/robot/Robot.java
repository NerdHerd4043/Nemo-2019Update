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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.SPI;

import jaci.pathfinder.*;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;
// import edu.wpi.first.wpilibj.Notifier;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.*;
import edu.wpi.first.cameraserver.CameraServer;

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

  // private static final double wheelbase_width = 0.635;

  // private static final String k_path_name = "Straight";

  // private Notifier m_follower_notifier;

  public static AHRS navX;

  public static DriveTrain driveTrain;
	public static Intake intake;
  public static Hopper hopper;
	
	public static DigitalInput arduinoDIOLeft;
	public static DigitalInput arduinoDIORight;
	
	public static boolean ballColorLeft;
	public static boolean ballColorRight;
	
	public static String selectedProfile;
	
	public static boolean teamColor;
	
	public static boolean upDown = true;
  
  public static final int ticksPerRev = 262;
  public static final int max_velocity = 3;

  public int distanceInTicks = 568 * 8;

  EncoderFollower left;
  EncoderFollower right;
  Trajectory trajectory;
  TankModifier modifier;

  Preferences prefs;

  public static OI m_oi;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() { 
    // Generate the trajectory
    
    // Trajectory trajectory = PathfinderFRC.getTrajectory("Straight.pf1.csv");
    // TankModifier modifier = new TankModifier(trajectory).modify(0.5);

    // EncoderFollower left = new EncoderFollower(modifier.getLeftTrajectory());
    //   left.configureEncoder(RobotMap.motorBL.getSelectedSensorPosition(), ticksPerRev, 0.1524);
    //   left.configurePIDVA(1.0, 0.0, 0.0, 1 / max_velocity, 0);

    // EncoderFollower right = new EncoderFollower(modifier.getRightTrajectory());
    //   right.configureEncoder(RobotMap.motorBL.getSelectedSensorPosition(), ticksPerRev, 0.1524);
    //   right.configurePIDVA(1.0, 0.0, 0.0, 1 / max_velocity, 0);


    navX = new AHRS(SPI.Port.kMXP); /* Alternatives:  SPI.Port.kMXP, I2C.Port.kMXP or SerialPort.Port.kUSB */

		prefs = Preferences.getInstance();
		selectedProfile = prefs.getString("DriverName", "ethan");
    
    CameraServer.getInstance().startAutomaticCapture();

		driveTrain = new DriveTrain();
		intake = new Intake(); 
		hopper = new Hopper();
		arduinoDIOLeft = new DigitalInput(0);
		arduinoDIORight = new DigitalInput(1);

    m_oi = new OI();

    System.out.println(selectedProfile);

    RobotMap.motorBL.setSelectedSensorPosition(0, 0, 10);
    RobotMap.motorBR.setSelectedSensorPosition(0, 0, 10);
    

    //m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
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
    // 3 Waypoints
    Waypoint[] points = new Waypoint[] {
      new Waypoint(0, 0, 0),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
      new Waypoint(10, 10, 90),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
      // Waypoint @ x=0, y=0,   exit angle=0 radians
    };

    /* 
      Create the Trajectory Configuration
    
      Arguments:
      Fit Method:          HERMITE_CUBIC or HERMITE_QUINTIC
      Sample Count:       SAMPLES_HIGH (100 000)
                          SAMPLES_LOW  (10 000)
                          SAMPLES_FAST (1 000)
      Time Step:           0.05 Seconds
      Max Velocity:        1.7 m/s
      Max Acceleration:    2.0 m/s/s
      Max Jerk:            60.0 m/s/s/s
    */

      
    
    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
   
    trajectory = Pathfinder.generate(points, config);
    modifier = new TankModifier(trajectory).modify(0.5);

    left = new EncoderFollower(modifier.getLeftTrajectory());
      left.configureEncoder(RobotMap.motorBL.getSelectedSensorPosition(), ticksPerRev, 0.1524);
      left.configurePIDVA(0.005, 0.8, 0.0, 1 / max_velocity, 0);

    right = new EncoderFollower(modifier.getRightTrajectory());
      right.configureEncoder(RobotMap.motorBL.getSelectedSensorPosition(), ticksPerRev, 0.1524);
      right.configurePIDVA(0.005, 0.8, 0.0, 1 / max_velocity, 0);


    // m_follower_notifier = new Notifier(this::followPath);
    // m_follower_notifier.startPeriodic(trajectory.get(0).dt);

    m_autonomousCommand = m_chooser.getSelected();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector","Default");
     * switch(autoSelected) { 
     * case "My Auto": 
     *  autonomousCommand = new MyAutoCommand(); 
     *  break; 
     * case "Default Auto": 
     * default:
     *  autonomousCommand = new ExampleCommand(); 
     *  break;
     * }
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

  private void followPath() {

    // if (left.isFinished() || right.isFinished()) {
    //   m_follower_notifier.stop();
    // } else {
      intake.stopAll();
      double l = left.calculate(RobotMap.motorBL.getSelectedSensorPosition());
      double r = right.calculate(RobotMap.motorBR.getSelectedSensorPosition());
  
      double gyro_heading = navX.getAngle();    // Assuming the gyro is giving a value in degrees
      double desired_heading = Pathfinder.r2d(left.getHeading());  // Should also be in degrees
  
      double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
      double turn = 0.8 * (-1.0/80.0) * angleDifference;

      System.out.println("l = " + l);
      System.out.println("r = " + r);

      
      driveTrain.diffDrive.tankDrive(l + turn, r - turn);
      // setLeftMotors(l + turn);
      // setRightMotors(r - turn);
    // }
     
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

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    followPath();
    // autoRoutineStraight();
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
		
		System.out.println(selectedProfile);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    ballColorLeft = arduinoDIOLeft.get();
		ballColorRight = arduinoDIORight.get();
		
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
  @Override
  public void testPeriodic() {

    System.out.println("L: " + RobotMap.motorBL.getSelectedSensorPosition(0));
		System.out.println("R: " + RobotMap.motorBR.getSelectedSensorPosition(0));
  }
}
