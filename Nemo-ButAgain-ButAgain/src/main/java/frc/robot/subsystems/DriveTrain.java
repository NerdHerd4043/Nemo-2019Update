package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.commands.Drive;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


/**
 *
 */
public class DriveTrain extends Subsystem {
	public DifferentialDrive diffDrive; 
	double inputSpeed;
	double inputTurn; 
	boolean flase = false;
	
	//These get drawn in from the correct driver profile
	double maxSpeed;
	double maxTurn;
	boolean arcadeTank = true;	//arcade = true ; tank = false
//	String profileSelected = Robot.selectedProfile;
	String profileSelected = "ethan";
	

	public DriveTrain() {
		super();
		diffDrive = new DifferentialDrive(RobotMap.motorFL, RobotMap.motorFR);
		RobotMap.motorBL.follow(RobotMap.motorFL);
		RobotMap.motorBR.follow(RobotMap.motorFR);
		
		RobotMap.motorFR.setSafetyEnabled(false);
		RobotMap.motorFL.setSafetyEnabled(false);
		RobotMap.motorBR.setSafetyEnabled(false);
		RobotMap.motorBL.setSafetyEnabled(false);
		diffDrive.setSafetyEnabled(false);
		
//		switch (profileSelected) {
//		case "ethan" :
//			maxSpeed = 1.0;
//			maxTurn = 1.0;
//			arcadeTank = true;
//			break;
//		case "nick" :
//			maxSpeed = 1.0;
//			maxTurn = 1.0;
//			arcadeTank = true;
//			break;
//		case "meira" :
//			maxSpeed = .9;
//			maxTurn = maxSpeed;
//			arcadeTank = false;
//			break;
//		case "lucca" :
//			maxSpeed = .5;
//			maxTurn = .75;
//			arcadeTank = true;
//			break;
//		}
//			
	}

	public void setDriveMode(boolean mode) {
		arcadeTank = mode;
	}

	public void drive(Joystick joy) {
		inputSpeed = -joy.getRawAxis(1);// * maxSpeed;

		if (arcadeTank) {
			inputTurn = joy.getRawAxis(4);// * maxTurn; 	//For arcade drive
		} else {
			inputTurn = -joy.getRawAxis(5);				//For tank drive
		} 
		drive(inputSpeed, inputTurn);
	}
	
	public void drive(double left, double right) {
		if (arcadeTank) {
			diffDrive.arcadeDrive(left * Robot.arcadeSpeed.getDouble(1.0), right * Robot.arcadeTurn.getDouble(1.0), Robot.squaredIn.getBoolean(true));
		} else {
			diffDrive.tankDrive(left * Robot.tankLeft.getDouble(1.0), right * Robot.tankRight.getDouble(1.0), Robot.squaredIn.getBoolean(true));
		}
	}
	
	// public void drive(double left, double right) {
	// 	diffDrive.arcadeDrive(left, right);
	// }
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new Drive());
    }
}

