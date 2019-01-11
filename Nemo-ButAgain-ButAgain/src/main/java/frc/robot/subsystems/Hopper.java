package frc.robot.subsystems;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */

public class Hopper extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	boolean flase = false;
	
	public void dumpLeft() {
		RobotMap.leftGate.set(true);
	}
	
	public void closeLeft() {
		RobotMap.leftGate.set(false);
	}
	
	public void dumpRight() {
		RobotMap.rightGate.set(true);
	}
	
	public void closeRight() {
		RobotMap.rightGate.set(false);
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

