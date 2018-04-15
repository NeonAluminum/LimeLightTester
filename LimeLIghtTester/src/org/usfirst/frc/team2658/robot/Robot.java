/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2658.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	private final int L_MOTOR_PORT = 3;
	private final int R_MOTOR_PORT = 4;
	private final int JOYSTICK_PORT = 0;
	private final int AXIS_PORT1 = 0;
	private final int AXIS_PORT2 = 0;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	
	public Talon l_motor, r_motor; // drive motors
	public DifferentialDrive mainDrive; // drive 
	public Joystick jstick; //joystick
	public Timer timer = new Timer(); // timer for autonomous
	
	public NetworkTable table; // main table from limelight 
	public NetworkTableEntry tx, ty, ta, tv, light; // network table entries from the main network table
	double x = 0, y = 0 , area = 0, targetFound = 0; // x, y, area, and targetFound values from the network table
									//target found 0 = false, 1 = true
	public int controlConstant = 27; // divisor to decrease speed when target is close
	public double turnValue = 0; // turn speed
	
	//see if button is pressed to init camera controlled driving
	boolean buttonVal;
	
	@Override
	public void robotInit() {
		l_motor = new Talon(L_MOTOR_PORT);// init all the motors and the drive train
		r_motor = new Talon(R_MOTOR_PORT);
		mainDrive = new DifferentialDrive(l_motor, r_motor);
		jstick = new Joystick(0);
		//get default instance of the limelight network table
		table = NetworkTableInstance.getDefault().getTable("limelight");
		tx = table.getEntry("tx"); // get all the vals
		ty = table.getEntry("ty");
		ta = table.getEntry("ta");
		tv = table.getEntry("tv");
		//attempt to turn off light on limelight but it doesnt work :(
		NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);

		
		
	}

	
	@Override
	public void autonomousInit() {
		//test autonomous thing
		driveForward(2.7, 0.6);
		pauseRobot(0.5);
		turnRTimer(1.3);
		driveToBox();
		
		
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		
		
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		// update the limelight values (x, y, area, targetFound)
		updateLimeLight();
		//tester thing to see control constant
		SmartDashboard.putNumber("CONTROL CONSTANT", controlConstant);
		
		//drive using an xbox controller yay
		int rstick = 0, lstick = 0;
		
		if (0 < jstick.getRawAxis(1)) {
			rstick = 1;
		} else {
			rstick = -1;
		}
		if (0 < jstick.getRawAxis(5)) {
			lstick = 1;
		} else {
			lstick = -1;
		}
		
		// DRIVE CONTROL PART
		//See if the button is pressed, if yes switch control to limelight
		buttonVal = jstick.getRawButton(1);
		
		if(buttonVal) {
			// set control constant to the value on smart dash (mostly for testing)
			controlConstant = (int) SmartDashboard.getNumber("CONTROL CONSTANT", 50);
			//scale down the turn value so that it is between -1 and 1
			// if we divide the x values by a control constant we can lower the turn speed
			// the robot is closer to the target
			turnValue = x / -controlConstant;
			SmartDashboard.putNumber("TURN VALUE", turnValue); // testing smartdashboard value
			mainDrive.arcadeDrive(-0.6, turnValue + 0.1); // drive to the box
			//random +0.1 because our robot is bad and it curves
			
		}
		
		else {
			SmartDashboard.putNumber("TURN VALUE", turnValue);
			turnValue = 0; // reset turn value if button is not held
			// this joystick controlled tank drive MUST be in an else statement or else it "overides" the 
			// arcade drive statement when the button is pressed
			mainDrive.tankDrive(Math.pow(jstick.getRawAxis(1), 2) * rstick,  Math.pow(jstick.getRawAxis(5), 2) * lstick);
		}
			
		
	
		
		

		
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		
	
	
	
	}
	
	/*
	 * This method updates the values from the limelight networktables
	 */
	public void updateLimeLight() {
		x = tx.getDouble(0);
		y = ty.getDouble(0);
		area = ta.getDouble(0);
		targetFound = tv.getDouble(0);
		
		SmartDashboard.putNumber("X", x);
		SmartDashboard.putNumber("Y", y);
		SmartDashboard.putNumber("Area", area);
		SmartDashboard.putNumber("Target Acquired: ", targetFound);
		
	}
	
	
	/*
	 * Drive forward for a set amount of time and set speed.
	 */
	public void driveForward(double limit, double power) {
		// cross autoline using timer
    	timer.start();
    	// while timer is less than limit
    	while(timer.get() <= limit) {
    		// lspeed, rspeed
    		mainDrive.tankDrive(-power-0.089, -power);
    		
    	}
    	timer.stop();
        timer.reset();
        mainDrive.arcadeDrive(0, 0);
	}
	/*
	 * pause robot for a set amount of time
	 */
    public void pauseRobot (double time) {
    	timer.reset();
    	timer.start();
    	while (timer.get() < time) {
    		
    	}
    	timer.stop();
    	timer.reset();
    }
    /*
     * turn left for a set amount of time
     */
    public void turnLTimer(double time) {
    	timer.start();
    	while(timer.get() <= time) {
    		mainDrive.arcadeDrive(0, 0.7);
    	}
    	timer.stop();
        timer.reset();
    }
    /*
     * turn right for a set amount of time
     */
    public void turnRTimer(double time) {
    	timer.start();
    	while(timer.get() <= time) {
    		mainDrive.arcadeDrive(0, -0.5);
    	}
    	timer.stop();
        timer.reset();
    }
    /*
     * drive to box using limelight
     */
    public void driveToBox() {
    	while(area <= 31) {
    		updateLimeLight();
    		controlConstant = 35;
    		turnValue = x / -controlConstant;
			SmartDashboard.putNumber("TURN VALUE", turnValue);
			mainDrive.arcadeDrive(-0.45, turnValue);
    	}
    	mainDrive.arcadeDrive(0, 0);
    		
    		
    }
    	
    
}
