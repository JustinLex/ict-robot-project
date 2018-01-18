package myPackage;


import java.util.concurrent.locks.ReentrantLock;

import lejos.hardware.Audio;
import lejos.hardware.Key;
import lejos.hardware.ev3.EV3;
import lejos.hardware.ev3.LocalEV3;

import java.io.File;

import lejos.hardware.lcd.LCD;

import lejos.hardware.motor.*;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.internal.ev3.EV3Audio;
import lejos.robotics.DirectionFinder;
import lejos.robotics.Gyroscope;
import lejos.robotics.GyroscopeAdapter;
import lejos.robotics.RangeFinder;
import lejos.robotics.RangeFinderAdapter;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;
import lejos.utility.GyroDirectionFinder;
import lejos.utility.Stopwatch;

public class SumoRobotProgram {
	//TODO: rotate at start, gyroturn, line blocking turn
	// define music files
	
	//ready music
	//final static File jcena = new File("//home/root/rocky.wav");
	//final static File soymiguel = new File("//home/root/rocky.wav");
	
	//battle music
	//final static File guile = new File("//home/root/rocky.wav");
	//final static File nyan = new File("//home/root/rocky.wav");
	
	//victory music
	final static File rocky = new File("//home/root/rocky.wav");
	//final static File nevergive = new File("//home/root/rocky.wav");
	

	static EV3 miguel = LocalEV3.get();
	static EV3Audio soundDaemon = (EV3Audio) miguel.getAudio();
	
	//Define drive motors
	static RegulatedMotor MotorL =  new EV3LargeRegulatedMotor(MotorPort.D);
	static RegulatedMotor MotorR =  new EV3LargeRegulatedMotor(MotorPort.A);
			
	//Initialize chassis model
	static final double tireDiameter = 5.6; //diameter is printed on the side of the wheel 
	static Wheel wheelL = WheeledChassis.modelWheel(MotorL, tireDiameter).offset(-8.25).invert(true);
	static Wheel wheelR = WheeledChassis.modelWheel(MotorR, tireDiameter).offset(8.25).invert(true);
	static Chassis sumochassis = new WheeledChassis(new Wheel[] { wheelL, wheelR }, WheeledChassis.TYPE_DIFFERENTIAL);
	static final byte maxSpeed = 25; //max speed we can go and not overshoot the line
	
	
	//initialize move controller
	static MovePilot miguelPilot = new MovePilot(sumochassis);
	
	//initialize odometer
	static OdometryPoseProvider odometer = new OdometryPoseProvider(miguelPilot);
	
	//Define sensors
	//light sensor
	static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
	static SampleProvider floorReflectivity = colorSensor.getRedMode();

	//scanner platform
	static RegulatedMotor sensorMotor =  new EV3MediumRegulatedMotor(MotorPort.C);
	static SampleProvider USSensor = new EV3UltrasonicSensor(SensorPort.S2).getDistanceMode();
	static RangeFinder USAdapter = new RangeFinderAdapter(USSensor);
	
	//initialize enemy scanner thread
	static ScannerThread scanner = new ScannerThread(sensorMotor, USAdapter);
	static Stopwatch scanStopwatch = new Stopwatch(); //stopwatch used during scanning

	
	//gyro
	//static SampleProvider gyroSensor = new EV3GyroSensor(SensorPort.S4);
	//static Gyroscope gyro = new GyroscopeAdapter(gyroSensor, 500);
	
	//Define detection models
	static PoseProvider OdoPP = sumochassis.getPoseProvider(); //change this
	//static DirectionFinder GyroDF = new GyroDirectionFinder(gyro);
	//augment pp with gyro data, and reset on line hit
	//static Pose heroPose = new Pose(); //position and angle of our robot
	
	//variables about the enemy
	static float enemyDistance = Float.POSITIVE_INFINITY; //meters
	static int enemyAngle; //degrees
	static boolean enemyComing;
	static boolean enemyExists = true;
	
	//variables about current behavior
	static boolean hasMatadored = false;
	static boolean lineHit = false;
	
	public static final byte testMode = 0; //used to put the robot in various test modes
	static final int giveUpTime = 10000; //time in milliseconds robot will search before assuming enemy left
	
	volatile public static boolean robotRunning = true; //used for the scanner thread to tell if robot is running
	
	@SuppressWarnings("unused")
	public static void main(String[] args) {
		
		/*//Hello World!
		LCD.drawString("TUNA OF MAXIMUM JUSTICE", 0, 4);
		Delay.msDelay(1000);
		*/
		if(testMode == -1) {
			//scanner.start();
			//soundDaemon.playSample(rocky, 100);
			//Delay.msDelay(10000);
			//soundDaemon.endPCMPlayback();
			miguelPilot.stop();
			lejos.hardware.Sound.beep();
			Delay.msDelay(5000);
		}
		
		else if(testMode == 1) {
			//Drive forward and back test
			Key enterKey = miguel.getKey("Enter");
			enterKey.waitForPress();
			Delay.msDelay(500);
			miguelPilot.travel(100);
			miguelPilot.travel(-100);
		}
		
		else if(testMode == 2) {
			Key enterKey = miguel.getKey("Enter");
			enterKey.waitForPress();
			Delay.msDelay(500);
			//miguelPilot.rotate(360);
			//miguelPilot.rotate(-360);
			//gyroRotate(90);
			//gyroRotate(-90);
		}
			
		else if(testMode == 4) {
			//Line Detector Test
			miguelPilot.setLinearSpeed(maxSpeed); 
			Key enterKey = miguel.getKey("Enter");
			enterKey.waitForPress();
			Delay.msDelay(500);
			miguelPilot.travel(100, true);
			Delay.msDelay(10);
			while(miguelPilot.isMoving()) {
				Delay.msDelay(1);
				if(checkForLine()) {
					miguelPilot.stop();
					lineHit = true;
					lejos.hardware.Sound.twoBeeps();
					Delay.msDelay(100);
					miguelPilot.rotate(90);
				}
			}
			if (!lineHit) lejos.hardware.Sound.beep();
			Delay.msDelay(1000);
		}
		
		else if(testMode == 5) { 
			//Scanner Test
			scanner.start(); //start enemy scanner
			Delay.msDelay(120000);
			robotRunning = false;
		}
		
		else if(testMode == 6) { 
			//Full Detection Test (check blind spots)
			//wait until we see enemy, times out after 3 sweeps or 10 seconds has elapsed for search
			scanner.start(); //start scanner thread
			
			//reset scanner's hit timer before we start searching, this lets us know how long we've searched without a hit
			scanner.resetTimeSinceLastHit(); 
			
			while(enemyExists) {
				scanStopwatch.reset();
				while(scanner.getDistance() == Float.POSITIVE_INFINITY && scanStopwatch.elapsed() < 1000) {
					Delay.msDelay(50); //check every 50ms
				}
				if(scanner.getDistance() < Float.POSITIVE_INFINITY ) {
					//scan success!
					lejos.hardware.Sound.twoBeeps();
					Delay.msDelay(5000);
				} else {
					//scan failure
					
					if(scanner.getTimeSinceLastHit() > 20000){
						//give up, declare victory
						lejos.hardware.Sound.beepSequenceUp();
						Delay.msDelay(1000);
						enemyExists = false; 						
					} 
					else {
						//spin because enemy might be behind us
						lejos.hardware.Sound.beep();
						int time = scanner.getTimeSinceLastHit();
						LCD.drawString(Integer.toString(time), 4, 4);
						miguelPilot.rotate(-120); 
					}
				}
			}
			Delay.msDelay(1000);
			robotRunning = false;
		}
		
		
		else if(testMode == 7) {
			//Scan+Charge Test NOT IMPLEMENTED
			scanner.start(); //start enemy scanner
			ReentrantLock enemyVarLock = scanner.getVarLock(); //get locker from scanner thread
			scanStopwatch.reset();
			while(scanStopwatch.elapsed() < 30000) {
				enemyVarLock.lock();
				if (scanner.getDistance() < 20) {
					enemyAngle = scanner.getAngle();
					miguelPilot.rotate(scanner.getAngle());
					lejos.hardware.Sound.beepSequenceUp();
					Delay.msDelay(1000);
				}
				else {
					enemyVarLock.unlock();
					Delay.msDelay(50); //check every 50ms
				}
				
			}
			robotRunning = false;
		}
		
		else if(testMode == 8) {
			//Matador Test NOT IMPLEMENTED
			robotRunning = false;
		}
		
		
		else if(testMode == 0) {
			//Main code
			
			miguelPilot.setLinearSpeed(maxSpeed); 
			
			//check for US sensor error before battle
			while(USAdapter.getRange() < 130) { //TODO: temp value
				lejos.hardware.Sound.buzz();
				Delay.msDelay(500);
			}
			
			//notify that robot is done initializing and there are no errors
			lejos.hardware.Sound.beepSequence(); 
			
			//wait for button press
			Key enterKey = miguel.getKey("Enter");
			enterKey.waitForPress();
			
			//start music playback
			/*Thread t = new Thread() {
				@Override
				public void run() {
					//if(Math.random() > 0.5) soundDaemon.playSample(guile, 100);
					//else soundDaemon.playSample(rocky, 100);
					soundDaemon.playSample(rocky, 100);
				}
			};
			t.start();*/
			
			Delay.msDelay(3000); //wait 3 seconds according to regulations
			
			//battle start!
			
			scanner.start(); //start enemy scanner
			ReentrantLock enemyVarLock = scanner.getVarLock(); //get locker object from scanner thread, see scanner thread for reason
			
			miguelPilot.rotate(180);
			
			//[BATTLE LOOP]
			while(true) {
		
				//begin [SEARCH]
				scanner.resetTimeSinceLastHit(); 
				while(true) {
					
					//start scan
					scanStopwatch.reset();
					while(scanner.getDistance() == Float.POSITIVE_INFINITY && scanStopwatch.elapsed() < 2000) {
						Delay.msDelay(50); //check every 50ms
					}
					//scan end
					
					if(scanner.getDistance() < Float.POSITIVE_INFINITY ) {
						//scan success!
						
						//record enemy info from scanner
						enemyVarLock.lock();
						enemyDistance = scanner.getDistance();
						enemyAngle = scanner.getAngle();
						enemyComing = scanner.isEnemyComing();
						enemyVarLock.unlock();
						break;
					} else {
						//scan timeout
						
						if(scanner.getTimeSinceLastHit() > giveUpTime){
							//give up, declare victory
							lejos.hardware.Sound.beepSequenceUp();
							enemyExists = false;
							break;
						} 
						else {
							//spin because enemy might be behind us and scan again
							miguelPilot.rotate(-120); 
						}
					}
				}
				//[SEARCH OVER]
				if(!enemyExists) break; //exit battle loop and do a victory dance

				//[ENEMY KNOWN]
				
				//[MATADOR]
				/*if(enemyComing == true && !hasMatadored) {
					//if enemy is moving at us and we didn't matador last time, do a matador move!
					
					hasMatadored = true; //we will do a matador
					
					//turn perpendicular to enemy
					if (enemyAngle <= 0) miguelPilot.rotate(enemyAngle + 90);
					else miguelPilot.rotate(enemyAngle - 90);
					
					miguelPilot.travel(55, true); //move forward 55 cm to clear the way
					
					//watch out for linehits while moving out of the way
					while(sumochassis.isMoving() && !lineHit) {
						Delay.msDelay(1);
						lineHit = checkForLine();
					}
					if(lineHit) resetPos(); //if we hit the line, reset our position to somewhere better
				
					sumochassis.rotate(180);//look behind us to see where the enemy went
					Delay.msDelay(1000); //wait one second to let enemy pass
				}*/
				
				//[CHARGE]
				//else { 
					//since matador wasn't viable, charge at the enemy
					
					//hasMatadored = false; //we wont do a matador
					
					miguelPilot.rotate(findRealEnemyAngle(enemyDistance, enemyAngle)); //calculate the enemy's real angle to the wheelbase (not angle to sensor)
					//miguelPilot.rotate(enemyAngle);
					miguelPilot.forward(); //charge!
					
					//wait until line lit
					while(!lineHit) {
						Delay.msDelay(1);
						lineHit = checkForLine();
					}
					resetPos(); // reset after line hit
				//}
			}
			// end [BATTLELOOP]
			robotRunning = false; //signal the scanner thread to turn off
			victoryDance(); // no enemy found, so do a victory dance
			
			
		}
		
	}
	
	//METHODS
	
	//Checks to see if we've hit the Edge Line
	static boolean checkForLine() {
		float[] lightSample = new float[1];
		floorReflectivity.fetchSample(lightSample,0);
		if(lightSample[0] < 0.1 ) return true;
		else return false;
	}
	
	//finds angle robot needs to turn to point at enemy
	static int findRealEnemyAngle(float dist, int angle) {
		final int sensorDistance = 8; //distance between wheelbase and US sensor
		//cosine rule to find dist between wheelbase and enemy
		double realDist = Math.sqrt( Math.pow(dist, 2) + Math.pow(sensorDistance, 2) - 2*dist*sensorDistance*Math.cos(Math.toRadians(angle)) );
		//sine rule to find angle from wheelbase to enemy
		double realAngle = Math.acos( dist * Math.sin(Math.toRadians(angle)) / realDist);
		//round off final angle and return it with the same sign as the input angle
		if(angle > 0) return (int) Math.round(Math.toDegrees(realAngle));
		else return (int) -Math.round(Math.toDegrees(realAngle));
	}
	
	//rotate using gyro [not implemented yet]
	/*static void gyroRotate(int degrees) {
		int gyroAngle = gyro.getAngle();
		int targetAngle = gyroAngle + degrees;
		
		if(degrees < 0) {
			miguelPilot.rotateLeft();
			while(gyroAngle > targetAngle && !lineHit) {
				Delay.msDelay(1);
				gyroAngle = gyro.getAngle();
				lineHit = checkForLine();
			}
			if(lineHit) {
				//spin opposite direction if original turn was less than 180, or just return 
				if(degrees < 180) return;
				else{
					gyroRotate(-degrees + (targetAngle - degrees - gyroAngle) );
				}
			}
			miguelPilot.stop();
		}
		
		else if(degrees > 0) {
			miguelPilot.rotateRight();
			while(gyroAngle < targetAngle) {
				Delay.msDelay(1);
				gyroAngle = gyro.getAngle();
			}
			miguelPilot.stop();
		}
	}*/
	
	//Returns the robot to a safe position after hitting Edge Line TODO: implement return to center function?
	static void resetPos() {
		miguelPilot.stop();
		miguelPilot.rotate(180);
		lineHit=false;
	}
	
	//So you think you can dance??
	static void victoryDance() {
		miguelPilot.rotateRight();
		Delay.msDelay(10000);
		//lejos.hardware.Sound.playSample(rocky, 100);
	}
}