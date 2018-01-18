package myPackage;

import java.lang.reflect.Array;
import java.util.concurrent.locks.ReentrantLock;

import lejos.hardware.lcd.LCD;
import lejos.robotics.RangeFinder;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
import lejos.utility.Stopwatch;

public class ScannerThread extends Thread {
//TODO: shutdown, sync rotation better, reset head?
	
	//Devices to use
	RegulatedMotor head;
	RangeFinder sensor;
	
	//enemy variables
	float enemyDist = Float.POSITIVE_INFINITY;
	int enemyAngle = 0;
	boolean enemyComing = false;
	
	final int maxDist = 110; //max distance to detect //TODO: temp value
	final int limit = Math.round(90*29/12); // limit for our motor to turn, in degrees
	Stopwatch hitTimer = new Stopwatch();
	
	//locks to ensure variables line up
	ReentrantLock enemyVarLock = new ReentrantLock(); //locks to ensure dist, angle, and velocity match up
	ReentrantLock timerLock = new ReentrantLock(); //locks to ensure hitTimer isn't written to during an access

	
	//thread constructor, main process defines the motor and US sensor 
	public ScannerThread(RegulatedMotor head, RangeFinder sensor) {
		this.head = head;
		this.sensor = sensor;
	}
	
	//behavior of the scanner thread
	public void run() {
		//lejos.hardware.Sound.playSample(SumoRobotProgram.rocky, 100);
		head.resetTachoCount(); //reset the tachometer
		
		int motorAngle = 0;
		float dist = Float.POSITIVE_INFINITY; //temp storage for reported distance
		
		//main loop for scanner
		while(true) {
			
			head.forward(); //begin L-R sweep
			
			//Sweep only inside scan range and while we have no hits
			while(motorAngle < limit) {
				
				dist = getSample(); //retrieve sample from sensor
				if (dist < maxDist ) {
					head.stop(); //hold on found target
					Delay.msDelay(5); //ensure motor is stopped
					motorAngle = head.getTachoCount();
					watchTarget(dist, motorAngle);
					head.forward(); //continue L-R sweep once target lost

				}
				
				//check motor 100x as often as sensor
				for( int i = 0; i< 100; i++) {
					motorAngle = head.getTachoCount();
					if(motorAngle > limit) break;
					Delay.msDelay(1);
				}
				
			}
			
			//check if robot is turning off
			if(!SumoRobotProgram.robotRunning) {
				head.rotateTo(0); //reset head
				break;
			}
			
			head.backward(); //begin R-L sweep
			
			//Sweep only inside scan range and while we have no hits
			while(motorAngle > -limit) {
				
				dist = getSample(); //retrieve sample from sensor
				if (dist < maxDist ) {
					head.stop(); //hold on found target
					Delay.msDelay(20);
					motorAngle = head.getTachoCount();
					watchTarget(dist, motorAngle);
					head.backward(); //continue R-L sweep once target lost

				}
				
				//check motor 100x as often as sensor
				for( int i = 0; i< 100; i++) {
					motorAngle = head.getTachoCount();
					if(motorAngle < -limit) break;
					Delay.msDelay(1);
				}
				
			}
			
			//check if robot is turning off
			if(!SumoRobotProgram.robotRunning) {
				head.rotateTo(0); //reset head
				break;
			}
		}
	}
	
	private float getSample() {
		float sample = sensor.getRange();
		if(sample < maxDist) {
			timerLock.lock();
			hitTimer.reset();
			timerLock.unlock();
		}
		return sample;
	}
	
	private void watchTarget(float dist, int motorAngle) {
		float lastDist = Float.POSITIVE_INFINITY;
		int hitCount = 1;
		int movementCount = 0; // tracks how many times the enemy was spotted moving towards us
		
		//confirm 5 hits before declaring an enemy, also calculate if enemy is moving towards us
		while (hitCount < 5) {
			Delay.msDelay(100);
			lastDist = dist;
			dist = getSample();
			if( dist > maxDist) {
				break;
			}
			if(dist < lastDist) movementCount++;
			else movementCount = 0;
			hitCount++;
		}
		if(hitCount == 5) {
			if(SumoRobotProgram.testMode == 5) lejos.hardware.Sound.twoBeeps();
			//enemy confirmed, write enemy vars
			enemyVarLock.lock();
			enemyDist = dist;
			enemyAngle = motorAngle;
			if(movementCount == 4) enemyComing = true;
			else enemyComing = false;
			enemyVarLock.unlock();
		
			//now that enemy is confirmed, pause sensor until we lose visual twice in a row
			while(dist < maxDist && lastDist < maxDist) {
				Delay.msDelay(100);
				lastDist = dist;
				dist = getSample();
				enemyDist = dist;
			}
			if(SumoRobotProgram.testMode == 5) lejos.hardware.Sound.beep();
		}
		//reset enemy vars
		enemyVarLock.lock();
		enemyDist = Float.POSITIVE_INFINITY;
		enemyVarLock.unlock();
	}
	
	//methods for main program to query scanner
	public float getDistance() {
		return enemyDist;
	}
	public int getAngle() {
		return enemyAngle;
	}
	public boolean isEnemyComing() {
		return enemyComing;
	}
	public void resetTimeSinceLastHit( ) {
		timerLock.lock();
		hitTimer.reset();
		timerLock.unlock();
	}
	public int getTimeSinceLastHit() {
		timerLock.lock();
		int time = hitTimer.elapsed();
		timerLock.unlock();
		return time;
	}
	
	//method for scanner to share enemyVarLock
	public ReentrantLock getVarLock() {
		return enemyVarLock;
	}
}
