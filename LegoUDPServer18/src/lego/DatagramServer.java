package lego;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketTimeoutException;
import java.nio.ByteBuffer;
import java.util.LinkedList;
import java.util.ListIterator;
import java.util.NoSuchElementException;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

public class DatagramServer {

	public static RegulatedMotor mleft;
	public static RegulatedMotor mright;
	public static RegulatedMotor[] marray;
	public static Wheel wheel1;
	public static Wheel wheel2;
	public static Chassis chassis;
	public static MovePilot pilot;
	public static EV3IRSensor ir;
	public static EV3UltrasonicSensor usL;
	public static EV3UltrasonicSensor usR;
	public static SampleProvider irDistance;
	public static SampleProvider usLDistance;
	public static SampleProvider usRDistance;
	public static float[] irSample;
	public static float[] usLSample;
	public static float[] usRSample;
	public static boolean priorityModeFlag = false;
	public static int rotationTime;
	private static LinkedList<ByteBuffer> recallList = new LinkedList<>();
//	private final BlockingQueue<ByteBuffer> blockingQueue = new LinkedBlockingDeque<>();

	public static String s = null;
	public static float maxSpeedA = 0;
	public static float maxSpeedD = 0;
	
	private static boolean isInRecall = false;
	private static boolean automatedDodge = true;
	private static ListIterator<ByteBuffer> listIterator;
	private static boolean objectInFront = false;
	private static boolean objectInLeft = false;
	private static boolean objectInRight = false;

	public static void main(String[] args) throws IOException {
		mleft = new EV3LargeRegulatedMotor(MotorPort.A);
		mright = new EV3LargeRegulatedMotor(MotorPort.D);
		marray = new RegulatedMotor[1];
		marray[0] = mright;
		mleft.synchronizeWith(marray);
		mleft.setSpeed(10000);
		mright.setSpeed(10000);

		@SuppressWarnings("resource")
		DatagramSocket socket = new DatagramSocket(4711);
		socket.setSoTimeout(5000);
		socket.setReceiveBufferSize(1);

		ir = new EV3IRSensor(SensorPort.S1);
		usR = new EV3UltrasonicSensor(SensorPort.S4);
		usL = new EV3UltrasonicSensor(SensorPort.S3);
		irDistance = ir.getDistanceMode();
		usLDistance = usL.getDistanceMode();
		usRDistance = usR.getDistanceMode();
		irSample = new float[irDistance.sampleSize()];
		usLSample = new float[usLDistance.sampleSize()];
		usRSample = new float[usRDistance.sampleSize()];
		
		SensorThread checkSensors = new DatagramServer().new SensorThread();
		checkSensors.start();
		
	
		System.out.println("Server ready...");
		while (true) {
			// Auf Anfrage warten
			priorityModeFlag = false;
			DatagramPacket packet = new DatagramPacket(new byte[5], 5);
			try {
				socket.receive(packet);

				// Empfänger auslesen

				// InetAddress address = packet.getAddress();
				// int port = packet.getPort();
				// int len = packet.getLength();
				byte[] data = packet.getData();

				s = new String(data);
				LCD.drawString(s, 0, 1);
				// Delay.msDelay(100);

				checkForObstacle();
				
				if (!priorityModeFlag) {
					react(data);
				}
			} catch (SocketTimeoutException e) {
				mleft.startSynchronization();
				mleft.stop();
				mright.stop();
				mleft.endSynchronization();
				continue;
			}
		}
	}

	public static void checkForObstacle() {
		if (!isInRecall && automatedDodge) {
			if (objectInFront || objectInLeft || objectInRight) {
				ByteBuffer buffer = ByteBuffer.allocate(4);
				buffer.put((byte) 79);
			
				if (objectInFront == false && objectInLeft == true && objectInRight == false) {
					priorityModeFlag = true;
					Button.LEDPattern(1);
					turnRight(500, false);
					buffer.put((byte) 82);
					buffer.putShort((short) 500);
				} else if (objectInFront == false && objectInLeft == false && objectInRight == true) {
					priorityModeFlag = true;
					Button.LEDPattern(1);
					turnLeft(500, false);
					buffer.put((byte) 76);
					buffer.putShort((short) 500);
				} else if (objectInFront == true && objectInLeft == true && objectInRight == false) {
					priorityModeFlag = true;
					Button.LEDPattern(1);
					turnRight(1000, false);
					buffer.put((byte) 82);
					buffer.putShort((short) 1000);
				} else if (objectInFront == true && objectInLeft == false && objectInRight == true) {
					priorityModeFlag = true;
					Button.LEDPattern(1);
					turnLeft(1000, false);
					buffer.put((byte) 76);
					buffer.putShort((short) 1000);
				} else if (objectInFront == true && objectInLeft == true && objectInRight == true) {
					priorityModeFlag = true;
					startBackwardManeuver(1, false);
					buffer.put((byte) 66);
				} else if (objectInFront == true && objectInLeft == false && objectInRight == false) {
					priorityModeFlag = true;
					boolean left = avoidFrontObject();
					if (left) {
						buffer.put((byte) 76);
						buffer.putShort((short) 1000);
					} else {
						buffer.put((byte) 82);
						buffer.putShort((short) 1000);
					}
				}
				recallList.add(buffer);
			}
		}
	}
	
	public static boolean avoidFrontObject(){
		//check left side

			//decide for one side
		Button.LEDPattern(2);
		if (usRSample[0] <= usLSample[0]) {
			turnLeft(1000, false);
			return true;
		} else {
			turnRight(1000, false);
			return false;
		}
	}

	private static void turnLeft(int d, boolean reverse) {
		mleft.stop();
		mright.setSpeed((int) (mright.getMaxSpeed()/2));
		if (!reverse)
			mright.forward();
		else
			mright.backward();
		Delay.msDelay(d);
		Button.LEDPattern(0);
	}
	
	private static void turnRight(int d, boolean reverse) {
		mright.stop();
		mleft.setSpeed((int) (mleft.getMaxSpeed()/2));
		if (!reverse)
			mleft.forward();
		else
			mleft.backward();
		Delay.msDelay(d);
		Button.LEDPattern(0);
	}


	public static void startBackwardManeuver(int c, boolean reverse) {
		mleft.startSynchronization();
		mleft.stop();
		mright.stop();
		mleft.endSynchronization();
		if (c == 0){
		Sound.buzz();
		} else if (c ==1){
			Sound.twoBeeps();
		}
		int quarterSpeed = (int) (mleft.getMaxSpeed() / 4);
		mleft.setSpeed(quarterSpeed);
		mright.setSpeed(quarterSpeed);
		mleft.startSynchronization();
		if (!reverse) {
			mleft.backward();
			mright.backward();
		} else {
			mleft.forward();
			mright.forward();
		}
		mleft.endSynchronization();
		Delay.msDelay(1000);
		mleft.startSynchronization();
		mleft.stop();
		mright.stop();
		mleft.endSynchronization();
	}
	
	public static void forward (boolean reverse) {
		int speedRegulated = (int)(mleft.getMaxSpeed());
		mleft.setSpeed(speedRegulated);		
		mright.setSpeed(speedRegulated);
		mleft.startSynchronization();
		if (!reverse) {
			mleft.forward();
			mright.forward();
		} else {
			mleft.backward();
			mright.backward();
		}
		mleft.endSynchronization();
	}
	
	public static void left (boolean reverse) {
		int speedRegulated = (int) (mleft.getMaxSpeed() * 0.8);
		mright.setSpeed((int) (speedRegulated * 0.8));
		mleft.setSpeed((int) (speedRegulated * 0.8));
		if (!reverse) {
			mright.forward(); 
			mleft.backward();
		} else {
			mright.backward(); 
			mleft.forward();
		}
	}
	
	public static void right (boolean reverse) {
		int speedRegulated = (int)(mleft.getMaxSpeed() * 0.8);
		mleft.setSpeed((int) (speedRegulated * 0.8));
		mright.setSpeed((int) (speedRegulated * 0.8));
		if (!reverse) {
			mright.backward(); 
			mleft.forward();
		} else {
			mright.forward(); 
			mleft.backward();
		}
	}
	
	public static void backward (boolean reverse) {
		int speedRegulated = (int) Math.floor(mleft.getMaxSpeed());
		mleft.setSpeed(speedRegulated);		
		mright.setSpeed(speedRegulated);
		mleft.startSynchronization();
		if (!reverse) {
			mleft.backward();
			mright.backward();
		} else {
			mleft.forward();
			mright.forward();
		}
		mleft.endSynchronization();
	}
	
	public static void stop() {
		mleft.startSynchronization();
		mleft.stop();
		mright.stop();
		mleft.endSynchronization();
	}
	
	public static void driveGyroInput(int lOrR, int fOrB, int intensity, int direction, boolean reverse) {	
			intensity = (int) ((mleft.getMaxSpeed() * intensity) / 100);
			float f = intensity * (1 - ((float) direction / 100));
	
			if (direction != 0) {
				if (lOrR == 76) {
					mleft.setSpeed((int) (f));
					mright.setSpeed(intensity);
				} else {
					mright.setSpeed((int) (f));
					mleft.setSpeed(intensity);
				}
			}
			
			mleft.startSynchronization();
			if (fOrB == 70 && !reverse || fOrB == 66 && reverse) {
				mleft.forward();
				mright.forward();
			} else {
				mleft.backward();
				mright.backward();
			}
			mleft.endSynchronization();
	}
	
	public static void reactRecall() {
		if (!isInRecall) {
			listIterator = recallList.listIterator(recallList.size());
			isInRecall = true;
		}
		ByteBuffer buffer;
		try {
			buffer = listIterator.previous();
			listIterator.remove();
		} catch (NoSuchElementException e) {
			stop();
			return;
		}
		buffer.position(0);
		int move = buffer.get() & 0xff;
		
		switch (move) {
			case 70:
				forward(true);
				break;
			case 76:
				left(true);
				break;
			case 82:
				right(true);
				break;
			case 66:
				backward(true);
				break;
			case 71:
				int fOrB = buffer.get() & 0xff;
				int intensity = buffer.get() & 0xff;
				int lOrR = buffer.get() & 0xff;
				int direction = buffer.get() & 0xff;
				
				LCD.drawString(Character.toString((char) fOrB) + String.valueOf(intensity) + Character.toString((char) lOrR) + String.valueOf(direction), 0, 4);
				
				driveGyroInput(lOrR, fOrB, intensity, direction, true);
				break;
			case 79:
				int mode = buffer.get() & 0xff;
				short s = buffer.getShort();
				if (mode == 76)
					turnLeft(s, true);
				else if (mode == 82)
					turnRight(s, true);
				else if (mode == 66)
					startBackwardManeuver(1, true);
				break;
			default:
				return;
		}
	}

	public static void react(byte[] command) {
		
		ByteBuffer buffer = ByteBuffer.wrap(command);
		
		int c = buffer.get() & 0xff;
		
		switch (c) {
		case 70:
			isInRecall = false;
			recallList.add(buffer);
			forward(false);
			break;
		case 76:
			isInRecall = false;
			recallList.add(buffer);
			left(false);
			break;
		case 82:
			isInRecall = false;
			recallList.add(buffer);
			right(false);
			break;
		case 66:
			isInRecall = false;
			recallList.add(buffer);
			backward(false);
			break;
		case 83:
			isInRecall = false;
			stop();
			break;
		case 71:
			isInRecall = false;
			int fOrB = buffer.get() & 0xff;
			int intensity = buffer.get() & 0xff;
			int lOrR = buffer.get() & 0xff;
			int direction = buffer.get() & 0xff;

			recallList.add(buffer);

			driveGyroInput(lOrR, fOrB, intensity, direction, false);
			break;
		case 114:
			reactRecall();
			break;
		case 97:
			 automatedDodge = ((buffer.get() & 0xff) != 48);
		default:
			return;
		}
	}
	class SensorThread extends Thread {
		@Override
		public void run() {
			while (true) {
				irDistance.fetchSample(irSample, 0);
				usLDistance.fetchSample(usLSample, 0);
				usRDistance.fetchSample(usRSample, 0);
				LCD.drawString("L " + Float.toString(usLSample[0]), 0, 5);
				LCD.drawString("R " + Float.toString(usRSample[0]), 0, 6);
				LCD.drawString("IR " + Float.toString(irSample[0]), 0, 7);
			
				objectInFront = irSample[0] < 29;
				objectInLeft = usLSample[0] < 0.15;
				objectInRight = usRSample[0] < 0.15;
			}
		}
	}
}
