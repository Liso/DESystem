/* 
** Course and Semester : 18-649 Fall 2013
** Group No: 16
** Group Members : Jiangtian Nie(jnie) , Yue Chen(yuechen),
**                 Sally Stevenson(ststeven) , Sri Harsha Koppaka(skoppaka)
** Author : Jiangtian Nie
** AndrewID : jnie
*/

package simulator.elevatorcontrol;

import jSimPack.SimTime;

import java.util.HashMap;

import simulator.elevatormodules.*;
import simulator.framework.*;
import simulator.payloads.*;
import simulator.payloads.translators.IntegerCanPayloadTranslator;
import simulator.payloads.DrivePayload.WriteableDrivePayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;

public class DriveControl extends Controller {

	SimTime simtime = MessageDictionary.DRIVE_CONTROL_PERIOD;
	int currentFloor;
	WriteableDrivePayload localDrive;
	ReadableDriveSpeedPayload localDriveSpeed;
	DriveCommandCanPayloadTranslator mDriveCommand;
	DriveSpeedCanPayloadTranslator mDriveSpeed;
	DesiredFloorCanPayloadTranslator mDesiredFloor;
	CarLevelPositionCanPayloadTranslator mCarLevelPosition;
	HashMap<Integer, DoorClosedCanPayloadTranslator> mDoorClosedArray;
	HashMap<Integer, DoorMotorCommandCanPayloadTranslator> mDoorMotorArray;
	SafetySensorCanPayloadTranslator mSafety;
	HashMap<Integer, HoistwayLimitSensorCanPayloadTranslator> mHoistwayLimitArray;
	LevelingCanPayloadTranslator mLevelSensorArray[];
	CarWeightCanPayloadTranslator mCarWeight;
	Utility.AtFloorArray mAtFloorArray;
	
	private enum State {
		STATE_STOP,
		STATE_LEVEL_UP,
		STATE_LEVEL_DOWN,
		STATE_SLOW_UP,
		STATE_SLOW_DOWN,
		STATE_EMERGENCY;	
	}
	
	private State state = State.STATE_STOP;
	
	public DriveControl(boolean flag){
		super("DriveControl", flag);
		//currentFloor ;
		localDrive = DrivePayload.getWriteablePayload();
		physicalInterface.sendTimeTriggered(localDrive, simtime);
		localDriveSpeed = DriveSpeedPayload.getReadablePayload();
		physicalInterface.registerTimeTriggered(localDriveSpeed);
		simulator.payloads.CanMailbox.WriteableCanMailbox writeablecanmailbox = CanMailbox
				.getWriteableCanMailbox(MessageDictionary.DRIVE_COMMAND_CAN_ID);
		mDriveCommand = new DriveCommandCanPayloadTranslator(
				writeablecanmailbox);
		canInterface.sendTimeTriggered(writeablecanmailbox, simtime);
		writeablecanmailbox = CanMailbox.getWriteableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
		mDriveSpeed = new DriveSpeedCanPayloadTranslator(writeablecanmailbox);
		canInterface.sendTimeTriggered(writeablecanmailbox, simtime);
		simulator.payloads.CanMailbox.ReadableCanMailbox readablecanmailbox = CanMailbox
				.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		mDesiredFloor = new DesiredFloorCanPayloadTranslator(readablecanmailbox);
		canInterface.registerTimeTriggered(readablecanmailbox);
		readablecanmailbox = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
		mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(
				readablecanmailbox);
		canInterface.registerTimeTriggered(readablecanmailbox);
		readablecanmailbox = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
		mCarWeight = new CarWeightCanPayloadTranslator(readablecanmailbox);
		canInterface.registerTimeTriggered(readablecanmailbox);
		readablecanmailbox = CanMailbox.getReadableCanMailbox(MessageDictionary.EMERGENCY_BRAKE_CAN_ID);
		mSafety = new SafetySensorCanPayloadTranslator(readablecanmailbox);
		canInterface.registerTimeTriggered(readablecanmailbox);
		mDoorClosedArray = new HashMap<Integer, DoorClosedCanPayloadTranslator>();
		mDoorMotorArray = new HashMap<Integer, DoorMotorCommandCanPayloadTranslator>();
		
		Object aobj[] = Hallway.replicationValues;
		
		int i = aobj.length;
		
		for (int j = 0; j < i; j++) {
			Hallway hallway = (Hallway) aobj[j];
			Side aside[] = Side.values();
			int j1 = aside.length;
			for (int k1 = 0; k1 < j1; k1++) {
				Side side = aside[k1];
				int l1 = ReplicationComputer
						.computeReplicationId(hallway, side);
				simulator.payloads.CanMailbox.ReadableCanMailbox readablecanmailbox1 = CanMailbox
						.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + 
								ReplicationComputer.computeReplicationId(hallway, side));
				mDoorClosedArray.put(Integer.valueOf(l1),
						new DoorClosedCanPayloadTranslator(readablecanmailbox1,
								hallway, side));
				canInterface.registerTimeTriggered(readablecanmailbox1);
				readablecanmailbox1 = CanMailbox
						.getReadableCanMailbox(MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID+ ReplicationComputer
								.computeReplicationId(hallway, side));
				mDoorMotorArray.put(Integer.valueOf(l1),
						new DoorMotorCommandCanPayloadTranslator(readablecanmailbox1, hallway, side));
				canInterface.registerTimeTriggered(readablecanmailbox1);
			}

		}
		
		mHoistwayLimitArray = new HashMap<Integer, HoistwayLimitSensorCanPayloadTranslator>();
		aobj = Direction.replicationValues;
		i = aobj.length;
		for (int k = 0; k < i; k++) {
			Direction direction = (Direction) aobj[k];
			int i1 = ReplicationComputer.computeReplicationId(direction);
			simulator.payloads.CanMailbox.ReadableCanMailbox readablecanmailbox2 = CanMailbox
					.getReadableCanMailbox(MessageDictionary.HOISTWAY_LIMIT_BASE_CAN_ID + i1);
			mHoistwayLimitArray.put(Integer.valueOf(i1),
					new HoistwayLimitSensorCanPayloadTranslator(
							readablecanmailbox2, direction));
			canInterface.registerTimeTriggered(readablecanmailbox2);
		}
		
		
		
		mLevelSensorArray = new LevelingCanPayloadTranslator[2];
		aobj = Direction.replicationValues;
		i = aobj.length;
		for (int l = 0; l < i; l++) {
			Direction direction1 = (Direction) aobj[l];
			simulator.payloads.CanMailbox.ReadableCanMailbox readablecanmailbox3 = CanMailbox
					.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID + ReplicationComputer
							.computeReplicationId(direction1));
			mLevelSensorArray[ReplicationComputer
					.computeReplicationId(direction1)] = new LevelingCanPayloadTranslator(
					readablecanmailbox3, direction1);
			canInterface.registerTimeTriggered(readablecanmailbox3);
		}

		mAtFloorArray = new Utility.AtFloorArray(canInterface);
		state = State.STATE_STOP;
		timer.start(simtime);

	}
		
		
		private boolean isLevel() {
			boolean flag = mLevelSensorArray[ReplicationComputer
					.computeReplicationId(Direction.DOWN)].getValue()
					&& mLevelSensorArray[ReplicationComputer
							.computeReplicationId(Direction.UP)].getValue();
			return flag;
		}

		private boolean isAnyDoorOpen() {
			Hallway ahallway[] = Hallway.replicationValues;
			int i = ahallway.length;
			for (int j = 0; j < i; j++) {
				Hallway hallway = ahallway[j];
				Side aside[] = Side.values();
				int k = aside.length;
				for (int l = 0; l < k; l++) {
					Side side = aside[l];
					int i1 = ReplicationComputer
							.computeReplicationId(hallway, side);
					if (!(mDoorClosedArray.get(Integer.valueOf(i1))).getValue()
							|| (mDoorMotorArray
									.get(Integer.valueOf(i1))).getDoorCommand() == DoorCommand.OPEN)
						return true;
				}

			}

			return false;
		}

		private boolean isSafetyViolation() {
			if (mSafety.getValue())
				return true;
			Direction adirection[] = Direction.replicationValues;
			int i = adirection.length;
			for (int j = 0; j < i; j++) {
				Direction direction = adirection[j];
				if ((mHoistwayLimitArray
						.get(Integer.valueOf(ReplicationComputer
								.computeReplicationId(direction)))).getValue())
					return true;
			}

			return false;
		}

		private boolean isOverweight() {
			return mCarWeight.getWeight() >= Elevator.MaxCarCapacity;
		}
		
		public void timerExpired(Object callbackData){
			State newstate = state;
			currentFloor = mAtFloorArray.getCurrentFloor();
			 switch (state) {
			 	case STATE_STOP:
			 		localDrive.set(Speed.STOP, Direction.STOP);
			 		//#transition 'T6.5.1'
					if (isSafetyViolation()) {
						newstate = State.STATE_EMERGENCY;
						break;
					}
					
					//#transition 'T6.12'
					if (isOverweight()){
						newstate=State.STATE_STOP;
						break;
					}

					if (mDesiredFloor.getFloor() != mAtFloorArray.getCurrentFloor()
							&& !isAnyDoorOpen() && !isOverweight()) {
						//#transition 'T6.1'
						if (mDesiredFloor.getFloor() - mAtFloorArray.getCurrentFloor() > 0)
						{	
							newstate = State.STATE_SLOW_UP;}
						else
						//#transition 'T6.4'
							newstate = State.STATE_SLOW_DOWN;
						break;
					}
					
					if (isLevel() || !isAnyDoorOpen())
						break;
					if (mLevelSensorArray[ReplicationComputer
							.computeReplicationId(Direction.DOWN)].getValue())
						//#transition 'T6.8'
						newstate = State.STATE_LEVEL_UP;
					else if (mLevelSensorArray[ReplicationComputer.computeReplicationId(Direction.UP)].getValue())
							//#transition 'T6.9'
							newstate = State.STATE_LEVEL_DOWN;
			 		break;
			 	case STATE_LEVEL_UP:
			 		localDrive.set(Speed.LEVEL, Direction.UP);
					if (isSafetyViolation()) {
						newstate = State.STATE_EMERGENCY;
						break;
					}
					if (isLevel())
						//#transition 'T6.7'
						newstate = State.STATE_STOP;
			 		break;
			 	case STATE_LEVEL_DOWN:
			 		localDrive.set(Speed.LEVEL, Direction.DOWN);
					if (isSafetyViolation()) {
						newstate = State.STATE_EMERGENCY;
						break;
					}
					if (isLevel())
						//#transition 'T6.10'
						newstate = State.STATE_STOP;
			 		break;
			 	case STATE_SLOW_UP:
			 		localDrive.set(Speed.SLOW, Direction.UP);
					if (isSafetyViolation()) {
						//#transition 'T6.5.2'
						newstate = State.STATE_EMERGENCY;
						break;
					}
					if (!isSafetyViolation()
							&& mAtFloorArray.getCurrentFloor() == mDesiredFloor
									.getFloor()){
						if (isLevel()){
							//#transition 'T6.3'
							newstate = State.STATE_STOP;
				 			break;
				 			}
						//#transition 'T6.6'
						newstate = State.STATE_LEVEL_UP;
						break;
					}
                                        break;
			 	case STATE_SLOW_DOWN:
			 		localDrive.set(Speed.SLOW, Direction.DOWN);
					if (isSafetyViolation()) {
						//#transition 'T6.5.3'
						newstate = State.STATE_EMERGENCY;
						break;
					}
					if (!isSafetyViolation()
							&& mAtFloorArray.getCurrentFloor() == mDesiredFloor
									.getFloor()){
						if (isLevel()){
							//#transition 'T6.2'
							newstate = State.STATE_STOP;
				 			break;}
						//#transition 'T6.11'
						newstate = State.STATE_LEVEL_DOWN;
						break;
			 		}
                                        break;
			 	case STATE_EMERGENCY:
			 		localDrive.set(Speed.STOP, Direction.STOP);		 		
			 		newstate=State.STATE_EMERGENCY;
			 		break;
			 	default:
			 		throw new RuntimeException("State " + state + " was not recognized.");
			 }
			 
				mDriveSpeed.set(localDriveSpeed.speed(),localDriveSpeed.direction());
				mDriveCommand.setSpeed(localDrive.speed());
				mDriveCommand.setDirection(localDrive.direction());
				if (state != newstate)
					log(new Object[] { "Transition from ", state, " to ", newstate });
				setState("STATE", state.toString());
				state = newstate;
				timer.start(simtime);
		}
		
		
	}
	

