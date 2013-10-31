/* 
 * Course and Semester : 18-649 Fall 2013
 * Group No: 16
 * Group Members : Jiangtian Nie(jnie) , Yue Chen(yuechen),
 *                 Sally Stevenson(ststeven) , Sri Harsha Koppaka(skoppaka)
 * Author : Yue Chen
 * AndrewID : yuechen
 */

package simulator.elevatorcontrol;

import java.util.HashMap;

import jSimPack.SimTime;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.DriveSpeedPayload;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

public class Dispatcher extends Controller {

    // network interface
    // command door motor
    private WriteableCanMailbox networkDesiredFloor;
    // translator for the door motor command message --
    // this is a generic translator
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    
    private ReadableDriveSpeedPayload localDriveSpeed;
    
    private ReadableCanMailbox networkCarLevelPosition;
    private CarLevelPositionCanPayloadTranslator mCarLevelPosition;

    private WriteableCanMailbox networkDesiredDwellFront;
    // translator for the door motor command message --
    // this is a generic translator
    private DesiredDwellCanPayloadTranslator mDesiredDwellFront;

    private WriteableCanMailbox networkDesiredDwellBack;
    // translator for the door motor command message --
    // this is a generic translator
    private DesiredDwellCanPayloadTranslator mDesiredDwellBack;

    //received at floor message
    private HashMap<Integer, AtFloorCanPayloadTranslator> mAtFloor =
            new HashMap<Integer, AtFloorCanPayloadTranslator>();
    
    private HashMap<Integer, BooleanCanPayloadTranslator> mCarCall = 
    		new HashMap<Integer, BooleanCanPayloadTranslator>();
    
    private HashMap<Integer, BooleanCanPayloadTranslator> mHallCall = 
    		new HashMap<Integer, BooleanCanPayloadTranslator>();

    //received door closed message
    private ReadableCanMailbox networkDoorClosedFrontLeft;
    //translator for the doorClosed message -- this translator is specific
    private DoorClosedCanPayloadTranslator mDoorClosedFrontLeft;

    //received door closed message
    private ReadableCanMailbox networkDoorClosedFrontRight;
    //translator for the doorClosed message -- this translator is specific
    private DoorClosedCanPayloadTranslator mDoorClosedFrontRight;

    //received door closed message
    private ReadableCanMailbox networkDoorClosedBackLeft;
    //translator for the doorClosed message -- this translator is specific
    private DoorClosedCanPayloadTranslator mDoorClosedBackLeft;

    //received door closed message
    private ReadableCanMailbox networkDoorClosedBackRight;
    //translator for the doorClosed message -- this translator is specific
    private DoorClosedCanPayloadTranslator mDoorClosedBackRight;

    private static Hallway hallway = Hallway.NONE;
    private static int targetFloor = 1;
    private final static SimTime dwell = new SimTime(5,
            SimTime.SimTimeUnit.SECOND);
    private int currentAtFloor = 0;
    private int currentFloor = 0;
    private Direction desiredDirection = Direction.STOP;
    private Direction currentDirection = Direction.STOP;

    //store the period for the controller
    private SimTime period;

    //enumerate states
    private enum State {
        STATE_UP,
        STATE_DOWN,
        STATE_STOP,
        STATE_REACHED_FLOOR_UP,
        STATE_REACHED_FLOOR_DOWN,
        STATE_REACHED_FLOOR,
        STATE_RESET,
    }

    //state variable initialized to the initial state FLASH_OFF
    private State state = State.STATE_STOP;

    public Dispatcher(int MaxFloor, SimTime period, boolean verbose) {
        super("Dispatcher", verbose);

	 this.period = period;

        log("Created Dispatcher with period = ", period);

        networkDesiredFloor = CanMailbox.getWriteableCanMailbox(
                MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(
                networkDesiredFloor);
        canInterface.sendTimeTriggered(networkDesiredFloor, period);
        
        localDriveSpeed = DriveSpeedPayload.getReadablePayload();
        physicalInterface.registerTimeTriggered(localDriveSpeed);

        networkCarLevelPosition = CanMailbox.getReadableCanMailbox(
                MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
        mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(
                networkCarLevelPosition);
        canInterface.registerTimeTriggered(networkCarLevelPosition);
        
        networkDesiredDwellFront = CanMailbox.getWriteableCanMailbox(
                MessageDictionary.DESIRED_DWELL_BASE_CAN_ID + 
                ReplicationComputer.computeReplicationId(Hallway.FRONT));
        mDesiredDwellFront = new DesiredDwellCanPayloadTranslator(
                networkDesiredDwellFront, Hallway.FRONT);
        canInterface.sendTimeTriggered(networkDesiredDwellFront, period);

        networkDesiredDwellBack = CanMailbox.getWriteableCanMailbox(
                MessageDictionary.DESIRED_DWELL_BASE_CAN_ID + 
                ReplicationComputer.computeReplicationId(Hallway.BACK));
        mDesiredDwellBack = new DesiredDwellCanPayloadTranslator(
                networkDesiredDwellBack, Hallway.BACK);
        canInterface.sendTimeTriggered(networkDesiredDwellBack, period);

        for (int i = 0; i < Elevator.numFloors; i++) {
            int floor = i + 1;
            for (Hallway h : Hallway.replicationValues) {
                int index = ReplicationComputer.computeReplicationId(floor, h);
                ReadableCanMailbox m = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + index);
                AtFloorCanPayloadTranslator t = new AtFloorCanPayloadTranslator(m, floor, h);
                canInterface.registerTimeTriggered(m);
                mAtFloor.put(index, t);
                
                ReadableCanMailbox networkCarCall = CanMailbox.getReadableCanMailbox(
                		MessageDictionary.CAR_CALL_BASE_CAN_ID + 
                		ReplicationComputer.computeReplicationId(floor, h));
                BooleanCanPayloadTranslator nCarCall = new BooleanCanPayloadTranslator(networkCarCall);
                canInterface.registerTimeTriggered(networkCarCall);
                mCarCall.put(index, nCarCall);
                
                for (Direction d : Direction.replicationValues) {
                	int indexHallCall = ReplicationComputer.computeReplicationId(floor, h, d);
	                ReadableCanMailbox networkHallCall = CanMailbox.getReadableCanMailbox(
	                		MessageDictionary.HALL_CALL_BASE_CAN_ID + 
	                		ReplicationComputer.computeReplicationId(floor, h, d));
	                BooleanCanPayloadTranslator nHallCall = new BooleanCanPayloadTranslator(networkHallCall);
	                canInterface.registerTimeTriggered(networkHallCall);
	                mHallCall.put(indexHallCall, nHallCall);
                }
            }
        }
        

        networkDoorClosedFrontLeft = CanMailbox.getReadableCanMailbox(
                MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + 
                ReplicationComputer.computeReplicationId(Hallway.FRONT,
                                                         Side.LEFT));
        mDoorClosedFrontLeft =
                new DoorClosedCanPayloadTranslator(networkDoorClosedFrontLeft,
                                                   Hallway.FRONT, Side.LEFT);
        canInterface.registerTimeTriggered(networkDoorClosedFrontLeft);

        networkDoorClosedFrontRight = CanMailbox.getReadableCanMailbox(
                MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + 
                ReplicationComputer.computeReplicationId(Hallway.FRONT,
                                                         Side.RIGHT));
        mDoorClosedFrontRight =
                new DoorClosedCanPayloadTranslator(networkDoorClosedFrontRight,
                                                   Hallway.FRONT, Side.RIGHT);
        canInterface.registerTimeTriggered(networkDoorClosedFrontRight);

        networkDoorClosedBackLeft = CanMailbox.getReadableCanMailbox(
                MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + 
                ReplicationComputer.computeReplicationId(Hallway.BACK,
                                                         Side.LEFT));
        mDoorClosedBackLeft =
                new DoorClosedCanPayloadTranslator(networkDoorClosedBackLeft,
                                                   Hallway.BACK, Side.LEFT);
        canInterface.registerTimeTriggered(networkDoorClosedBackLeft);

        networkDoorClosedBackRight = CanMailbox.getReadableCanMailbox(
                MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + 
                ReplicationComputer.computeReplicationId(Hallway.BACK,
                                                         Side.RIGHT));
        mDoorClosedBackRight =
                new DoorClosedCanPayloadTranslator(networkDoorClosedBackRight,
                                                   Hallway.BACK, Side.RIGHT);
        canInterface.registerTimeTriggered(networkDoorClosedBackRight);

        mDesiredFloor.set(targetFloor, hallway, Direction.STOP);
        mDesiredDwellFront.setDwell(dwell);
        mDesiredDwellBack.setDwell(dwell);
        timer.start(period);
    }

    /*
     * The timer callback is where the main controller code is executed.  For
     * time triggered design, this consists mainly of a switch block with a
     * case blcok for each state.  Each case block executes actions for that
     * state, then executes a transition to the next state if the transition
     * conditions are met.
     */
    public void timerExpired(Object callbackData) {
        State newState = state;

        boolean isAtFloor = false;
        int nHallway = 0;
        int index = 0;
        int indexHallCall;
        int indexCarCall;
        int commitPointUp;
        int commitPointDown;
        boolean desiredHallwayOpen = false;
        int acc = 1;
        int position = mCarLevelPosition.getValue();
        double currentSpeed = localDriveSpeed.speed();
        for (int i = 0; i < Elevator.numFloors; i++) {
            int floor = i + 1;
            for (Hallway h : Hallway.replicationValues) {
            	
                index = ReplicationComputer.computeReplicationId(floor, h);
                if (mAtFloor.get(index).getValue()) {
                    isAtFloor = true;
                    currentAtFloor = floor;
                }
            }
        }
        if(isAtFloor == true){
        	currentFloor  =currentAtFloor;
        }
        else{
        	for(int i = 0; i < Elevator.numFloors; i++){
        		int floor = i + 1;
        		if((position >= ((5000*(floor - 1)) - 50)) && (position < ((5000*floor) -50))){
        			currentFloor = floor;
        		}
        	}
        }
    

        switch (state) {
        case STATE_STOP:
            // state actions for state S11.1 'SET TARGET'
            mDesiredFloor.set(targetFloor, Hallway.NONE, Direction.STOP);

            // #transition 'T11.1'
            // all doors are closed
            if (!(mDoorClosedFrontLeft.getValue() && 
                    mDoorClosedFrontRight.getValue() && 
                    mDoorClosedBackLeft.getValue() && 
                    mDoorClosedBackRight.getValue())) {
                if (!isAtFloor){
                    newState = State.STATE_RESET;
                    break;
                }
            }
            int flag = 0;
            for(int i = 0; i < Elevator.numFloors; i++){
            	int floor = i + 1;
            	
            	for(Hallway h : Hallway.replicationValues){
            		for(Direction d : Direction.replicationValues){
            			indexHallCall = ReplicationComputer.computeReplicationId(floor, h, d);
            			indexCarCall = ReplicationComputer.computeReplicationId(floor, h);
            				
            			if(flag == 0){
            				if(mHallCall.get(indexHallCall).getValue() || mCarCall.get(indexCarCall).getValue()){
            			
            					flag = 1;
            					targetFloor = floor;
            					if(mHallCall.get(indexHallCall).getValue()){
            						if(targetFloor > currentAtFloor){
            							newState = State.STATE_UP;
            							currentDirection = d;
            							desiredDirection = Direction.UP;
            						}
            						else if(targetFloor < currentAtFloor) {
            							newState = State.STATE_DOWN;
            							currentDirection = d;
            							desiredDirection = Direction.DOWN;
            						}    
            					
            						else if(targetFloor == currentAtFloor){
            							newState = State.STATE_REACHED_FLOOR;
            							currentDirection = d;
            							desiredDirection = d;
            						}
            					}
            					
            					else{
            						if(targetFloor > currentAtFloor){
            							newState = State.STATE_UP;
            							currentDirection = Direction.UP;
            							desiredDirection = Direction.STOP;
            						}
            						else if(targetFloor < currentAtFloor) {
            							newState = State.STATE_DOWN;
            							currentDirection = Direction.DOWN;
            							desiredDirection = Direction.STOP;
            						}    
            					
            						else if(targetFloor == currentAtFloor){
            							newState = State.STATE_REACHED_FLOOR;
            							desiredDirection = Direction.STOP;
            						}	
            					}
            						
            				}       				
            			}
            		}
           		}
           	}
            
            nHallway = 0;
            for (Hallway h : Hallway.replicationValues) {
            	indexCarCall = ReplicationComputer.computeReplicationId(targetFloor, h);
            	if (mCarCall.get(indexCarCall).getValue()) {
                    nHallway++;
                    if (nHallway >= 2)
                        hallway = Hallway.BOTH;
                    else
                        hallway = h;
        		}
            	
        		indexHallCall = ReplicationComputer.computeReplicationId(targetFloor, h, currentDirection);
        		if (mHallCall.get(indexHallCall).getValue() && !mCarCall.get(indexCarCall).getValue()) {
                    nHallway++;
                    if (nHallway >= 2)
                        hallway = Hallway.BOTH;
                    else
                        hallway = h;
        		}
            }
            
            break;
        case STATE_UP:
            // state actions for state S11.1 'SET TARGET'
        	
            mDesiredFloor.set(targetFloor, hallway, desiredDirection);
            if (hallway == Hallway.FRONT)
                mDesiredDwellFront.setDwell(dwell);
            else if (hallway == Hallway.BACK)
                mDesiredDwellBack.setDwell(dwell);
            else if (hallway == Hallway.BOTH) {
                mDesiredDwellFront.setDwell(dwell);
                mDesiredDwellBack.setDwell(dwell);
            }

            // #transition 'T11.1'
            // all doors are closed
            if (!(mDoorClosedFrontLeft.getValue() && 
                    mDoorClosedFrontRight.getValue() && 
                    mDoorClosedBackLeft.getValue() && 
                    mDoorClosedBackRight.getValue())) {
                if (!isAtFloor)
                    newState = State.STATE_RESET;
            }
            boolean Calls = false;
            for(Hallway h : Hallway.replicationValues){
            	int indextargetHallCall = ReplicationComputer.computeReplicationId(targetFloor,h,Direction.UP);
                int indextargetCarCall = ReplicationComputer.computeReplicationId(targetFloor,h);
                
                if(mHallCall.get(indextargetHallCall).getValue() || mCarCall.get(indextargetCarCall).getValue()){
                	Calls = true;
                }
            }
            
            int targetFlag = 0;

        		  if(Calls == false){
        	   for(int i = targetFloor; i < Elevator.numFloors; i++){
        		   int floor = i;
        		   if(currentSpeed == 0){
                   	commitPointUp = (5000*(floor -1)) + 100;
                   }
                   else
        		   commitPointUp = (int)(((5*(floor - 1)) - ((currentSpeed * currentSpeed)/(2*acc))) * 1000) - 200;
        		   for(Hallway hall : Hallway.replicationValues){
        			   for(Direction d : Direction.replicationValues){
        			   indexHallCall = ReplicationComputer.computeReplicationId(floor, hall, d);
        			   		if(mHallCall.get(indexHallCall).getValue()){
        			   			if((targetFlag == 0) && (position < commitPointUp)){
        			   				targetFloor = floor;
        			   				currentDirection = d;
        			   				targetFlag = 1;
        			   			}
        			   		}
        			   		indexCarCall = ReplicationComputer.computeReplicationId(floor, hall);
        			   		if(mCarCall.get(indexCarCall).getValue()){
        			   			if((targetFlag == 0) && (position < commitPointUp)){
        			   				targetFloor = floor;
        			   				currentDirection = Direction.UP;
        			   				targetFlag = 1;
        			   			}
        			   		}
        			   }
        		   }
        	   }
           }
                for (int i = 0; i < Elevator.numFloors; i++) {
                    int floor = i + 1;
                    if(currentSpeed == 0.0){
                    	commitPointUp = (5000*(floor -1)) + 100;
                    }
                    else
                    	commitPointUp = (int)(((5*(floor - 1)) - ((currentSpeed * currentSpeed)/(2*acc))) * 1000) - 200;
                    if (Math.abs(floor - currentFloor) <= Math.abs(targetFloor - currentFloor)) {
	                    for (Hallway h : Hallway.replicationValues) {                    	
	                    		indexHallCall = ReplicationComputer.computeReplicationId(floor, h, Direction.UP);
		                    	indexCarCall = ReplicationComputer.computeReplicationId(floor, h);
		                    	
	                    		if (mHallCall.get(indexHallCall).getValue() || mCarCall.get(indexCarCall).getValue()) { 
	                    			if(position <= commitPointUp){
	                    				currentDirection = Direction.UP;
	                    				targetFloor = floor;
	                    			}
	                    		}
	                    }
                    }
                    
                }
                if (!(mDoorClosedFrontLeft.getValue() && 
                        mDoorClosedFrontRight.getValue() && 
                        mDoorClosedBackLeft.getValue() && 
                        mDoorClosedBackRight.getValue())) {
                	desiredDirection = Direction.UP;
                }
                else{
                int dirflag = 0;
                for(int i = targetFloor ; i < Elevator.numFloors; i++){
            		int floor = i +1 ;
                    for (Hallway h : Hallway.replicationValues) {
                    	for (Direction d : Direction.replicationValues) {
                    		indexHallCall = ReplicationComputer.computeReplicationId(floor, h, d);
                    		indexCarCall = ReplicationComputer.computeReplicationId(floor, h);
                    		
                    		if(mHallCall.get(indexHallCall).getValue() || mCarCall.get(indexCarCall).getValue()){
                    			desiredDirection = Direction.UP;
                    			dirflag = 1;
                    		}
                    	}
                    }
                }
                int dirflag1 = 0;
                if(dirflag == 0){
                	  for(int i = targetFloor-1 ; i >0; i--){
                  		int floor = i ;
                          for (Hallway h : Hallway.replicationValues) {
                          	for (Direction d : Direction.replicationValues) {
                          		indexHallCall = ReplicationComputer.computeReplicationId(floor, h, d);
                          		indexCarCall = ReplicationComputer.computeReplicationId(floor, h);
                          		
                          		if(mHallCall.get(indexHallCall).getValue() || mCarCall.get(indexCarCall).getValue()){
                          			desiredDirection = Direction.DOWN;
                          			dirflag1 = 1;
                          		}
                          	}
                          }
                      }
                }
                if(dirflag ==0 && dirflag1 == 0){
                	desiredDirection = Direction.STOP;
                }
                }


            int flag2 = 0;
            if(targetFloor == currentAtFloor){
            	flag2 =2;
            	for(int i = currentAtFloor ; i < Elevator.numFloors; i++){
            		int floor = i +1 ;
                    for (Hallway h : Hallway.replicationValues) {
                    	for (Direction d : Direction.replicationValues) {
                    		indexHallCall = ReplicationComputer.computeReplicationId(floor, h, d);
                    		indexCarCall = ReplicationComputer.computeReplicationId(floor, h);
                    		
                    		if(mHallCall.get(indexHallCall).getValue() || mCarCall.get(indexCarCall).getValue()){
                    			flag2 =1;
                    			newState = State.STATE_REACHED_FLOOR_UP;
                    			desiredDirection = Direction.UP;
                    		}
                    	}
                    	
                    }
            		
            	}
            }
        	if(flag2 == 2){
            for(Hallway h : Hallway.replicationValues){
            	int indexCurrentCall = ReplicationComputer.computeReplicationId(currentAtFloor, h, Direction.DOWN);
            
            		if(mHallCall.get(indexCurrentCall).getValue()){
            			targetFloor = currentAtFloor;
                		currentDirection = Direction.DOWN;
                		desiredDirection = Direction.DOWN;
                		newState = State.STATE_DOWN;
            		}
            		else{
            			newState = State.STATE_REACHED_FLOOR;
            		}
            	}
            }
            

            nHallway = 0;
            for (Hallway h : Hallway.replicationValues) {
            	indexCarCall = ReplicationComputer.computeReplicationId(targetFloor, h);
            	if (mCarCall.get(indexCarCall).getValue()) {
                    nHallway++;
                    if (nHallway >= 2)
                        hallway = Hallway.BOTH;
                    else
                        hallway = h;
        		}
            	if(targetFloor == 8){
            		currentDirection = Direction.DOWN;
            	}
        		indexHallCall = ReplicationComputer.computeReplicationId(targetFloor, h, currentDirection);
        		if (mHallCall.get(indexHallCall).getValue() && !mCarCall.get(indexCarCall).getValue()) {
                    nHallway++;
                    if (nHallway >= 2)
                        hallway = Hallway.BOTH;
                    else
                        hallway = h;
        		}
            }
            
            
                	

                // make sure that the last is false as well
                // #transition 'T11.9'

            break;      
        case STATE_DOWN:
            // state actions for state S11.1 'SET TARGET'
            mDesiredFloor.set(targetFloor, hallway, desiredDirection);
            if (hallway == Hallway.FRONT)
                mDesiredDwellFront.setDwell(dwell);
            else if (hallway == Hallway.BACK)
                mDesiredDwellBack.setDwell(dwell);
            else if (hallway == Hallway.BOTH) {
                mDesiredDwellFront.setDwell(dwell);
                mDesiredDwellBack.setDwell(dwell);
            }

            // #transition 'T11.1'
            // all doors are closed
            if (!(mDoorClosedFrontLeft.getValue() && 
                    mDoorClosedFrontRight.getValue() && 
                    mDoorClosedBackLeft.getValue() && 
                    mDoorClosedBackRight.getValue())) {            	
                if (!isAtFloor)
                    newState = State.STATE_RESET;
            }

            Calls = false;
            for(Hallway h : Hallway.replicationValues){
            	int indextargetHallCall = ReplicationComputer.computeReplicationId(targetFloor,h,Direction.DOWN);
                int indextargetCarCall = ReplicationComputer.computeReplicationId(targetFloor,h);
                
                if(mHallCall.get(indextargetHallCall).getValue() || mCarCall.get(indextargetCarCall).getValue()){
                	Calls = true;
                }
            }

            int targetflag1 = 0;
          
        		   if(Calls == false){
        	   for(int i = targetFloor; i > 0; i--){
        		   int floor = i;
        		   if(currentSpeed == 0){
                   	commitPointDown = (5000*(floor -1)) - 100;
                   }
                   else
                	   commitPointDown = (int)(((5*(floor - 1)) + ((currentSpeed * currentSpeed)/(2*acc))) * 1000) + 200;
        		   for(Hallway hall : Hallway.replicationValues){
        			   for(Direction d : Direction.replicationValues){
        			   indexHallCall = ReplicationComputer.computeReplicationId(floor, hall, d);
        			   		if(mHallCall.get(indexHallCall).getValue()){
        			   			if((targetflag1 == 0) && (position > commitPointDown)){
        			   				targetFloor = floor;
        			   				currentDirection = d;
        			   				targetflag1 = 1;
        			   			}
        			   		}
        			   		
        			   		indexCarCall = ReplicationComputer.computeReplicationId(floor, hall);
        			   		if(mCarCall.get(indexCarCall).getValue()){
        			   			if((targetflag1 == 0) && (position > commitPointDown)){
        			   				targetFloor = floor;
        			   				currentDirection = Direction.DOWN;
        			   				targetFlag = 1;
        			   			}
        			   		}
        			   }
        			   
        		   }
        	   }
           }
                for (int i = 0; i < currentFloor ; i++) {
                    int floor = i + 1;
                    if(currentSpeed == 0){
                       	commitPointDown = (5000*(floor -1)) - 100;
                       }
                       else
                    	   commitPointDown = (int)(((5*(floor - 1)) + ((currentSpeed * currentSpeed)/(2*acc))) * 1000) + 200;
                    if (Math.abs(floor - currentFloor) <= Math.abs(targetFloor - currentFloor)) {
	                    for (Hallway h : Hallway.replicationValues) {
                    	

	                    		indexHallCall = ReplicationComputer.computeReplicationId(floor, h, Direction.DOWN);
	                    		indexCarCall = ReplicationComputer.computeReplicationId(floor, h);
	                    		if (mHallCall.get(indexHallCall).getValue() || mCarCall.get(indexCarCall).getValue()) {
	                    			if(position >= commitPointDown){
	                    				targetFloor = floor;
	                    				currentDirection = Direction.DOWN;
	                    			}
	                    		}	                    	             	
	                    }
                    }
                    
                }
            
                if (!(mDoorClosedFrontLeft.getValue() && 
                        mDoorClosedFrontRight.getValue() && 
                        mDoorClosedBackLeft.getValue() && 
                        mDoorClosedBackRight.getValue())) {
                	desiredDirection = Direction.DOWN;
                }
                else{
                int dirflag2 = 0;
                for(int i = targetFloor -1 ; i >0; i--){
            		int floor = i  ;
                    for (Hallway h : Hallway.replicationValues) {
                    	for (Direction d : Direction.replicationValues) {
                    		indexHallCall = ReplicationComputer.computeReplicationId(floor, h, d);
                    		indexCarCall = ReplicationComputer.computeReplicationId(floor, h);
                    		
                    		if(mHallCall.get(indexHallCall).getValue() || mCarCall.get(indexCarCall).getValue()){
                    			desiredDirection = Direction.DOWN;
                    			dirflag2 = 1;
                    		}
                    	}
                    }
                }
                int dirflag3 = 0;
                if(dirflag2 == 0){
                	  for(int i = targetFloor ; i < Elevator.numFloors; i++){
                  		int floor = i+1 ;
                          for (Hallway h : Hallway.replicationValues) {
                          	for (Direction d : Direction.replicationValues) {
                          		indexHallCall = ReplicationComputer.computeReplicationId(floor, h, d);
                          		indexCarCall = ReplicationComputer.computeReplicationId(floor, h);
                          		
                          		if(mHallCall.get(indexHallCall).getValue() || mCarCall.get(indexCarCall).getValue()){
                          			desiredDirection = Direction.UP;
                          			dirflag3 = 1;
                          		}
                          	}
                          }
                      }
                }
                if(dirflag2 ==0 && dirflag3 == 0){
                	desiredDirection = Direction.STOP;
                }
                }
                int flag3 = 0;
                if(targetFloor == currentAtFloor){
                	flag3 = 2;
                	for(int i = currentAtFloor -1 ; i > 0; i--){
                		int floor = i ;
                        for (Hallway h : Hallway.replicationValues) {
                        	for (Direction d : Direction.replicationValues) {
                        		indexHallCall = ReplicationComputer.computeReplicationId(floor, h, d);
                        		indexCarCall = ReplicationComputer.computeReplicationId(floor, h);
                        		
                        		if(mHallCall.get(indexHallCall).getValue() || mCarCall.get(indexCarCall).getValue()){
                        			flag3 =1;
                        			newState = State.STATE_REACHED_FLOOR_DOWN;
                        			desiredDirection = Direction.DOWN;
                        		}
                        	}
                        	
                        }
                		
                	}
                }
                
                for(Hallway h : Hallway.replicationValues){
                	int indexCurrentCall = ReplicationComputer.computeReplicationId(currentAtFloor, h, Direction.UP);
                	if(flag3 == 2){
                		if(mHallCall.get(indexCurrentCall).getValue()){
                			targetFloor = currentAtFloor;
                			currentDirection = Direction.UP;
                			desiredDirection = Direction.UP;
                			newState = State.STATE_UP;
                		}
                		else{
                			newState = State.STATE_REACHED_FLOOR;
                		}
                	}	
                }
             
                
                nHallway = 0;
                for (Hallway h : Hallway.replicationValues) {
                	indexCarCall = ReplicationComputer.computeReplicationId(targetFloor, h);
                	if (mCarCall.get(indexCarCall).getValue()) {
                        nHallway++;
                        if (nHallway >= 2)
                            hallway = Hallway.BOTH;
                        else
                            hallway = h;
            		}
                	if(targetFloor == 1){
                		currentDirection = Direction.UP;
                	}
            		indexHallCall = ReplicationComputer.computeReplicationId(targetFloor, h, currentDirection);
            		if (mHallCall.get(indexHallCall).getValue() && !mCarCall.get(indexCarCall).getValue()) {
                        nHallway++;
                        if (nHallway >= 2)
                            hallway = Hallway.BOTH;
                        else
                            hallway = h;
            		}
                }
                // make sure that the last is false as well
                // #transition 'T11.9'

            break;       
            
        case STATE_REACHED_FLOOR_UP:
        	
            mDesiredFloor.set(targetFloor, hallway, Direction.UP);
            desiredHallwayOpen = false;
        	if(hallway == Hallway.BOTH){
        		if(!mDoorClosedFrontLeft.getValue() && 
                        !mDoorClosedFrontRight.getValue() && 
                        !mDoorClosedBackLeft.getValue() && 
                        !mDoorClosedBackRight.getValue()){
        			desiredHallwayOpen = true;     			
        		}
        	}
        		
        	else if(hallway == Hallway.FRONT){
        		if(!mDoorClosedFrontLeft.getValue() && 
                        !mDoorClosedFrontRight.getValue()){
        			desiredHallwayOpen = true;    
        		}
        	}

        	else if(hallway == Hallway.BACK){
        		if(!mDoorClosedBackLeft.getValue() && 
                    !mDoorClosedBackRight.getValue()){
        			desiredHallwayOpen = true;     			
        		}
        	}
            if (desiredHallwayOpen){
                int flag1 = 0;
                for(int i = currentAtFloor; i < Elevator.numFloors; i++){
                	int floor = i + 1;
                	
                	for(Hallway h : Hallway.replicationValues){
                		for(Direction d : Direction.replicationValues){
                			indexHallCall = ReplicationComputer.computeReplicationId(floor, h, Direction.UP);
                			indexCarCall = ReplicationComputer.computeReplicationId(floor, h);
                				
                			if(flag1 == 0){
                				if(mHallCall.get(indexHallCall).getValue() || mCarCall.get(indexCarCall).getValue()){
                			
                					flag1 = 1;
                					targetFloor = floor;
                					currentDirection = Direction.UP;
                					desiredDirection = Direction.UP;
                					newState = State.STATE_UP;
                				}       				
                			}
                		}
               		}
               	}
                if(flag1 == 0){
                	for(int i = currentAtFloor; i < Elevator.numFloors; i++){
                    	int floor = i + 1;
                    	
                    	for(Hallway h : Hallway.replicationValues){
                    		
                    		indexHallCall = ReplicationComputer.computeReplicationId(floor, h, Direction.DOWN);
                    		
                    		if(mHallCall.get(indexHallCall).getValue()){
                    			targetFloor = floor;
                    			currentDirection = Direction.DOWN;
                    			desiredDirection = Direction.UP;
                    			newState = State.STATE_UP;
                  //  			flag7 = 1;
                    		}
                    	}
                	}
                }                 
            }

            nHallway = 0;
            for (Hallway h : Hallway.replicationValues) {
            	indexCarCall = ReplicationComputer.computeReplicationId(targetFloor, h);
            	if (mCarCall.get(indexCarCall).getValue()) {
                    nHallway++;
                    if (nHallway >= 2)
                        hallway = Hallway.BOTH;
                    else
                        hallway = h;
        		}
            	
        		indexHallCall = ReplicationComputer.computeReplicationId(targetFloor, h, currentDirection);
        		if (mHallCall.get(indexHallCall).getValue() && !mCarCall.get(indexCarCall).getValue()) {
                    nHallway++;
                    if (nHallway >= 2)
                        hallway = Hallway.BOTH;
                    else
                        hallway = h;
        		}
            }
            break;
            
        case STATE_REACHED_FLOOR_DOWN:
        	
            mDesiredFloor.set(targetFloor, hallway, Direction.DOWN);
            desiredHallwayOpen = false;
        	if(hallway == Hallway.BOTH){
        		if(!mDoorClosedFrontLeft.getValue() && 
                        !mDoorClosedFrontRight.getValue() && 
                        !mDoorClosedBackLeft.getValue() && 
                        !mDoorClosedBackRight.getValue()){
        			desiredHallwayOpen = true;     			
        		}
        	}
        		
        	else if(hallway == Hallway.FRONT){
        		if(!mDoorClosedFrontLeft.getValue() && 
                        !mDoorClosedFrontRight.getValue()){
        			desiredHallwayOpen = true;    
        		}
        	}

        	else if(hallway == Hallway.BACK){
        		if(!mDoorClosedBackLeft.getValue() && 
                    !mDoorClosedBackRight.getValue()){
        			desiredHallwayOpen = true;     			
        		}
        	}
            if (desiredHallwayOpen){
            	 int flag1 = 0;
                 for(int i = currentFloor -1; i > 0; i--){
                 	int floor = i ;
                 	
                 	for(Hallway h : Hallway.replicationValues){
                 		for(Direction d : Direction.replicationValues){
                 			indexHallCall = ReplicationComputer.computeReplicationId(floor, h, Direction.DOWN);
                 			indexCarCall = ReplicationComputer.computeReplicationId(floor, h);
                 				
                 			if(flag1 != 1){
                 				if(mHallCall.get(indexHallCall).getValue() || mCarCall.get(indexCarCall).getValue()){
                 			
                 					flag1 = 1;
                 					targetFloor = floor;
                 					currentDirection = Direction.DOWN;
                 					newState = State.STATE_DOWN;
                 				}       				
                 			}
                 		}
                	}
                }
                 if(flag1 == 0){
                 	for(int i = currentFloor; i > 1; i--){
                     	int floor = i - 1;
                     	
                     	for(Hallway h : Hallway.replicationValues){
                     		
                     		indexHallCall = ReplicationComputer.computeReplicationId(floor, h, Direction.UP);
                     		
                     		if(mHallCall.get(indexHallCall).getValue()){
                     			targetFloor = floor;
                     			currentDirection = Direction.UP;
                     			desiredDirection = Direction.DOWN;
                     			newState = State.STATE_DOWN;
                     		}
                     	}
                 	}
                 }
            }   		
    		
            nHallway = 0;
            for (Hallway h : Hallway.replicationValues) {
            	indexCarCall = ReplicationComputer.computeReplicationId(targetFloor, h);
            	if (mCarCall.get(indexCarCall).getValue()) {
                    nHallway++;
                    if (nHallway >= 2)
                        hallway = Hallway.BOTH;
                    else
                        hallway = h;
        		}
            	
        		indexHallCall = ReplicationComputer.computeReplicationId(targetFloor, h, currentDirection);
        		if (mHallCall.get(indexHallCall).getValue() && !mCarCall.get(indexCarCall).getValue()) {
                    nHallway++;
                    if (nHallway >= 2)
                        hallway = Hallway.BOTH;
                    else
                        hallway = h;
        		}
            }
            break;
            
        case STATE_REACHED_FLOOR:
        	
            mDesiredFloor.set(targetFloor, hallway, Direction.STOP);
            desiredHallwayOpen = false;
        	if(hallway == Hallway.BOTH){
        		if(!mDoorClosedFrontLeft.getValue() && 
                        !mDoorClosedFrontRight.getValue() && 
                        !mDoorClosedBackLeft.getValue() && 
                        !mDoorClosedBackRight.getValue()){
        			desiredHallwayOpen = true;     			
        		}
        	}
        		
        	else if(hallway == Hallway.FRONT){
        		if(!mDoorClosedFrontLeft.getValue() && 
                        !mDoorClosedFrontRight.getValue()){
        			desiredHallwayOpen = true;    
        		}
        	}

        	else if(hallway == Hallway.BACK){
        		if(!mDoorClosedBackLeft.getValue() && 
                    !mDoorClosedBackRight.getValue()){
        			desiredHallwayOpen = true;     			
        		}
        	}
        	
            if (desiredHallwayOpen){
            	newState = State.STATE_STOP;
            }
        	
        	break;
        case STATE_RESET:
            // state actions for state S11.2 'RESET'
            mDesiredFloor.set(1, Hallway.NONE, Direction.STOP);
            break;
        default:
            throw new RuntimeException("State " + state +
                                       " was not recognized.");
        }

        if (state == newState) {
            log("remains in state: ",state);
        }
        else {
            log("Transition:",state,"->",newState);
        }

        state = newState;
        setState(STATE_KEY,newState.toString());

        // schedule the next iteration of the controller
        // you must do this at the end of the timer callback in order to
        // restart the timer
        timer.start(period);
    }

}
