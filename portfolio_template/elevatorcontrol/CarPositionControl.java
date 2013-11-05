/* 
 * Course and Semester : 18-649 Fall 2013
 * Group No: 16
 * Group Members : Jiangtian Nie(jnie) , Yue Chen(yuechen),
 *                 Sally Stevenson(ststeven) , Sri Harsha Koppaka(skoppaka)
 * Author : Jiangtian Nie
 * AndrewID : jnie
 */

package simulator.elevatorcontrol;

import java.util.HashMap;

import jSimPack.SimTime;
import simulator.elevatormodules.*;
import simulator.framework.*;
import simulator.payloads.*;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CarPositionIndicatorPayload.*;

public class CarPositionControl extends Controller {

	private SimTime period ;
	// Translator for AtFloor message -- Specific to Message
	
    private int currentAtFloor = 0;
    private int currentFloor = 0;

	private enum State {
		STATE_DISPLAY;
	}
	
	//physical CarPosition indicator;
	private WriteableCarPositionIndicatorPayload localCPI;
	
    private ReadableCanMailbox networkCarLevelPosition;
    private IntCanPayloadTranslator mCarLevelPosition;

	//define a HashMap to represent 4 status of hallway. 
    //received at floor message
    private HashMap<Integer, BitCanPayloadTranslator> mAtFloor =
            new HashMap<Integer, BitCanPayloadTranslator>();
	
	//current state
	State state = State.STATE_DISPLAY;

	public CarPositionControl(SimTime period, boolean verbose) {
		super("CarPositionControl", verbose);
		
		this.period = period;
		
		//Initializing the Hallway values
		
		//Send Physical message to CarPositionIndicator
		localCPI = CarPositionIndicatorPayload.getWriteablePayload();
		//Register PayLoad to be sent periodically
		physicalInterface.sendTimeTriggered(localCPI, period);
		
		
        networkCarLevelPosition = CanMailbox.getReadableCanMailbox(
                MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
        mCarLevelPosition = new IntCanPayloadTranslator(
                networkCarLevelPosition);
        canInterface.registerTimeTriggered(networkCarLevelPosition);
		
	
		
		//Set Current State to Display
		state = State.STATE_DISPLAY;
		
		//register all mAtFloor
        for (int i = 0; i < Elevator.numFloors; i++) {
            int floor = i + 1;
            for (Hallway h : Hallway.replicationValues) {
                int index = ReplicationComputer.computeReplicationId(floor, h);
                ReadableCanMailbox m = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + index);
                BitCanPayloadTranslator t = new BitCanPayloadTranslator(m);
                canInterface.registerTimeTriggered(m);
                mAtFloor.put(index, t);
            }
        }
		
		//Start timer
		timer.start(period);

	}

	@Override
	public void timerExpired(Object callbackData) {
		State newState = state;
	    boolean isAtFloor = false;
	    int index;
	    int position = mCarLevelPosition.getValue();
		
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
			case STATE_DISPLAY:
				//look up the current floor.
				
				
							// State Actions to Display Current Floor
							localCPI.set(currentFloor);
							
			
			
				//Sets State to iterate Continuously 
				newState = State.STATE_DISPLAY;
			
				break;
			default:
				throw new RuntimeException("State" + state
					+ "was not recognized");
		}
		
		if (state == newState) {
            log("Remaining in state: ",state);
        }
		
		state = newState;
		setState(STATE_KEY, state.toString());
		
        // Schedule the next iteration of the controller
        // You must do this at the end of the timer callback in order to
        // restart the timer
		timer.start(period);
	}

}
