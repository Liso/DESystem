/* 
 * Course and Semester : 18-649 Fall 2013
 * Group No: 16
 * Group Members : Jiangtian Nie(jnie) , Yue Chen(yuechen),
 *                 Sally Stevenson(ststeven) , Sri Harsha Koppaka(skoppaka)
 * Author : Sri Harsha Koppaka
 * AndrewID : skoppaka
 */

package simulator.elevatorcontrol;

import java.util.HashMap;

import jSimPack.SimTime;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CarLanternPayload;
import simulator.payloads.CarLanternPayload.WriteableCarLanternPayload;



/* Lantern Control is used to light up the car lanterns that are used to
 * indicate to the passenger the direction the elevator is going to travel after 
 * it leaves the current floor.
 */

public class LanternControl extends Controller{
	
    /**************************************************************************
     * Declarations
     *************************************************************************/
	
	//local Physical state for CarLantern
	private WriteableCarLanternPayload localCarLantern;
	
	//Network Interfaces 
	private HashMap<Integer, BitCanPayloadTranslator> mAtFloor = new HashMap<Integer, BitCanPayloadTranslator>();
	private HashMap<Integer, BitCanPayloadTranslator> mDoorClosed = new HashMap<Integer, BitCanPayloadTranslator>();
 // Receive DesiredFloor Message
    private ReadableCanMailbox networkDesiredFloor;
    // Translator for DesiredFloor message -- Specific to Message
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    

    // These variables keep track of current instance

    private final Direction direction;
    
 // Store period for the controller from the Message Dictionary.
    private SimTime period;
    
    // Enumerate States
    private enum State {
        STATE_OFF,
        STATE_ON,
    }

    // State Variable initialized to initial state OFF.
    private State state = State.STATE_OFF;
	

	public LanternControl(Direction direction, SimTime period, boolean verbose) {
		super("LanternControl" + ReplicationComputer.makeReplicationString(direction),verbose);
		
			
		 // Create a can mailbox for DesiredFloor.
        networkDesiredFloor = CanMailbox.getReadableCanMailbox(
                MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(
                networkDesiredFloor);
        // Register for Updates
        canInterface.registerTimeTriggered(networkDesiredFloor);
        
        this.direction =  direction;   
        this.period = period;
        // Add creation of Controller to Log.
        log("Created LanternControl with period = ", period);
        
        //Create a payload object for the Lantern
		localCarLantern = CarLanternPayload.getWriteablePayload(direction);
		//Register payload to be sent periodically
		physicalInterface.sendTimeTriggered(localCarLantern, period);
		
        
		//Register for AtFloor messages
		for (int i = 0; i < Elevator.numFloors; i++) {
            int floor = i + 1;
            for (Hallway h : Hallway.replicationValues) {
                int index = ReplicationComputer.computeReplicationId(floor, h);
                ReadableCanMailbox networkAtFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + index);
                BitCanPayloadTranslator t = new BitCanPayloadTranslator(networkAtFloor);
                canInterface.registerTimeTriggered(networkAtFloor);
                mAtFloor.put(index, t);
            }
        }
        
		//Register for DoorClosed Messages
		for(Hallway h : Hallway.replicationValues){
			for(Side s : Side.values()){
				int index = ReplicationComputer.computeReplicationId(h, s);
				ReadableCanMailbox networkDoorClosed = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + index);
				BitCanPayloadTranslator d = new BitCanPayloadTranslator(networkDoorClosed);
				canInterface.registerTimeTriggered(networkDoorClosed);
				mDoorClosed.put(index, d);
			}
		}
		
		//Start the periodic timer.
		timer.start(period);
		
	}

	
	public void timerExpired(Object callbackData){
		State newState = state;
		
		switch(state){
			case STATE_OFF:
				//State actions for STATE OFF
				
				localCarLantern.set(false);
				
				boolean aDoorIsOpen = false;
				for(Hallway h : Hallway.replicationValues){
					for(Side s : Side.values()){
						int index = ReplicationComputer.computeReplicationId(h, s);
						aDoorIsOpen = aDoorIsOpen || !mDoorClosed.get(index).getValue();
					}
				}
				
				
				
				// #transition 'T7.2'
				for (int i = 0; i < Elevator.numFloors; i++) {
                    int floor = i + 1;
                    for (Hallway h : Hallway.replicationValues) {
                        int index = ReplicationComputer.computeReplicationId(floor, h);
                        if (mAtFloor.get(index).getValue() && aDoorIsOpen && (mDesiredFloor.getDirection() != Direction.STOP) 
				&& (direction == mDesiredFloor.getDirection())){
            							newState = State.STATE_ON;
                        }
                    }
				}
				
				break;
				
			case STATE_ON :
				//State actions for State ON
				
				localCarLantern.set(true);
				
				boolean allDoorsClosed = true;
				// #transition 'T7.1'
				for(Hallway h : Hallway.replicationValues){
					for(Side s : Side.values()){
						int index = ReplicationComputer.computeReplicationId(h, s);
						allDoorsClosed = allDoorsClosed && mDoorClosed.get(index).getValue();
					}
				}
				
				if(allDoorsClosed || mDesiredFloor.getDirection() != direction){
					newState = State.STATE_OFF;
				}
				
				break;
				
			default:
                throw new RuntimeException("State " + state + " was not recognized.");
		}
		
		if (state == newState) {
            log("remains in state: ",state);
        } else {
            log("Transition:",state,"->",newState);

        }

        state = newState;
        setState(STATE_KEY,newState.toString());

        // Schedule the next iteration of the controller
        // You must do this at the end of the timer callback in order to restart
        // the timer
        timer.start(period);
	}
}
