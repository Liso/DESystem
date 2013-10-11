package simulator.elevatorcontrol;

import java.util.LinkedHashMap;
import jSimPack.SimTime;
import simulator.elevatormodules.*;
import simulator.framework.*;
import simulator.payloads.*;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.IntegerCanPayloadTranslator;
import simulator.payloads.CarPositionIndicatorPayload.*;

public class CarPositionControl extends Controller {

	private SimTime period = MessageDictionary.CAR_POSITION_CONTROL_PERIOD;
	// Translator for AtFloor message -- Specific to Message

	private enum State {
		STATE_DISPLAY;
	}
	
	//current state
	State currentState = State.STATE_DISPLAY;
	
	//physical carposition indicator;
	WriteableCarPositionIndicatorPayload localCPI;
	//network carposition indicator
	IntegerCanPayloadTranslator mCPI;
	CarLevelPositionCanPayloadTranslator networkCarLevelPosTranslator;
	//network message for futrue extention
	IntegerCanPayloadTranslator networkDriveSpeedTranslator;
	IntegerCanPayloadTranslator networkDesiredFloorTranslator;
	WriteableCanMailbox networkCPI;
	ReadableCanMailbox atfloor;
	//define all mAtFloor in an array;
	private ReadableCanMailbox[][] networkAtFloor=new ReadableCanMailbox[8][2];
	//define a hashmap to represent 4 status of hallway. 
	LinkedHashMap<Integer, Hallway> hallway=new LinkedHashMap<Integer, Hallway>();
	AtFloorCanPayloadTranslator mAtFloor;

	public CarPositionControl(boolean verbose) {
		// TODO Auto-generated constructor stub
		super("CarPositionControl", verbose);
		hallway.put(1, Hallway.FRONT);
		hallway.put(2, Hallway.BACK);
		hallway.put(3, Hallway.BOTH);
		hallway.put(4, Hallway.NONE);
		
		//Send Physical message to CarPositionIndicator
		localCPI = CarPositionIndicatorPayload.getWriteablePayload();
		networkCPI = CanMailbox
				.getWriteableCanMailbox(MessageDictionary.CAR_POSITION_CAN_ID);
		atfloor = CanMailbox
				.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID);
		physicalInterface.sendTimeTriggered(localCPI, period);
		
		//Send network Message mCarPositionIndicator
		mCPI = new IntegerCanPayloadTranslator(networkCPI);
		canInterface.sendTimeTriggered(networkCPI, period);
		
		
		//Message for CarLevel Position Indicator - Future Expansion
		ReadableCanMailbox networkCarLevelPosIndicator = CanMailbox
				.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
		networkCarLevelPosTranslator = new CarLevelPositionCanPayloadTranslator(
				networkCarLevelPosIndicator);
		canInterface.registerTimeTriggered(networkCarLevelPosIndicator);
		
		currentState = State.STATE_DISPLAY;
		
		//register all mAtFloor
		for (int i = 1; i < 9; i++) {
			for (int j = 1; j < 3; j++) {
				networkAtFloor[i-1][j-1] = CanMailbox.getReadableCanMailbox(
		                MessageDictionary.AT_FLOOR_BASE_CAN_ID +
		                ReplicationComputer.computeReplicationId(i, hallway.get(j)));
				canInterface.registerTimeTriggered(networkAtFloor[i-1][j-1]);
			}
		}
		
		
		timer.start(period);

	}

	@Override
	public void timerExpired(Object callbackData) {
		// TODO Auto-generated method stub

		State newState = currentState;
		
		switch (currentState) {
		
		case STATE_DISPLAY:
			//look up the current floor.
			for (int i = 1; i < 9; i++) {
				for (int j = 1; j < 3; j++) {
					//# transition 'T10.1'
					if ((new AtFloorCanPayloadTranslator(networkAtFloor[i-1][j-1], i,
							hallway.get(j))).getValue()) {
						mAtFloor=new AtFloorCanPayloadTranslator(networkAtFloor[i-1][j-1],
								i, hallway.get(j));
						// State Actions
						localCPI.set(i);
						mCPI.set(i);
						break;
					}
				}
			}
			

			newState = State.STATE_DISPLAY;
			
			
			break;

		default:
			throw new RuntimeException("State" + currentState
					+ "was not recognized");

		}
		currentState = newState;
		setState(STATE_KEY, currentState.toString());
		timer.start(period);
	}

}
