/* 
 * Course and Semester : 18-649 Fall 2013
 * Group No: 16
 * Group Members : Jiangtian Nie(jnie) , Yue Chen(yuechen),
 *                 Sally Stevenson(ststeven) , Sri Harsha Koppaka(skoppaka)
 * Author : Jiangtian Nie
 * AndrewID : jnie
 */

package simulator.elevatorcontrol;

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
    int currentFloor;

    private enum State {
        STATE_DISPLAY;
    }

    State currentState = State.STATE_DISPLAY;
    WriteableCarPositionIndicatorPayload localCPI;
    IntegerCanPayloadTranslator mCPI;
    CarLevelPositionCanPayloadTranslator networkCarLevelPosTranslator;
    IntegerCanPayloadTranslator networkDriveSpeedTranslator;
    IntegerCanPayloadTranslator networkDesiredFloorTranslator;
    Utility.AtFloorArray atFloorArray;
    WriteableCanMailbox networkCPI;

    public CarPositionControl(boolean verbose) {
        super("CarPositionControl", verbose);
        localCPI = CarPositionIndicatorPayload.getWriteablePayload();
        networkCPI= CanMailbox.getWriteableCanMailbox(
                MessageDictionary.CAR_POSITION_CAN_ID );
        physicalInterface.sendTimeTriggered(localCPI, period);
        mCPI=new IntegerCanPayloadTranslator(networkCPI);
        canInterface.sendTimeTriggered(networkCPI, period);
        ReadableCanMailbox readablecanmailbox = 
                CanMailbox.getReadableCanMailbox(
                        MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
        networkCarLevelPosTranslator =
                new CarLevelPositionCanPayloadTranslator(readablecanmailbox);
        canInterface.registerTimeTriggered(readablecanmailbox);
        readablecanmailbox = 
                CanMailbox.getReadableCanMailbox(
                        MessageDictionary.DRIVE_SPEED_CAN_ID);
        networkDriveSpeedTranslator =
                new IntegerCanPayloadTranslator(readablecanmailbox);
        canInterface.registerTimeTriggered(readablecanmailbox);
        readablecanmailbox =
                CanMailbox.getReadableCanMailbox(
                        MessageDictionary.DESIRED_FLOOR_CAN_ID);
        networkDesiredFloorTranslator =
                new IntegerCanPayloadTranslator(readablecanmailbox);
        canInterface.registerTimeTriggered(readablecanmailbox);
        atFloorArray = new Utility.AtFloorArray(canInterface);
        currentState = State.STATE_DISPLAY;
        timer.start(period);

    }

    @Override
    public void timerExpired(Object callbackData) {
        // TODO Auto-generated method stub
        currentFloor = atFloorArray.getCurrentFloor();
        State newState = currentState;
        switch (currentState) {
        case STATE_DISPLAY:
            if (atFloorArray.isAtFloor(2, Hallway.FRONT)
                    || atFloorArray.isAtFloor(2, Hallway.BACK)) {
                newState = State.STATE_DISPLAY;
                localCPI.set(currentFloor);
                mCPI.set(currentFloor);
            }
            break;
        default:
            throw new RuntimeException("State" + currentState +
                                       "was not recognized");
        }
        currentState = newState;
        setState(STATE_KEY, currentState.toString());
        timer.start(period);
    }
}
