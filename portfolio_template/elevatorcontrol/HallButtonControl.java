/* 
 * Course and Semester : 18-649 Fall 2013
 * Group No: 16
 * Group Members : Jiangtian Nie(jnie) , Yue Chen(yuechen),
 *                 Sally Stevenson(ststeven) , Sri Harsha Koppaka(skoppaka)
 * Author : Sri Harsha Koppaka
 * AndrewID : skoppaka
 */



package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.HallCallPayload;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;
import simulator.payloads.HallLightPayload;
import simulator.payloads.HallLightPayload.WriteableHallLightPayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;


/*
 * HallButton Control is used to take input Hall Calls from the passengers
 * and set HallLight ON or OFF depending on the location and desired direction 
 * of elevator.
 */

public class HallButtonControl extends Controller {

    /**************************************************************************
     * Declarations
     *************************************************************************/

    // Inputs are Readable objects while outputs are Writeable objects.

    // local Physical State of HallCall & HallLight 
    // Receive Hall Calls from buttons
    private ReadableHallCallPayload   localHallCall;
    private WriteableHallLightPayload localHallLight; 

    // Network Interfaces

    // Send network message for HallCall
    private WriteableCanMailbox networkHallCall;
    // Translator for HallCall message -- Generic Translator
    private BooleanCanPayloadTranslator mHallCall;

    // Send network message for HallLight
    private WriteableCanMailbox networkHallLightOut;
    // Translator for HallLight message -- Generic Translator
    private BooleanCanPayloadTranslator mHallLight;

    // Receive AtFloor Message
    private ReadableCanMailbox networkAtFloor;
    // Translator for AtFloor message -- Specific to Message
    private AtFloorCanPayloadTranslator mAtFloor;

    // Receive DesiredFloor Message
    private ReadableCanMailbox networkDesiredFloor;
    // Translator for DesiredFloor message -- Specific to Message
    private DesiredFloorCanPayloadTranslator mDesiredFloor;

    // Receive DoorClosedFrontLeft
    private ReadableCanMailbox networkDoorClosedFrontLeft;
    // Translator for DoorClosed message -- Specific to Message
    private DoorClosedCanPayloadTranslator mDoorClosedFrontLeft;

    // Receive DoorClosedFrontRight
    private ReadableCanMailbox networkDoorClosedFrontRight;
    // Translator for DoorClosed message -- Specific to Message
    private DoorClosedCanPayloadTranslator mDoorClosedFrontRight;

    // Receive DoorClosedBackLeft
    private ReadableCanMailbox networkDoorClosedBackLeft;
    // Translator for DoorClosed message -- Specific to Message
    private DoorClosedCanPayloadTranslator mDoorClosedBackLeft;

    // Receive DoorClosedBackRight
    private ReadableCanMailbox networkDoorClosedBackRight;
    // Translator for DoorClosed message -- Specific to Message
    private DoorClosedCanPayloadTranslator mDoorClosedBackRight;        

    // These variables keep track of current instance

    private final Direction direction;

    // Store period for the controller from the Message Dictionary.
    private SimTime period = MessageDictionary.HALL_BUTTON_CONTROL_PERIOD;

    // Enumerate States
    private enum State {
        STATE_OFF,
        STATE_ON,
    }

    // State Variable initialized to initial state OFF.
    private State state = State.STATE_OFF;

    // Constructor for the HallButtonControl Class. Arguments in .cf file
    // should match order and type given here.
    public HallButtonControl(int floor, Hallway hallway, Direction direction,
                             boolean verbose) {
        // Call to the Controller superclass.
        super("HallButtonControl" +
              ReplicationComputer.makeReplicationString(floor, hallway,
                                                        direction), verbose);

        // Constructor arguments stored in local variable.
        this.direction = direction;

        // Add creation of Controller to Log.
        log("Created HallButtonControl with period = ", period);

        // Initialize physical state

        // Create a payload object for this floor, hallway, and direction
        // using the static factory method in HallCallPayload.
        localHallCall = HallCallPayload.getReadablePayload(floor, hallway,
                                                           direction);
        // Register the payload with the physical interface (as in input) --
        // it will be updated periodically when the hall call button state is
        // modified.
        physicalInterface.registerTimeTriggered(localHallCall);

        // Create a payload object for this floor, hallway, and direction. This
        // is an output, so it is created with the writeable static factory
        // method
        localHallLight = HallLightPayload.getWriteablePayload(floor, hallway,
                direction);
        // Register the payload to be sent periodically -- whatever value is
        // stored in the localHallLight object will be sent out periodically
        // with the period specified by the period parameter.
        physicalInterface.sendTimeTriggered(localHallLight, period);

        // Initialize network interface

        // Create a can mailbox for HallCall - this object has the binary
        // representation of the message data. The CAN message ids are declared
        // in the MessageDictionary class.  The ReplicationComputer class
        // provides utility methods for computing offsets for replicated
        // controllers.
        networkHallCall = CanMailbox.getWriteableCanMailbox(
                MessageDictionary.HALL_CALL_BASE_CAN_ID +
                ReplicationComputer.computeReplicationId(floor, hallway,
                                                         direction));
        // Create a translator with a reference to the CanMailbox.  Use the 
        // translator to read and write values to the mailbox
        mHallCall = new BooleanCanPayloadTranslator(networkHallCall);
        // Register the mailbox to have its value broadcast on the network
        // periodically with a period specified by the period parameter.
        canInterface.sendTimeTriggered(networkHallCall, period);

        // Create a can mailbox for HallLight.
        networkHallLightOut = CanMailbox.getWriteableCanMailbox(
                MessageDictionary.HALL_LIGHT_BASE_CAN_ID +
                ReplicationComputer.computeReplicationId(floor, hallway,
                                                         direction));
        // Create a translator for HallLight.
        mHallLight = new BooleanCanPayloadTranslator(networkHallLightOut);
        // Broadcast HallLight on network periodically
        canInterface.sendTimeTriggered(networkHallLightOut, period);

        // Create a can mailbox for AtFloor.
        networkAtFloor = CanMailbox.getReadableCanMailbox(
                MessageDictionary.AT_FLOOR_BASE_CAN_ID +
                ReplicationComputer.computeReplicationId(floor, hallway));
        mAtFloor = new AtFloorCanPayloadTranslator(networkAtFloor, floor,
                                                   hallway);
        // Register to receive periodic updates to the mailbox via the CAN
        // network. The period of updates will be determined by the sender of
        // the message
        canInterface.registerTimeTriggered(networkAtFloor);

        // Create a can mailbox for DesiredFloor.
        networkDesiredFloor = CanMailbox.getReadableCanMailbox(
                MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(
                networkDesiredFloor);
        // Register for Updates
        canInterface.registerTimeTriggered(networkDesiredFloor);

        // Create a can mailbox for DoorClosedFront Left
        networkDoorClosedFrontLeft = CanMailbox.getReadableCanMailbox(
                MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID +
                ReplicationComputer.computeReplicationId(Hallway.FRONT,
                                                         Side.LEFT));
        mDoorClosedFrontLeft = new DoorClosedCanPayloadTranslator(
                networkDoorClosedFrontLeft, Hallway.FRONT, Side.LEFT);
        // Register for Updates
        canInterface.registerTimeTriggered(networkDoorClosedFrontLeft);

        // Create a can mailbox for DoorClosedFrontRight
        networkDoorClosedFrontRight = CanMailbox.getReadableCanMailbox(
                MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID +
                ReplicationComputer.computeReplicationId(Hallway.FRONT,
                                                         Side.RIGHT));
        mDoorClosedFrontRight = new DoorClosedCanPayloadTranslator(
                networkDoorClosedFrontRight, Hallway.FRONT, Side.RIGHT);
        // Register for Updates
        canInterface.registerTimeTriggered(networkDoorClosedFrontRight);

        // Create a can mailbox for DoorClosedBackLeft
        networkDoorClosedBackLeft = CanMailbox.getReadableCanMailbox(
                MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID +
                ReplicationComputer.computeReplicationId(Hallway.BACK,
                                                         Side.LEFT));
        mDoorClosedBackLeft = new DoorClosedCanPayloadTranslator(
                networkDoorClosedBackLeft, Hallway.BACK, Side.LEFT);
        // Register for Updates
        canInterface.registerTimeTriggered(networkDoorClosedBackLeft);

        // Create a can mailbox for DoorClosedBackRight
        networkDoorClosedBackRight = CanMailbox.getReadableCanMailbox(
                MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID +
                ReplicationComputer.computeReplicationId(Hallway.BACK, 
                                                         Side.RIGHT));
        mDoorClosedBackRight = new DoorClosedCanPayloadTranslator(
                networkDoorClosedBackRight, Hallway.BACK, Side.RIGHT);
        // Register for Updates
        canInterface.registerTimeTriggered(networkDoorClosedBackRight);

        // Starts Timer that schedules this code to expire after time
        // interval has passed, at which point timerExpired will be called.
        timer.start(period);
    }

    /*
     * The timer callback is where the main controller code is executed.  For time
     * triggered design, this consists mainly of a switch block with a case block for
     * each state.  Each case block executes actions for that state, then executes
     * a transition to the next state if the transition conditions are met.
     */
    public void timerExpired(Object callbackData) {
        State newState = state;
        boolean aDoorIsOpen = !mDoorClosedFrontLeft.getValue() ||
                              !mDoorClosedFrontRight.getValue() ||
                              !mDoorClosedBackLeft.getValue() ||
                              !mDoorClosedBackRight.getValue();

        switch (state) {
            case STATE_OFF:
                // State actions for 'OFF'
                localHallLight.set(false);
                mHallCall.set(false);
                mHallLight.set(false);

                // #transition 'T8.1'
                if(localHallCall.pressed() == true){
                    newState = State.STATE_ON;
                }

                break;

            case STATE_ON:
                // State actions for 'ON'
                localHallLight.set(true);
                mHallCall.set(true);
                mHallLight.set(true);

                // #transition 'T8.2'
                if (!localHallCall.pressed() && mAtFloor.getValue() &&
                      aDoorIsOpen &&
                      ((mDesiredFloor.getDirection() == direction) ||
                       (mDesiredFloor.getDirection() == Direction.STOP))) {
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
