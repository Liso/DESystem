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
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.DoorCommand;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.framework.Speed;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.DoorMotorPayload;
import simulator.payloads.DoorMotorPayload.WriteableDoorMotorPayload;
/**
* @author Yue Chen
*/
public class DoorControl extends Controller {

    /**************************************************************************
     * Declarations
     *************************************************************************/
    //note that inputs are Readable objects, while outputs are Writeable objects

    //local physical state
    private WriteableDoorMotorPayload localDoorMotor;
    
    //network interface
    // command door motor
    private WriteableCanMailbox networkDoorMotor;
    // translator for the door motor command message -- this is a generic translator
    private DoorMotorCommandCanPayloadTranslator mDoorMotor;

    //received door opened message
    private ReadableCanMailbox networkDoorOpened;
    //translator for the doorOpened message -- this translator is specific
    private BitCanPayloadTranslator mDoorOpened;
    
    //received door reversal message
    private ReadableCanMailbox networkDoorReversal;
    //translator for the doorReversal message -- this translator is specific
    private BitCanPayloadTranslator mDoorReversal;
    
    //received door reversal message for Opp Side
    private ReadableCanMailbox networkDoorReversalOpp;
    //translator for the doorReversal message -- this translator is specific
    private BitCanPayloadTranslator mDoorReversalOpp;
    //received car weight message
    private ReadableCanMailbox networkCarWeight;
    //translator for the CarWeight message -- this translator is specific
    private IntCanPayloadTranslator mCarWeight;
    
    //received door closed message
    private ReadableCanMailbox networkDoorClosed;
    //translator for the doorClosed message -- this translator is specific
    private BitCanPayloadTranslator mDoorClosed;
    
    //received at floor message
    private HashMap<Integer, BitCanPayloadTranslator> mAtFloor = new HashMap<Integer, BitCanPayloadTranslator>();
    
    private HashMap<Integer, BitCanPayloadTranslator> mCarCall = new HashMap<Integer, BitCanPayloadTranslator>();
    
    private HashMap<Integer, BitCanPayloadTranslator> mHallCall = new HashMap<Integer, BitCanPayloadTranslator>();
    
    //received desired floor message
    private ReadableCanMailbox networkDesiredFloor;
    //translator for the DesiredFloor message -- this translator is specific
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    
    //received drive speed message
    private ReadableCanMailbox networkDriveSpeed;
    //translator for the DriveCommand message -- this translator is specific
    private DriveSpeedCanPayloadTranslator mDriveSpeed;
   
    //these variables keep track of which instance this is.
    private final Hallway hallway;
    private final Side side;
    private final Side Oppside;
    private Direction direction;
    private int currentFloor;
    private int indexHallCallUp;
    private int indexHallCallDown;
    private int indexCarCall;
    private int reversalCount = 0;
    
    //store the period for the controller
    private SimTime period;
    
    //additional internal state variables
    private SimTime countDown = SimTime.ZERO;
    private final static SimTime dwell = new SimTime(7,
            SimTime.SimTimeUnit.SECOND);

    //enumerate states
    private enum State {
        STATE_OPEN,
        STATE_STOP_OPENING,
        STATE_CLOSE,
        STATE_STOP_CLOSING,
        STATE_REVERSING,
        STATE_NUDGE,
    }
    
    //state variable initialized to the initial state FLASH_OFF
    private State state = State.STATE_CLOSE;

    /**
     * The arguments listed in the .cf configuration file should match the
     * order and type given here.
     *
     * For your elevator controllers, you should make sure that the constructor
     * matches the method signatures in ControllerBuilder.makeAll().
     */
    public DoorControl(Hallway hallway, Side side, SimTime period, boolean verbose) {
        // call to the Controller superclass constructor is required
        super("DoorControl" +
              ReplicationComputer.makeReplicationString(hallway, side),
              verbose);
        
        // stored the constructor arguments in internal state
        this.period = period;
        this.hallway = hallway;
        this.side = side;
     if(side == Side.LEFT){
        Oppside = Side.RIGHT;
     }
     else
        Oppside = Side.LEFT;
        currentFloor = 1; 

        log("Created DoorControl with period = ", period);
    
        // Initialize physical interface
        localDoorMotor = DoorMotorPayload.getWriteablePayload(hallway, side);
        physicalInterface.sendTimeTriggered(localDoorMotor, period);

        // Initialize network interface
        
        // Create a can mailbox - this object has the binary representation of
        // the message data the CAN message ids are declared in the
        // MessageDictionary class.  The ReplicationComputer class provides
        // utility methods for computing offsets for replicated controllers
        networkDoorMotor = CanMailbox.getWriteableCanMailbox(
                MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID +
                ReplicationComputer.computeReplicationId(hallway, side));
        mDoorMotor = new DoorMotorCommandCanPayloadTranslator(
                networkDoorMotor, hallway, side);
        canInterface.sendTimeTriggered(networkDoorMotor, period);
        networkDoorOpened = CanMailbox.getReadableCanMailbox(
                MessageDictionary.DOOR_OPEN_SENSOR_BASE_CAN_ID +
                ReplicationComputer.computeReplicationId(hallway, side));
        mDoorOpened = new BitCanPayloadTranslator(networkDoorOpened);
        canInterface.registerTimeTriggered(networkDoorOpened);
        
        networkDoorReversal = CanMailbox.getReadableCanMailbox(
                MessageDictionary.DOOR_REVERSAL_SENSOR_BASE_CAN_ID +
                ReplicationComputer.computeReplicationId(hallway, side));
        mDoorReversal = new BitCanPayloadTranslator(
                networkDoorReversal);
        canInterface.registerTimeTriggered(networkDoorReversal);

     networkDoorReversalOpp = CanMailbox.getReadableCanMailbox(
                MessageDictionary.DOOR_REVERSAL_SENSOR_BASE_CAN_ID +
                ReplicationComputer.computeReplicationId(hallway, Oppside));
        mDoorReversalOpp = new BitCanPayloadTranslator(
                networkDoorReversalOpp);
        canInterface.registerTimeTriggered(networkDoorReversalOpp);

        networkCarWeight = CanMailbox.getReadableCanMailbox(
                MessageDictionary.CAR_WEIGHT_CAN_ID);
        mCarWeight = new IntCanPayloadTranslator(networkCarWeight);
        canInterface.registerTimeTriggered(networkCarWeight);

        networkDoorClosed = CanMailbox.getReadableCanMailbox(
                MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID +
                ReplicationComputer.computeReplicationId(hallway, side));
        mDoorClosed = new BitCanPayloadTranslator(networkDoorClosed);
        canInterface.registerTimeTriggered(networkDoorClosed);

        networkDesiredFloor = CanMailbox.getReadableCanMailbox(
                MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(
                networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDesiredFloor);
        
        for (int i = 0; i < Elevator.numFloors; i++) {
            int floor = i + 1;
            for (Hallway h : Hallway.replicationValues) {
                
                int index = ReplicationComputer.computeReplicationId(floor, h);
                ReadableCanMailbox m = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + index);
                BitCanPayloadTranslator t = new BitCanPayloadTranslator(m);
                canInterface.registerTimeTriggered(m);
                mAtFloor.put(index, t);
                

                ReadableCanMailbox networkCarCall = CanMailbox.getReadableCanMailbox(
                        MessageDictionary.CAR_CALL_BASE_CAN_ID + 
                        ReplicationComputer.computeReplicationId(floor, hallway));
                BitCanPayloadTranslator nCarCall = new BitCanPayloadTranslator(networkCarCall);
                canInterface.registerTimeTriggered(networkCarCall);
                mCarCall.put(index, nCarCall);
                
                for (Direction d : Direction.replicationValues) {
                    int indexHallCall = ReplicationComputer.computeReplicationId(floor, h, d);
                    ReadableCanMailbox networkHallCall = CanMailbox.getReadableCanMailbox(
                            MessageDictionary.HALL_CALL_BASE_CAN_ID + 
                            ReplicationComputer.computeReplicationId(floor, hallway, d));
                    BitCanPayloadTranslator nHallCall = new BitCanPayloadTranslator(networkHallCall);
                    canInterface.registerTimeTriggered(networkHallCall);
                    mHallCall.put(indexHallCall, nHallCall);
                }

            }
        }
        
        networkDriveSpeed = CanMailbox.getReadableCanMailbox(
                MessageDictionary.DRIVE_SPEED_CAN_ID);
        mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeed);
        canInterface.registerTimeTriggered(networkDriveSpeed);
        
        direction = mDesiredFloor.getDirection();
        timer.start(period);
    }

    /*
     * The timer callback is where the main controller code is executed.  For
     * time triggered design, this consists mainly of a switch block with a
     * case block for each state.  Each case block executes actions for that
     * state, then executes a transition to the next state if the transition
     * conditions are met.
     */
    public void timerExpired(Object callbackData) {
        State newState = state;
        direction = mDesiredFloor.getDirection();
        for (int i = 0; i < Elevator.numFloors; i++) {
            int floor = i + 1;
            int index = ReplicationComputer.computeReplicationId(floor, hallway);
            if (mAtFloor.get(index).getValue()) {
                indexHallCallUp = ReplicationComputer.computeReplicationId(floor, hallway, Direction.UP);
                indexHallCallDown = ReplicationComputer.computeReplicationId(floor, hallway, Direction.DOWN);
                indexCarCall = ReplicationComputer.computeReplicationId(floor, hallway);
                currentFloor = floor;
                    break;
            }
        }
                
        
        switch (state) {
            case STATE_OPEN:
                // state actions for 'DOOR OPEN'
                localDoorMotor.set(DoorCommand.OPEN);
                mDoorMotor.set(DoorCommand.OPEN);
                countDown = SimTime.add(SimTime.ZERO, dwell);
                
                // #transition 'T5.1'
                if (mDoorOpened.getValue()) {
                    newState = State.STATE_STOP_OPENING;
                }
                break;
            case STATE_NUDGE:
                // state actions for 'DOOR NUDGE'
                localDoorMotor.set(DoorCommand.NUDGE);
                mDoorMotor.set(DoorCommand.NUDGE);
                
                //Index for getting current AtFloor
                int index2 = ReplicationComputer.computeReplicationId(currentFloor, hallway);
                
                //#transition 'T5.9'
                if (mAtFloor.get(index2).getValue()) {
                    if ((mDriveSpeed.getSpeed() == Speed.STOP) &&
                            ((mCarWeight.getValue() >= Elevator.MaxCarCapacity) || 
                             mCarCall.get(indexCarCall).getValue())) {
                        newState = State.STATE_OPEN;
                    }
                    else if ((mDriveSpeed.getSpeed() == Speed.STOP) &&
                             ((direction == Direction.STOP) ||
                              (direction == Direction.UP)) &&
                             mHallCall.get(indexHallCallUp).getValue()) {
                        newState = State.STATE_OPEN;
                    }
                    else if ((mDriveSpeed.getSpeed() == Speed.STOP) &&
                             ((direction == Direction.STOP) ||
                              (direction == Direction.DOWN)) &&
                             mHallCall.get(indexHallCallDown).getValue()) {
                        newState = State.STATE_OPEN;
                    }
                }
                
                //#transition 'T5.10'
                if (mDoorClosed.getValue()) {
                    newState = State.STATE_STOP_CLOSING;
                }
                break;
            case STATE_REVERSING:
                // state actions for 'REVERSING'
                localDoorMotor.set(DoorCommand.OPEN);
                mDoorMotor.set(DoorCommand.OPEN);
                countDown = SimTime.add(SimTime.ZERO, dwell);
                reversalCount++;
                
                //#transition 'T5.5'
                newState = State.STATE_OPEN;
                break;
            case STATE_STOP_OPENING:
                // state actions for 'DOOR STOP OPENING'
                localDoorMotor.set(DoorCommand.STOP);
                mDoorMotor.set(DoorCommand.STOP);
                countDown = SimTime.subtract(countDown, period);
                
                if (countDown.isLessThanOrEqual(SimTime.ZERO) && 
                        !mDoorReversal.getValue() &&
                        !mDoorReversalOpp.getValue() &&
                        (mCarWeight.getValue() < Elevator.MaxCarCapacity)) {
                    if (reversalCount >= 3) {
                        // #transition 'T5.8'
                        newState = State.STATE_NUDGE;
                    }
                    else {
                        // #transition '5.2'
                        newState = State.STATE_CLOSE;
                    }
                }
                break;
            case STATE_CLOSE:
                // state actions for 'DOOR NOT CLOSED'
                localDoorMotor.set(DoorCommand.CLOSE);
                mDoorMotor.set(DoorCommand.CLOSE);
                
                //Index for getting current AtFloor
                int index = ReplicationComputer.computeReplicationId(currentFloor, hallway);
                
                if (mAtFloor.get(index).getValue()) {
                    //#transition 'T5.4'
                    if ((mDriveSpeed.getSpeed() == Speed.STOP) &&
                            (mDoorReversal.getValue() ||
                             mDoorReversalOpp.getValue())) {
                        newState = State.STATE_REVERSING;
                    }
                    //#transition 'T5.3'
                    else if ((mDriveSpeed.getSpeed() == Speed.STOP) &&
                             ((mCarWeight.getValue() >= Elevator.MaxCarCapacity) || 
                              mCarCall.get(indexCarCall).getValue())) {
                        newState = State.STATE_OPEN;
                    }
                    //#transition 'T5.3'
                    else if ((mDriveSpeed.getSpeed() == Speed.STOP) &&
                             mHallCall.get(indexHallCallUp).getValue() &&
                             ((direction == Direction.STOP) ||
                              (direction == Direction.UP))) {
                        newState = State.STATE_OPEN;
                    }
                    //#transition 'T5.3'
                    else if ((mDriveSpeed.getSpeed() == Speed.STOP) &&
                             mHallCall.get(indexHallCallDown).getValue() &&
                             ((direction == Direction.STOP) ||
                              (direction == Direction.DOWN))) {
                        newState = State.STATE_OPEN;
                    }
                }
                
                //#transition 'T5.6'
                if (mDoorClosed.getValue()) {
                    newState = State.STATE_STOP_CLOSING;
                }
                break;
            case STATE_STOP_CLOSING:
                //state actions for 'DOOR STOP CLOSING'
                localDoorMotor.set(DoorCommand.STOP);
                mDoorMotor.set(DoorCommand.STOP);
                reversalCount = 0;

                //Index for getting current AtFloor
                int index1 = ReplicationComputer.computeReplicationId(currentFloor, hallway);             

                // System.out.println("doorcontrol[" + hallway + "]: " + mDesiredFloor.getHallway());
                //#transition 'T5.7'
                if(mAtFloor.get(index1).getValue()){
                 
                    if ((mDesiredFloor.getFloor() == currentFloor) &&
                            ((mDesiredFloor.getHallway() == hallway) || 
                             (mDesiredFloor.getHallway() == Hallway.BOTH)) &&
                            (mDriveSpeed.getDirection() == Direction.STOP) && 
                            (mDriveSpeed.getSpeed() == Speed.STOP)) {
                        newState = State.STATE_OPEN;
                    }
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

        // schedule the next iteration of the controller
        // you must do this at the end of the timer callback in order to
        // restart the timer
        timer.start(period);
    }
}
