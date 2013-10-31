/* 
 * Course and Semester : 18-649 Fall 2013
 * Group No: 16
 * Group Members : Jiangtian Nie(jnie) , Yue Chen(yuechen),
 *                 Sally Stevenson(ststeven) , Sri Harsha Koppaka(skoppaka)
 * Author : Yue Chen
 * AndrewID : yuechen
 */

package simulator.elevatorcontrol;

import java.util.Collections;
import java.util.HashMap;
import java.util.PriorityQueue;

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
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.DriveSpeedPayload;
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
    private int currentFloor = 0;
    private PriorityQueue<Integer> serviceQueueMin = new PriorityQueue<Integer>();
    private PriorityQueue<Integer> serviceQueueMax = new PriorityQueue<Integer>(Elevator.numFloors, Collections.reverseOrder());

    private int acc = 1;
    private Direction currentDirection = Direction.STOP;

    //store the period for the controller
    private SimTime period;

    //enumerate states
    private enum State {
        STATE_UP,
            STATE_DOWN,
            STATE_ATFLOOR,
            STATE_STOP,
            STATE_RESET,
            STATE_NONE,
    }

    //state variable initialized to the initial state FLASH_OFF
    private State state = State.STATE_STOP;

    //state variable initialized to the initial state FLASH_OFF
    private State parentState = State.STATE_NONE;

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


        int commitPointUp;
        int commitPointDown;
        int nHallway = 0;
        double currentSpeed = localDriveSpeed.speed();
        int position = mCarLevelPosition.getValue();
        int indexHallCall;
        int indexCarCall;

        boolean isAtFloor = false;
        int index = 0;

        for (int i = 0; i < Elevator.numFloors; i++) {
            int floor = i + 1;
            for (Hallway h : Hallway.replicationValues) {

                index = ReplicationComputer.computeReplicationId(floor, h);
                if (mAtFloor.get(index).getValue()) {
                    isAtFloor = true;
                    currentFloor = floor;
                }
            }
        }

        //		int indexHC = ReplicationComputer.computeReplicationId(2, Hallway.FRONT, Direction.UP);
        //                System.out.println(mHallCall.get(indexHC).getValue());
        //                System.out.println(state);
        switch (state) {

            case STATE_STOP:
                //System.out.println("STATE_STOP: hallway -> " + hallway); 
                // state actions for state S11.1 'SET TARGET'
                mDesiredFloor.set(targetFloor, Hallway.NONE, Direction.STOP);
                if (hallway == Hallway.FRONT)
                    mDesiredDwellFront.setDwell(dwell);
                else if (hallway == Hallway.BACK)
                    mDesiredDwellBack.setDwell(dwell);
                else if (hallway == Hallway.BOTH) {
                    mDesiredDwellFront.setDwell(dwell);
                    mDesiredDwellBack.setDwell(dwell);
                }
                else {
                    mDesiredDwellFront.setDwell(SimTime.ZERO);
                    mDesiredDwellBack.setDwell(SimTime.ZERO);
                }


                int flag = 0;
                int bottomFloor = 0;
                int topFloor = Elevator.numFloors;

                if (currentDirection == Direction.UP)
                    bottomFloor = currentFloor - 1;
                else if (currentDirection == Direction.DOWN)
                    topFloor = currentFloor;

                for (int i = bottomFloor; i < topFloor; i++) {
                    int floor = i + 1;
                    for (Hallway h : Hallway.replicationValues) {
                        for (Direction d : Direction.replicationValues) {
                            indexHallCall = ReplicationComputer.computeReplicationId(floor, h, d);
                            if (flag == 0 && mHallCall.get(indexHallCall).getValue()) {
                                flag = 1;
                                targetFloor = floor;
                                if (d == Direction.UP && !serviceQueueMin.contains(floor)){
                                    serviceQueueMin.add(floor);
                                }

                                if (d == Direction.DOWN && !serviceQueueMax.contains(floor)){
                                    serviceQueueMax.add(floor);
                                }

                                if (floor > currentFloor) {
                                    currentDirection = d;
                                    newState = State.STATE_UP;
                                }

                                else if (floor < currentFloor){
                                    currentDirection = d;
                                    newState = State.STATE_DOWN;
                                }

                                else if (floor == currentFloor){
                                    if(d == Direction.UP) {
                                        currentDirection = d;
                                        newState = State.STATE_UP;
                                    }
                                    if(d == Direction.DOWN) {
                                        currentDirection = d;
                                        newState = State.STATE_DOWN;
                                    }
                                }

                            }


                        }
                        indexCarCall = ReplicationComputer.computeReplicationId(floor, h);
                        if (flag == 0 && mCarCall.get(indexCarCall).getValue()) {
                            flag = 1;
                            targetFloor = floor;
                            if (floor >= currentFloor && !serviceQueueMin.contains(floor)) {
                                serviceQueueMin.add(floor);
                                currentDirection = Direction.UP;
                                newState = State.STATE_UP;
                            }

                            else if (floor < currentFloor && !serviceQueueMax.contains(floor)){
                                serviceQueueMax.add(floor);
                                currentDirection = Direction.DOWN;
                                newState = State.STATE_DOWN;
                            }
                        }
                    }
                }


                nHallway = 0;
                hallway = Hallway.NONE;
                for (Hallway h : Hallway.replicationValues) {
                    indexCarCall = ReplicationComputer.computeReplicationId(targetFloor, h);
                    if (mCarCall.get(indexCarCall).getValue()) {
                        nHallway++;
                        if (nHallway >= 2)
                            hallway = Hallway.BOTH;
                        else
                            hallway = h;
                    }
                    if (currentDirection != Direction.STOP) { 
                        indexHallCall = ReplicationComputer.computeReplicationId(targetFloor, h, currentDirection);
                        if (mHallCall.get(indexHallCall).getValue() && !mCarCall.get(indexCarCall).getValue()) {
                            nHallway++;
                            if (nHallway >= 2)
                                hallway = Hallway.BOTH;
                            else
                                hallway = h;
                        }
                    }
                }



                // make sure that the last is false as well
                // #transition 'T11.9'
                if (!(mDoorClosedFrontLeft.getValue() && 
                            mDoorClosedFrontRight.getValue() && 
                            mDoorClosedBackLeft.getValue() && 
                            mDoorClosedBackRight.getValue())) {

                    if (!isAtFloor)
                        newState = State.STATE_RESET;
                }
                break;
            case STATE_UP:
                // state actions for state S11.1 'SET TARGET'
                mDesiredFloor.set(targetFloor, hallway, Direction.UP);
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

                for (int i = currentFloor; i <= Elevator.numFloors; i++) {
                    int floor = i;
                    if (currentSpeed <= 0.25)
                        commitPointUp = 5000*(floor - 1);
                    else
                        commitPointUp = (int)(((5*(floor - 1)) - ((currentSpeed * currentSpeed)/(2*acc))) * 1000) - 200;

                    for (Hallway h : Hallway.replicationValues) {
                        indexHallCall = ReplicationComputer.computeReplicationId(floor, h, Direction.UP);
                        indexCarCall = ReplicationComputer.computeReplicationId(floor, h);
                        if ((mCarCall.get(indexCarCall).getValue() || mHallCall.get(indexHallCall).getValue()) && position <= commitPointUp) {
                            if (!serviceQueueMin.contains(floor))
                                serviceQueueMin.add(floor);
                            currentDirection = Direction.UP;
                        }		
                    }
                }	


                if (serviceQueueMin.isEmpty()){
                    for (int i = currentFloor - 1; i < Elevator.numFloors; i++) {
                        int floor = i + 1;

                        if (currentSpeed <= 0.25)
                            commitPointUp = 5000*(floor - 1);
                        else
                            commitPointUp = (int)(((5*(floor - 1)) - ((currentSpeed * currentSpeed)/(2*acc))) * 1000) - 200;

                        for (Hallway h : Hallway.replicationValues) {
                            indexHallCall = ReplicationComputer.computeReplicationId(floor, h, Direction.DOWN);
                            if (mHallCall.get(indexHallCall).getValue() && position <= commitPointUp) {        			
                                if (!serviceQueueMax.contains(floor))
                                    serviceQueueMax.add(floor);
                                currentDirection = Direction.DOWN;
                            }
                        }
                    }
                }

                parentState = State.STATE_NONE;
                if (currentFloor == targetFloor){
                    if(hallway == Hallway.BOTH){
                        if (isAtFloor) {
                            /*
                               if (!mDoorClosedFrontLeft.getValue() && 
                               !mDoorClosedFrontRight.getValue() && 
                               !mDoorClosedBackLeft.getValue() && 
                               !mDoorClosedBackRight.getValue()) {
                               */
                            if (!serviceQueueMin.isEmpty()) {
                                if (serviceQueueMin.peek() == currentFloor) {
                                    currentDirection = Direction.STOP;
                                    serviceQueueMin.poll();
                                    parentState = State.STATE_UP;
                                }
                            }
                            else if (!serviceQueueMax.isEmpty()) {
                                if (serviceQueueMax.peek() == currentFloor) {
                                    currentDirection = Direction.DOWN;
                                    serviceQueueMax.poll();
                                    parentState = State.STATE_DOWN;
                                }

                            }

                        }
                        }

                        else if(hallway == Hallway.FRONT){
                            if (isAtFloor) {
                                /*
                                   if (!mDoorClosedFrontLeft.getValue() && 
                                   !mDoorClosedFrontRight.getValue()) {
                                   */
                                if (!serviceQueueMin.isEmpty()) {
                                    if (serviceQueueMin.peek() == currentFloor) {
                                        currentDirection = Direction.STOP;
                                        serviceQueueMin.poll();
                                        parentState = State.STATE_UP;
                                    }
                                }
                                else if (!serviceQueueMax.isEmpty()) {
                                    if (serviceQueueMax.peek() == currentFloor) {
                                        currentDirection = Direction.DOWN;
                                        serviceQueueMax.poll();
                                        parentState = State.STATE_DOWN;
                                    }

                                }

                            }
                            }

                            else if(hallway == Hallway.BACK){
                                if (isAtFloor) {
                                    /*
                                       if (!mDoorClosedBackLeft.getValue() && 
                                       !mDoorClosedBackRight.getValue()) {
                                       */
                                    if (!serviceQueueMin.isEmpty()) {
                                        if (serviceQueueMin.peek() == currentFloor) {
                                            currentDirection = Direction.STOP;
                                            serviceQueueMin.poll();
                                            parentState = State.STATE_UP;
                                        }
                                    }
                                    else if (!serviceQueueMax.isEmpty()) {
                                        if (serviceQueueMax.peek() == currentFloor) {
                                            currentDirection = Direction.DOWN;
                                            serviceQueueMax.poll();
                                            parentState = State.STATE_DOWN;
                                        }

                                    }

                                }
                                }
                            }

                            if (parentState != State.STATE_NONE) {
                                newState = State.STATE_ATFLOOR;
                                if (!serviceQueueMin.isEmpty()) {
                                    currentDirection = Direction.UP;
                                }

                            }
                            else if (!serviceQueueMin.isEmpty()) {
                                targetFloor = serviceQueueMin.peek();
                                currentDirection = Direction.UP;
                            }   
                            else if (!serviceQueueMax.isEmpty()){
                                if(serviceQueueMax.peek() > currentFloor){
                                    targetFloor = serviceQueueMax.peek();
                                    currentDirection = Direction.DOWN;
                                    newState = State.STATE_UP;
                                }
                                else {
                                    targetFloor = serviceQueueMax.peek();
                                    currentDirection = Direction.DOWN;
                                    newState = State.STATE_DOWN;
                                }
                            }
                            else{
                                targetFloor = currentFloor;
                                newState = State.STATE_STOP;
                            }

                            nHallway = 0;
                            hallway = Hallway.NONE;
                            for (Hallway h : Hallway.replicationValues) {
                                indexCarCall = ReplicationComputer.computeReplicationId(targetFloor, h);
                                if (mCarCall.get(indexCarCall).getValue()) {
                                    //					System.out.println("carcall");
                                    nHallway++;
                                    if (nHallway >= 2)
                                        hallway = Hallway.BOTH;
                                    else
                                        hallway = h;
                                }
                                if (currentDirection != Direction.STOP) {
                                    indexHallCall = ReplicationComputer.computeReplicationId(targetFloor, h, currentDirection);
                                    if (mHallCall.get(indexHallCall).getValue() && !mCarCall.get(indexCarCall).getValue()) {
                                        //					System.out.println("hallcall: "+ h + "currentDirection: " + currentDirection);
                                        nHallway++;
                                        if (nHallway >= 2)
                                            hallway = Hallway.BOTH;
                                        else
                                            hallway = h;
                                    }
                                }
                            }
                            // make sure that the last is false as well
                            // #transition 'T11.9'
                            if (!(mDoorClosedFrontLeft.getValue() && 
                                        mDoorClosedFrontRight.getValue() && 
                                        mDoorClosedBackLeft.getValue() && 
                                        mDoorClosedBackRight.getValue())) {

                                if (!isAtFloor) {
                                    newState = State.STATE_RESET;
                                }
                            }
                            break;      
                            case STATE_DOWN:
                            // state actions for state S11.1 'SET TARGET'
                            mDesiredFloor.set(targetFloor, hallway, Direction.DOWN);
                            if (hallway == Hallway.FRONT)
                                mDesiredDwellFront.setDwell(dwell);
                            else if (hallway == Hallway.BACK)
                                mDesiredDwellBack.setDwell(dwell);
                            else if (hallway == Hallway.BOTH) {
                                mDesiredDwellFront.setDwell(dwell);
                                mDesiredDwellBack.setDwell(dwell);
                            }


                            for (int i = currentFloor; i > 0; i--) {
                                int floor = i;

                                if (currentSpeed <= 0.25)
                                    commitPointDown = 5000*(floor - 1);
                                else
                                    commitPointDown = (int)(((5*(floor - 1)) + ((currentSpeed * currentSpeed)/(2*acc))) * 1000) + 200;

                                for (Hallway h : Hallway.replicationValues) {
                                    indexHallCall = ReplicationComputer.computeReplicationId(floor, h, Direction.DOWN);
                                    indexCarCall = ReplicationComputer.computeReplicationId(floor, h);
                                    if ((mCarCall.get(indexCarCall).getValue() || mHallCall.get(indexHallCall).getValue()) && position >= commitPointDown) {
                                        if (!serviceQueueMax.contains(floor))
                                            serviceQueueMax.add(floor);
                                        currentDirection = Direction.DOWN;
                                    }		
                                }
                            }	


                            if (serviceQueueMax.isEmpty()){
                                for (int i = currentFloor; i > 0; i--) {
                                    int floor = i;

                                    if (currentSpeed <= 0.25)
                                        commitPointDown = 5000*(floor - 1);
                                    else
                                        commitPointDown = (int)(((5*(floor - 1)) + ((currentSpeed * currentSpeed)/(2*acc))) * 1000) + 200;

                                    for (Hallway h : Hallway.replicationValues) {
                                        indexHallCall = ReplicationComputer.computeReplicationId(floor, h, Direction.UP);
                                        if (mHallCall.get(indexHallCall).getValue() && position >= commitPointDown) {        			
                                            if (!serviceQueueMin.contains(floor))
                                                serviceQueueMin.add(floor);
                                            currentDirection = Direction.UP;
                                        }
                                    }
                                }
                            }

                            parentState = State.STATE_NONE;
                            if (currentFloor == targetFloor){
                                if(hallway == Hallway.BOTH){
                                    if (isAtFloor) {
                                        /*
                                           if (!mDoorClosedFrontLeft.getValue() && 
                                           !mDoorClosedFrontRight.getValue() && 
                                           !mDoorClosedBackLeft.getValue() && 
                                           !mDoorClosedBackRight.getValue()) {
                                           */
                                        if (!serviceQueueMax.isEmpty()) {
                                            if (serviceQueueMax.peek() == currentFloor) {
                                                currentDirection = Direction.STOP;
                                                serviceQueueMax.poll();
                                                parentState = State.STATE_DOWN;
                                            }
                                        }
                                        else if (!serviceQueueMin.isEmpty()) {
                                            if (serviceQueueMin.peek() == currentFloor) {
                                                currentDirection = Direction.UP;
                                                serviceQueueMin.poll();
                                                parentState = State.STATE_UP;
                                            }

                                        }

                                    }
                                    }

                                    else if(hallway == Hallway.FRONT){
                                        if (isAtFloor) {
                                            /*
                                               if (!mDoorClosedFrontLeft.getValue() && 
                                               !mDoorClosedFrontRight.getValue()) {
                                               */
                                            if (!serviceQueueMax.isEmpty()) {
                                                if (serviceQueueMax.peek() == currentFloor) {
                                                    currentDirection = Direction.STOP;
                                                    serviceQueueMax.poll();
                                                    parentState = State.STATE_DOWN;
                                                }
                                            }
                                            else if (!serviceQueueMin.isEmpty()) {
                                                if (serviceQueueMin.peek() == currentFloor) {
                                                    currentDirection = Direction.UP;
                                                    serviceQueueMin.poll();
                                                    parentState = State.STATE_UP;
                                                }

                                            }

                                        }
                                        }

                                        else if(hallway == Hallway.BACK){
                                            if (isAtFloor) {
                                                /*
                                                   if (!mDoorClosedBackLeft.getValue() && 
                                                   !mDoorClosedBackRight.getValue()) {
                                                   */
                                                if (!serviceQueueMax.isEmpty()) {
                                                    if (serviceQueueMax.peek() == currentFloor) {
                                                        currentDirection = Direction.STOP;
                                                        serviceQueueMax.poll();
                                                        parentState = State.STATE_DOWN;
                                                    }
                                                }
                                                else if (!serviceQueueMin.isEmpty()) {
                                                    if (serviceQueueMin.peek() == currentFloor) {
                                                        currentDirection = Direction.UP;
                                                        serviceQueueMin.poll();
                                                        parentState = State.STATE_UP;
                                                    }

                                                }

                                            }
                                            }
                                        }

                                        if (parentState != State.STATE_NONE) {
                                            newState = State.STATE_ATFLOOR;
                                            if (!serviceQueueMax.isEmpty()) {
                                                currentDirection = Direction.DOWN;
                                            }
                                        }
                                        else if (!serviceQueueMax.isEmpty()) {
                                            targetFloor = serviceQueueMax.peek();
                                            currentDirection = Direction.DOWN;
                                        }   
                                        else if (!serviceQueueMin.isEmpty()){
                                            if(serviceQueueMin.peek() < currentFloor){
                                                targetFloor = serviceQueueMin.peek();
                                                currentDirection = Direction.UP;
                                                newState = State.STATE_DOWN;
                                            }
                                            else {
                                                targetFloor = serviceQueueMin.peek();
                                                currentDirection = Direction.UP;
                                                newState = State.STATE_UP;
                                            }
                                        }
                                        else {
                                            targetFloor = currentFloor;
                                            newState = State.STATE_STOP;
                                        }

                                        nHallway = 0;
                                        hallway = Hallway.NONE;
                                        for (Hallway h : Hallway.replicationValues) {
                                            indexCarCall = ReplicationComputer.computeReplicationId(targetFloor, h);
                                            if (mCarCall.get(indexCarCall).getValue()) {
                                                nHallway++;
                                                if (nHallway >= 2)
                                                    hallway = Hallway.BOTH;
                                                else
                                                    hallway = h;
                                            }
                                            if (currentDirection != Direction.STOP) {
                                                indexHallCall = ReplicationComputer.computeReplicationId(targetFloor, h, currentDirection);
                                                if (mHallCall.get(indexHallCall).getValue() && !mCarCall.get(indexCarCall).getValue()) {
                                                    nHallway++;
                                                    if (nHallway >= 2)
                                                        hallway = Hallway.BOTH;
                                                    else
                                                        hallway = h;
                                                }
                                            }
                                        }

                                        // all doors are closed
                                        if (!(mDoorClosedFrontLeft.getValue() && 
                                                    mDoorClosedFrontRight.getValue() && 
                                                    mDoorClosedBackLeft.getValue() && 
                                                    mDoorClosedBackRight.getValue())) {

                                            // make sure that the last is false as well
                                            // #transition 'T11.9'
                                            if (!isAtFloor) {
                                                newState = State.STATE_RESET;
                                            }
                                        }
                                        break;         


                                        case STATE_ATFLOOR:

                                        mDesiredFloor.set(targetFloor, hallway, currentDirection);

                                        if (hallway == Hallway.BOTH){
                                            if(!mDoorClosedFrontLeft.getValue() && 
                                                    !mDoorClosedFrontRight.getValue() && 
                                                    !mDoorClosedBackLeft.getValue() && 
                                                    !mDoorClosedBackRight.getValue()){

                                                newState = parentState;

                                            }
                                        }

                                        else if (hallway == Hallway.FRONT){
                                            if(!mDoorClosedFrontLeft.getValue() && 
                                                    !mDoorClosedFrontRight.getValue()){
                                                newState = parentState;
                                            }
                                        }

                                        else if (hallway == Hallway.BACK){
                                            if(!mDoorClosedBackLeft.getValue() && 
                                                    !mDoorClosedBackRight.getValue()){
                                                newState = parentState;
                                            }
                                        }
                                        else {
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
