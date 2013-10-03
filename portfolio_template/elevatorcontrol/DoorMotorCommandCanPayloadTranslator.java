/* 
 * Course and Semester : 18-649 Fall 2013
 * Group No: 16
 * Group Members : Jiangtian Nie(jnie) , Yue Chen(yuechen),
 *                 Sally Stevenson(ststeven) , Sri Harsha Koppaka(skoppaka)
 * Author : Yue Chen
 * AndrewID : yuechen
 */


package simulator.elevatorcontrol;

import java.util.BitSet;

import simulator.framework.DoorCommand;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

public class DoorMotorCommandCanPayloadTranslator extends CanPayloadTranslator {
    public DoorMotorCommandCanPayloadTranslator(WriteableCanMailbox p, Hallway hallway, Side side) {
        super(p, 4, MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, side));
    }
    
    public DoorMotorCommandCanPayloadTranslator(ReadableCanMailbox p, Hallway hallway, Side side) {
        super(p, 4, MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, side));
    }

    /**
     * This method is required for setting values by reflection in the
     * MessageInjector.  The order of parameters in .mf files should match the
     * signature of this method.
     * All translators must have a set() method with the signature that contains
     * all the parameter values.
     *
     * @param DoorCommand
     */
    public void set(DoorCommand doorCommand) {
        setDoorCommand(doorCommand);
    }
    
    public void setDoorCommand(DoorCommand doorCommand) {
        BitSet b = getMessagePayload();
        addIntToBitset(b, doorCommand.ordinal(), 0, 32);
        setMessagePayload(b, getByteSize());
    }

    public DoorCommand getDoorCommand() {
        int val = getIntFromBitset(getMessagePayload(), 0, 32);
        for (DoorCommand dc : DoorCommand.values()) {
            if (dc.ordinal() == val) {
                return dc;
            }
        }
        throw new RuntimeException("Unrecognized DoorCommand Value " + val);
    }

    @Override
    public String payloadToString() {
        return "DoorMotorCommand:  DoorCommand=" + getDoorCommand();
    }
}
