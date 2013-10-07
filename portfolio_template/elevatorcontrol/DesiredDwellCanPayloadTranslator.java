package simulator.elevatorcontrol;

import jSimPack.SimTime;

import java.util.BitSet;

import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

public class DesiredDwellCanPayloadTranslator extends CanPayloadTranslator {
    /**
     * Constructor for WriteableCanMailbox.  You should always implement both a 
     * Writeable and Readable constructor so the same translator can be used for
     * both objects
     * @param payload
     */
    public DesiredDwellCanPayloadTranslator(WriteableCanMailbox payload, Hallway hallway) {
        super(payload, 4, MessageDictionary.DESIRED_DWELL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway));
    }

    /**
     * Constructor for ReadableCanMailbox.  You should always implement both a 
     * Writeable and Readable constructor so the same translator can be used for
     * both objects
     * @param payload
     */
    public DesiredDwellCanPayloadTranslator(ReadableCanMailbox payload, Hallway hallway) {
        super(payload, 4, MessageDictionary.DESIRED_DWELL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway));
    }
    
    /**
     * Set the direction for mDesiredFloor in bits 32-47 of the can payload
     * @param dir
     */
    public void setDwell(SimTime dwell) {
        BitSet b = getMessagePayload();
        addIntToBitset(b, (int)dwell.getTruncSeconds(), 0, 32);
        setMessagePayload(b, getByteSize());
    }
    
    /**
     * 
     * @return the direction value from the can payload
     */
    public SimTime getDwell() {
        int val = getIntFromBitset(getMessagePayload(), 0, 32);
        SimTime dwell = new SimTime(val,
                SimTime.SimTimeUnit.SECOND);
       return dwell;
    }
    
    /**
     * Implement a printing method for the translator.
     * @return
     */
    @Override
    public String payloadToString() {
        return "DesiredDwell = " + getDwell().getFracSeconds();
    }
}
