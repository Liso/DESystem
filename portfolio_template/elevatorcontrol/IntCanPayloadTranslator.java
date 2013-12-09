/* 
 * Course and Semester : 18-649 Fall 2013
 * Group No: 16
 * Group Members : Jiangtian Nie(jnie) , Yue Chen(yuechen),
 *                 Sally Stevenson(ststeven) , Sri Harsha Koppaka(skoppaka)
 */

package simulator.elevatorcontrol;

import java.util.BitSet;

import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

/**
 * This class takes a single integer or boolean value and translates it to a
 * 4 byte CanMailbox
 * @author justinr2
 */
public class IntCanPayloadTranslator extends CanPayloadTranslator {

    /**
     * Constructor for use with WriteableCanMailbox objects
     * @param payload
     */
    public IntCanPayloadTranslator(WriteableCanMailbox payload) {
        super(payload, 2);
    }

    /**
     * Constructor for use with ReadableCanMailbox objects
     * @param payload
     */

    public IntCanPayloadTranslator(ReadableCanMailbox payload) {
        super(payload, 2);
    }

    
    //required for reflection
    public void set(int value) {
        setValue(value);
    }
    
    public void setValue(int value) {
        BitSet b = new BitSet();
        addUnsignedIntToBitset(b, value, 0, 16);
        setMessagePayload(b, getByteSize());
    }
    
    public int getValue() {
        return getUnsignedIntFromBitset(getMessagePayload(), 0, 16);
    }
    
    @Override
    public String payloadToString() {
        return "0x" + Integer.toString(getValue(),16);
    }
}
