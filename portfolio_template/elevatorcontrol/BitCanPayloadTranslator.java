/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package simulator.elevatorcontrol;

import java.util.BitSet;

import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

/**
 * Translates a single boolean value into a 1-byte payload.
 * 
 */

public class BitCanPayloadTranslator extends CanPayloadTranslator {

    
    /**
     * Constructor for use with WriteableCanMailbox objects
     * @param payload
     */
    public BitCanPayloadTranslator(WriteableCanMailbox payload) {
        super(payload, 1);
    }

    /**
     * Constructor for use with ReadableCanMailbox objects
     * @param payload
     */

    public BitCanPayloadTranslator(ReadableCanMailbox payload) {
        super(payload, 1);
    }
    
    //required for reflection
    public void set(boolean value) {
        setValue(value);
    }

    
    public void setValue(boolean value) {       
        BitSet b = new BitSet(1);
        b.set(1, value);
        setMessagePayload(b, getByteSize());
    }
    
    public boolean getValue() {
        return getMessagePayload().get(0);
    }
    
    @Override
    public String payloadToString() {
        return Boolean.toString(getValue());
    }
}
