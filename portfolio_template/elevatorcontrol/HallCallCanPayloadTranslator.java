/*
 * 18649 - Fall 2013
 * Group 20 - Fiona Britto (fbritto), Shubhang Chaudhary (shubhanc), Mitali Naik (mnaik), and Adwaith Venkataraman (adwaithv) 
 */

package simulator.elevatorcontrol;

import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.BooleanCanPayloadTranslator;


public class HallCallCanPayloadTranslator extends BooleanCanPayloadTranslator {
    
    /**
     * Constructor for use with WriteableCanMailbox objects
     * @param payload
     */
    public HallCallCanPayloadTranslator(WriteableCanMailbox payload) {
        super(payload);
    }

    /**
     * Constructor for use with ReadableCanMailbox objects
     * @param payload
     */

    public HallCallCanPayloadTranslator(ReadableCanMailbox payload) {
        super(payload);
    }
    
    //set, setValue, getValue, payloadToString methods inherited
}