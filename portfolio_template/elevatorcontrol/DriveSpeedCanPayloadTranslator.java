/* 
 * Course and Semester : 18-649 Fall 2013
 * Group No: 16
 * Group Members : Jiangtian Nie(jnie) , Yue Chen(yuechen),
 *                 Sally Stevenson(ststeven) , Sri Harsha Koppaka(skoppaka)
 */

package simulator.elevatorcontrol;

import java.util.BitSet;

import simulator.framework.Direction;
import simulator.framework.Speed;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

public class DriveSpeedCanPayloadTranslator extends CanPayloadTranslator {

    public DriveSpeedCanPayloadTranslator(WriteableCanMailbox p) {
        super(p, 2, MessageDictionary.DRIVE_SPEED_CAN_ID);
    }
    
    public DriveSpeedCanPayloadTranslator(ReadableCanMailbox p) {
        super(p, 2, MessageDictionary.DRIVE_SPEED_CAN_ID);
    }

    /**
     * This method is required for setting values by reflection in the
     * MessageInjector.  The order of parameters in .mf files should match the
     * signature of this method.
     * All translators must have a set() method with the signature that contains
     * all the parameter values.
     *
     * @param speed
     * @param dir
     */
    public void set(Speed speed, Direction dir) {
        setSpeed(speed);
        setDirection(dir);
    }
    
    public void setSpeed(Speed speed) {
        BitSet b = getMessagePayload();
        addUnsignedIntToBitset(b, speed.ordinal(), 0, 9);
        setMessagePayload(b, getByteSize());
    }

    public Speed getSpeed() {
        int val = getUnsignedIntFromBitset(getMessagePayload(), 0, 9);
        for (Speed s : Speed.values()) {
            if (s.ordinal() == val) {
                return s;
            }
        }
        throw new RuntimeException("Unrecognized Speed Value " + val);
    }
    
    public void setDirection(Direction dir) {
        BitSet b = getMessagePayload();
        addUnsignedIntToBitset(b, dir.ordinal(), 9, 2);
        setMessagePayload(b, getByteSize());
    }

    public Direction getDirection() {
        int val = getUnsignedIntFromBitset(getMessagePayload(), 9, 2);
        for (Direction d : Direction.values()) {
            if (d.ordinal() == val) {
                return d;
            }
        }
        throw new RuntimeException("Unrecognized Direction Value " + val);
    }
    
    @Override
    public String payloadToString() {
        return "DriveCommand:  speed=" + getSpeed() + " direction=" + getDirection();
    }
}
