/* 
 * Course and Semester : 18-649 Fall 2013
 * Group No: 16
 * Group Members : Jiangtian Nie(jnie) , Yue Chen(yuechen),
 *                 Sally Stevenson(ststeven) , Sri Harsha Koppaka(skoppaka)
 */


/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package simulator.elevatorcontrol;

import java.util.BitSet;
import java.util.HashMap;

import simulator.payloads.CANNetwork;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.Harness;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;

/**
 * This class provides some example utility classes that might be useful in more
 * than one spot.  It is okay to create new classes (or modify the ones given
 * below), but you may not use utility classes in such a way that they constitute
 * a communication channel between controllers.
 *
 * @author justinr2
 */
public class Utility {

	public static class DoorClosedArray {

		HashMap<Integer, BitCanPayloadTranslator> translatorArray = new HashMap<Integer, BitCanPayloadTranslator>();
		public final Hallway hallway;

		public DoorClosedArray(Hallway hallway, CANNetwork.CanConnection conn) {
			this.hallway = hallway;
			for (Side s : Side.values()) {
				int index = ReplicationComputer.computeReplicationId(hallway, s);
				ReadableCanMailbox m = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + index);
				BitCanPayloadTranslator t = new BitCanPayloadTranslator(m);
				conn.registerTimeTriggered(m);
				translatorArray.put(index, t);
			}
		}

		public boolean getBothClosed() {
			return translatorArray.get(ReplicationComputer.computeReplicationId(hallway, Side.LEFT)).getValue() &&
					translatorArray.get(ReplicationComputer.computeReplicationId(hallway, Side.RIGHT)).getValue();
		}
	}

	public static class AtFloorArray {

		public HashMap<Integer, BitCanPayloadTranslator> networkAtFloorsTranslators = new HashMap<Integer, BitCanPayloadTranslator>();
		public final int numFloors = Elevator.numFloors;

		public AtFloorArray(CANNetwork.CanConnection conn) {
			for (int i = 0; i < numFloors; i++) {
				int floor = i + 1;
				for (Hallway h : Hallway.replicationValues) {
					int index = ReplicationComputer.computeReplicationId(floor, h);
					ReadableCanMailbox m = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + index);
					BitCanPayloadTranslator t = new BitCanPayloadTranslator(m);
					conn.registerTimeTriggered(m);
					networkAtFloorsTranslators.put(index, t);
				}
			}
		}

		public boolean isAtFloor(int floor, Hallway hallway) {
			return networkAtFloorsTranslators.get(ReplicationComputer.computeReplicationId(floor, hallway)).getValue();
		}

		public int getCurrentFloor() {
			int retval = MessageDictionary.NONE;
			for (int i = 0; i < numFloors; i++) {
				int floor = i + 1;
				for (Hallway h : Hallway.replicationValues) {
					int index = ReplicationComputer.computeReplicationId(floor, h);
					BitCanPayloadTranslator t = networkAtFloorsTranslators.get(index);
					if (t.getValue()) {
						if (retval == MessageDictionary.NONE) {
							//this is the first true atFloor
							retval = floor;
						} else if (retval != floor) {
							//found a second floor that is different from the first one
							throw new RuntimeException("AtFloor is true for more than one floor at " + Harness.getTime());
						}
					}
				}
			}
			return retval;
		}
	}

	public static int getValueFromBitset(BitSet b, int startLocation, int bitSize)
	{
		if ((startLocation + b.size()) < bitSize)
			throw new IllegalArgumentException("bitSize is greater than the size of BitSet");
		int value = 0;
		int mask = 0x1;
		int bitOffset = startLocation;
		for (int i = 0; i < bitSize; i++)
		{
			if (b.get(bitOffset))
			{
				value = value | mask;
			}
			mask = mask << 1;
			bitOffset++;
		}
		return value;
	}

	public static void addValueToBitset(BitSet b, int value, int startLocation,
			int bitSize)
	{

		int max = (int) Math.pow(2.0, bitSize) - 1;
		if (value > max)
		{
			throw new IllegalArgumentException("Value " + value
					+ " is too large place into " + bitSize + " bits.");
		}
		if (value < 0)
		{
			throw new IllegalArgumentException("Value " + value
					+ " is too small to place into " + bitSize + " bits.");
		}
		int mask = 0x1;
		int bitOffset = startLocation;
		for (int i = 0; i < bitSize; i++)
		{
			b.set(bitOffset, (value & mask) == mask);
			mask = mask << 1;
			bitOffset++;
		}
	}


}
