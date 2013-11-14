/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package simulator.elevatorcontrol;

import jSimPack.SimTime;
import jSimPack.SimTime.SimTimeUnit;
import simulator.framework.Harness;
import simulator.framework.RuntimeMonitor;
import simulator.framework.Speed;
import simulator.payloads.DrivePayload.ReadableDrivePayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;

/**
 * This example monitor shows how to use the RuntimeMonitor hooks to check for
 * fast speed between floors.  You will need to implement additional checks
 * to fulfill the project 12 requirements.
 *
 * See the documentation of simulator.framework.RuntimeMonitor for more details.
 *
 * @author Justin Ray
 */
public class RTSpeed extends RuntimeMonitor {

    protected int currentFloor = MessageDictionary.NONE;
    protected int lastStoppedFloor = MessageDictionary.NONE;
    protected boolean fastSpeedReached = false;
    private Speed command=null;
    private double currentSpeed;
    private SimTime beginTime;
    private SimTime endTime;

    public RTSpeed() {
        //initialization goes here
    }

    public void timerExpired(Object callbackData) {
        //implement time-sensitive behaviors here
    }

    
    
    //my part from here
    @Override
    public void receive(ReadableDriveSpeedPayload msg) {
        getCurrentSpeed(msg);
        checkDelayFast();
        checkSlowEarly();
        
    }
    
    @Override
    public void receive(ReadableDrivePayload msg) {
        getSpeed(msg);
    }

    
    
    private void getSpeed(ReadableDrivePayload msg){
    	command=msg.speed();
    }
    
    private void getCurrentSpeed(ReadableDriveSpeedPayload msg){
    	currentSpeed=msg.speed();
    }
    
    //check whether the car get to fast in time.
    private void checkDelayFast(){
    	if (command==Speed.SLOW&&currentSpeed==0.01){
    		beginTime=Harness.getTime();
    	}

    	
    	if(beginTime!=null){
    		if (Harness.getTime()==SimTime.add(beginTime,new SimTime(300,SimTimeUnit.MILLISECOND))){
    			if (currentSpeed<=0.25){
    				message("RT-9 violated: The Drive shall be commanded to fast speed to the maximum degree practicable.(Delay to be Fast)");
    			}
    			beginTime=null;	
    		}
    	}
    }
    
    //check whether the car get to slow too soon.
    private void checkSlowEarly(){
    	if ((currentSpeed==(0.25+0.01))&&command==Speed.SLOW){
    		endTime=Harness.getTime();
    	}
    	
    	if(endTime!=null){
    		if (command!=Speed.SLOW){
    			SimTime offset=SimTime.subtract(Harness.getTime(), endTime);
    			if (offset.isGreaterThan(new SimTime(300,SimTimeUnit.MILLISECOND))){
    				message("RT-9 violated: The Drive shall be commanded to fast speed to the maximum degree practicable."
    						+ "(Too early to escape from fast)");
    			}
    			endTime=null;
    		}
    	}
    }
    
    
    
    
    
    //my part end here.
    
    


    @Override
    protected String[] summarize() {
        return new String[0]; //nothing to summarize
    }




}
