// Copyright 2019 Nina Marie Wahl og Charlotte Heggem.
// Copyright 2019 Norwegian University of Science and Technology.
// Modifications copyright (C) 2020 Morten Melby Dahl
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


package API_ROS2_Sunrise;


import API_ROS2_Sunrise.KMPjogger;

// Imported classes to control thread priorities.
import API_ROS2_Sunrise.KMP_sensor_reader;
import API_ROS2_Sunrise.KMP_status_reader;

import com.kuka.jogging.provider.api.common.ICartesianJoggingSupport;
import com.kuka.roboticsAPI.deviceModel.kmp.KmpOmniMove;
import com.kuka.roboticsAPI.executionModel.ICommandContainer;
import com.kuka.roboticsAPI.motionModel.kmp.MobilePlatformRelativeMotion;


public class KMP_commander extends Node{

	// Robot
	KmpOmniMove kmp;
	
	// Motion variables: KMP
	ICommandContainer KMP_currentMotion;
	double[] velocities = {0.0,0.0,0.0};
	KMPjogger kmp_jogger;
	long jogging_period  = 1L;

	// Implemented node classes
	KMP_sensor_reader kmp_sensor_reader;
	KMP_status_reader kmp_status_reader;

	public KMP_commander(int port, KmpOmniMove robot, String ConnectionType) {
		super(port,ConnectionType, "KMP commander");
		this.kmp = robot;
		this.kmp_jogger = new KMPjogger((ICartesianJoggingSupport)kmp, jogging_period);
		
		
		if (!(isSocketConnected())) {
			Thread monitorKMPCommandConnections = new MonitorKMPCommandConnectionsThread();
			monitorKMPCommandConnections.start();
			}else {
				setisKMPConnected(true);
		}
	}

	@Override
	public void run() {
		Thread emergencyStopThread = new MonitorEmergencyStopThread();
		emergencyStopThread.start();
		
		while(isNodeRunning())
		{
			String Commandstr = this.socket.receive_message(); 
	    	String []splt = Commandstr.split(" ");
	    	if( !getShutdown() && !closed){
		    	if ((splt[0]).equals("shutdown")){
		    		System.out.println("KMP received shutdown");
					setShutdown(true);	
					break;
					}
		
		    	if ((splt[0]).equals("setTwist") && !getEmergencyStop()){
					// Sets max priority if a twist is received.
					if(KMP_commander.getPriority() != Thread.MAX_PRIORITY){
						KMP_commander.setPriority(Thread.MAX_PRIORITY);
						kmp_sensor_reader.setPriority(Thread.MAX_PRIORITY);
						kmp_status_reader.setPriority(Thread.MAX_PRIORITY);
					}
					setNewVelocity(Commandstr);
					}

		    	if ((splt[0]).equals("setPose") && !getEmergencyStop()){
					// Sets max priority if a pose is received.
					if(KMP_commander.getPriority() != Thread.MAX_PRIORITY){
						KMP_commander.setPriority(Thread.MAX_PRIORITY);
						kmp_sensor_reader.setPriority(Thread.MAX_PRIORITY);
						kmp_status_reader.setPriority(Thread.MAX_PRIORITY);
					}
					setNewPose(Commandstr);
					}
	    	}
		}System.out.println("KMPcommander no longer running.");
    }
	
	public class MonitorEmergencyStopThread extends Thread {
		public void run(){
			// Sets emergency stop monitoring thread to max priority.
			MonitorEmergencyStopThread.setPriority(Thread.MAX_PRIORITY);
			while(isNodeRunning()) {
				if (getEmergencyStop() && getisKMPMoving()){
					  setisKMPMoving(false);
					  kmp_jogger.killJoggingExecution(getisKMPMoving());
					  System.out.println("Successfully killed jogging execution from emergency stop.");
					  if(!(KMP_currentMotion==null)){
						  KMP_currentMotion.cancel();
					  }
				}
				}
			}
		}
	
	public void setNewPose(String data){
		String []lineSplt = data.split(" ");
		if (lineSplt.length==4){
			double pose_x = Double.parseDouble(lineSplt[1]);
			double pose_y = Double.parseDouble(lineSplt[2]);
			double pose_theta = Double.parseDouble(lineSplt[3]);

			MobilePlatformRelativeMotion MRM = new MobilePlatformRelativeMotion(pose_x, pose_y, pose_theta);

			MRM.setVelocity(300, 10);
			MRM.setTimeout(100);
			MRM.setAcceleration(10, 10);
			
			if(kmp.isReadyToMove()) {
				System.out.println("Moving!");
				this.KMP_currentMotion =  kmp.moveAsync(MRM);
			}

			else {
				System.out.println("KMP is not ready to move!");
			}

		}else{
			System.out.println("Unacceptable Mobile Platform Relative Velocity command!");

		}
	}

	
	public void setNewVelocity(String vel){
		String []lineSplt = vel.split(" ");

		if(lineSplt.length==4){
				this.velocities[0] = Double.parseDouble(lineSplt[1]); // x
				this.velocities[1] = Double.parseDouble(lineSplt[2]); // y
				this.velocities[2] = Double.parseDouble(lineSplt[3]); // theta
				
				if(velocities[0] == 0 && velocities[1] == 0 && velocities[2] ==0) {
					  if(getisKMPMoving()) {
						  setisKMPMoving(false);

						  // If the velocity is set to 0, KMP threads gets lowest priority.
						  KMP_commander.setPriority(Thread.MIN_PRIORITY);
						  kmp_sensor_reader.setPriority(Thread.MIN_PRIORITY);
						  kmp_status_reader.setPriority(Thread.MIN_PRIORITY);

						  this.kmp_jogger.killJoggingExecution(true);
						  }
				}
				else{
					  if(getisKMPMoving()&& !getEmergencyStop()) {
						  this.kmp_jogger.updateVelocities(this.velocities);
					  }
					  else if(!getisKMPMoving() && !getEmergencyStop() && kmp.isReadyToMove()){
						  setisKMPMoving(true);
						  this.kmp_jogger.updateVelocities(this.velocities);
						  this.kmp_jogger.startJoggingExecution();
					  }
					  else{System.out.println("Cannot jog robot, isReadyToMove() is: " +kmp.isReadyToMove() + " calculated value: " + KmpOmniMove.calculateReadyToMove(kmp.getSafetyState()));
					  }
				  }
				}
		}
	
	public class MonitorKMPCommandConnectionsThread extends Thread {
		int timeout = 3000;
		public void run(){
			while(!(isSocketConnected()) && (!(closed))) {
				if(getisLBRConnected()) {
					timeout = 5000;
				}
				createSocket();
				if (isSocketConnected()){
					setisKMPConnected(true);
					break;
				}
				try {
					Thread.sleep(timeout);
				} catch (InterruptedException e) {
					System.out.println(node_name + " connection thread could not sleep");
				}
			}
			if(!closed){
				System.out.println("Connection with KMP Command Node OK!");
				runmainthread();
				}	
		}
	}


	@Override
	public void close() {
		closed = true;
		try{
			 this.kmp_jogger.killJoggingExecution(getisKMPMoving());
			 System.out.println("KMPJogger ended successfully");
		}catch(Exception e){
			System.out.println("Could not kill jogging execution.");
		}
		try{
			this.socket.close();
		}catch(Exception e){
				System.out.println("Could not close KMP commander connection: " +e);
			}
		System.out.println("KMP commander closed!");
 	}
	
}