package collisionDetection;


import java.text.DecimalFormat;
import java.util.concurrent.TimeUnit;
import java.util.Calendar;
import java.util.Date;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.motionModel.directServo.DirectServo;
import com.kuka.connectivity.motionModel.directServo.IDirectServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.sensorModel.DataRecorder;
import com.kuka.roboticsAPI.sensorModel.ForceSensorData;
import com.kuka.roboticsAPI.sensorModel.DataRecorder.AngleUnit;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

/**
 
 * This program records sensory data (pose, torque ...)
 * To be used as input for AI-based collision detection algorithms
 * 
 * Scenario : Robot's tool moves around a circular path while applying (constant)force on the surface
 * 
 * 
Author : Siavash Esteki
*/

public class DataAquisition extends RoboticsAPIApplication {
	@Inject
	private LBR lbr14;
	
	private static final int stiffnessZ = 1000;//500
	private static final int stiffnessXY = 5000;
	double initialPosition[] = {0,700, 60};
	double initOrientation[] = {179,0,180};
	double desiredCartesianPose[] ={0,0,0};
	
	
	@Inject
	@Named("Cylinder")
	private Tool tool;//page 371 pdf
	
	@Inject
	private MediaFlangeIOGroup mediaflange;
	
	
	@Override
	public void initialize() {
		// initialize your application here
		lbr14 = getContext().getDeviceFromType(LBR.class);
		tool.attachTo(lbr14.getFlange());
	}

	public Frame moveToInitialPosition()
    {
		
    Frame desiredFrame0  = lbr14.getFlange().copyWithRedundancy();
    desiredFrame0.setX(initialPosition[0]);
    desiredFrame0.setY(initialPosition[1]);
    desiredFrame0.setZ(initialPosition[2]);
    
    desiredFrame0.setAlphaRad(initOrientation[0]*3.14/180);
    desiredFrame0.setBetaRad(initOrientation[1]*3.14/180);
    desiredFrame0.setGammaRad(initOrientation[2]*3.14/180);
    
    getLogger().info(tool.getDefaultMotionFrame().toString()); 
    
    	try{
    		tool.move(ptp(desiredFrame0).setJointVelocityRel(0.3));
    	}
    	catch (Exception e) {
    		getLogger().error(e.getMessage());  
		}
    	return desiredFrame0;
    	
    }
	
	
	@SuppressWarnings("deprecation")
	@Override
	public void run() {
		// your application execution starts here
		DecimalFormat df = new DecimalFormat("0.000000");
		double desiredForce = 0;
		Frame initialPoseOfTool = moveToInitialPosition();
		///-----------Impedance Controller	
				CartesianImpedanceControlMode cartImpCtrlMode;
		        cartImpCtrlMode = new CartesianImpedanceControlMode();
		        
		        cartImpCtrlMode.parametrize(CartDOF.X,
		        CartDOF.Y).setStiffness(stiffnessXY);
		        cartImpCtrlMode.parametrize(CartDOF.Z).setStiffness(stiffnessZ);
		        cartImpCtrlMode.parametrize(CartDOF.ROT).setStiffness(300.0);
		        cartImpCtrlMode.parametrize(CartDOF.ALL).setDamping(0.2);
		//----------end
		///-------Direct Servo Motion
		        DirectServo aDirectServoMotion = new DirectServo(
		                lbr14.getCurrentJointPosition());
		        aDirectServoMotion.setMinimumTrajectoryExecutionTime(40e-3);
		        ServoMotion.validateForImpedanceMode(tool);
		        aDirectServoMotion.setJointVelocityRel(0.4);
		        //aDirectServoMotion
		        getLogger().info("Starting DirectServo motion in position control mode");
		        tool.getDefaultMotionFrame().moveAsync(aDirectServoMotion.setMode(cartImpCtrlMode));// binds 'move' method to our DirectServo motion
		        getLogger().info("Get the runtime of the DirectServo motion");
		        IDirectServoRuntime theServoRuntime = aDirectServoMotion
		                .getRuntime();
		        
		///-------
		//measuring bias force in kuka sensor
		ForceSensorData initialExtForceTorques = lbr14.getExternalForceTorque(lbr14.getFlange(),World.Current.getRootFrame());
		
		getLogger().info("Bias forces: " + initialExtForceTorques.toString());
		
		Frame desiredFrame = lbr14.getFlange().copyWithRedundancy();// initialPoseOfTool.copyWithRedundancy();
        
		desiredFrame.setX(initialPoseOfTool.getX());
        desiredFrame.setY(initialPoseOfTool.getY());
        desiredFrame.setZ(initialPoseOfTool.getZ());
        desiredFrame.setAlphaRad(initOrientation[0]*3.14/180);
        desiredFrame.setBetaRad(initOrientation[1]*3.14/180);
        desiredFrame.setGammaRad(initOrientation[2]*3.14/180);
        
        
        Calendar myCalendar = Calendar.getInstance();
        String dateTime = "6May2023"+
        		"_" + String.valueOf(Calendar.HOUR_OF_DAY)+ String.valueOf(myCalendar.get(Calendar.MINUTE) ) + String.valueOf(myCalendar.get(Calendar.SECOND) ) ;

        		//String.valueOf(myCalendar.get(Calendar.YEAR) )+String.valueOf(myCalendar.get(Calendar.MONTH) )+ String.valueOf(myCalendar.get(Calendar.DAY_OF_MONTH) )+

        String collisionType="NoColl";
        int option = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose the type of collisions that occur during the execution:", "No collision" ,"Intentional","Accidental");
        if (option == 0){
        	collisionType="NoColl";
        }
        else if (option == 1){
        	collisionType="Intentional";
        }
        else {
        	collisionType = "Accidntal";
        }
        
        int forceval = getApplicationUI().displayModalDialog(ApplicationDialogType.QUESTION, "Choose the force to be applied in Z direction", "0 N" ,"-10 N","-15 N");
        if (forceval == 0){
        	desiredForce = 0;
        }
        else if (forceval == 1){
        	desiredForce = -10;
        }
        else {
        	desiredForce = -15;
        }
        Date dd = new Date();
        
        getLogger().info( String.valueOf( myCalendar.get(Calendar.DAY_OF_MONTH) ));
        
        getLogger().info("collDetect_DataAq_f" + String.valueOf( Math.abs(desiredForce) )+ 
        		"N_CollType"+ collisionType + dateTime +".log" );
        
        DataRecorder rec = new DataRecorder("collDetect_DataAq_f" + String.valueOf( df.format(Math.abs(desiredForce)) )+ 
        		"N_CollType"+ collisionType + dateTime +".log", -1,
				TimeUnit.SECONDS, 1);
		//rec.setFileName("sampleCartesianImp");
		rec.addCurrentJointPosition(lbr14, AngleUnit.Degree);// joint position
		rec.addExternalJointTorque(lbr14);						// Joint torque 
		//rec.addCurrentCartesianPositionXYZ(lbr14.getFlange(),
		//		lbr14.getRootFrame());
		rec.addCurrentCartesianPositionXYZ(tool.getDefaultMotionFrame(),
				lbr14.getRootFrame());  						// Cartesian Position
		rec.addCartesianForce(tool.getDefaultMotionFrame(),
				lbr14.getRootFrame());							// Cartesian Force
		
		rec.addCartesianTorque(tool.getDefaultMotionFrame(),
				lbr14.getRootFrame());							// Cartesian Torque
		
		rec.addCommandedCartesianPositionXYZ(tool.getDefaultMotionFrame(),
				lbr14.getRootFrame());							// Commanded Cartesian Position
		
		
        // Path specs
        final double TOTAL_CIRCULAR_PATH_TIME=12.0; //In seconds
        final double FREQENCY = 1.0/TOTAL_CIRCULAR_PATH_TIME;
        final double AMPLITUDE = 150;// Circular path Radius (mm). MAX =150. 
        final double TOTAL_LOOPTIME = 60;
		mediaflange.setLEDBlue(true);

        try
        {
     
            double omega = FREQENCY * 2 * Math.PI * 1e-9;//In nanoseconds
            long startTimeStamp = System.nanoTime(); 
            double curTime = System.nanoTime() - startTimeStamp;
            
            rec.enable();
			rec.startRecording();
			
            while ((curTime/1e9) < TOTAL_LOOPTIME)
            {

                // Synchronize with the realtime system
            	theServoRuntime.updateWithRealtimeSystem();
                // Get the measured position in Cartesian...
            	Frame currentCartesanPosition =  theServoRuntime.getCurrentCartesianPosition(tool.getDefaultMotionFrame());//we can have TCP coordinate using only this method
                
                // do a cyclic loop
                curTime = System.nanoTime() - startTimeStamp;
                
                double sinArgument = omega * curTime;

                
                // compute a new commanded position
                double offsetX = AMPLITUDE * Math.sin(sinArgument);
                double offsetY = AMPLITUDE * (Math.cos(sinArgument)-1.0);
                double offsetZ =  (desiredForce ) / stiffnessZ *1000 ;
                
                desiredFrame.setX(initialPoseOfTool.getX() + offsetX);
                desiredFrame.setY(initialPoseOfTool.getY() + offsetY);
                desiredFrame.setZ(currentCartesanPosition.getZ() + offsetZ);     
                try{
        			theServoRuntime.setDestination(desiredFrame); 			
        		}
        		catch (Exception e) {
        			getLogger().error("Out of reach destination");
        			mediaflange.setLEDBlue(false);

        		}

            }
            rec.stopRecording();
    		ForceSensorData force = lbr14.getExternalForceTorque(lbr14.getFlange(),World.Current.getRootFrame());
    		getLogger().info(force.toString());
	        getLogger().info("Finish");
			mediaflange.setLEDBlue(false);


            
        }
        catch (Exception e)
        {
            getLogger().info(e.getLocalizedMessage());
            e.printStackTrace();
        }
        
        
	}
}