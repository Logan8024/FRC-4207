package frc.robot.subsystems;


//import edu.wpi.first.cscore.CameraServerJNI;
//import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.cameraserver.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cscore.VideoSource;
public class HFCameraPairSwitchable2021 extends SubsystemBase {
	
	VideoSink server;
	int curCamera = 0;
	public static UsbCamera[] camera;
	int numCameras;
	public  HFCameraPairSwitchable2021(int numCameras) {
		this.numCameras = numCameras;
//		CameraServer cs = CameraServer.getInstance();
		camera = new UsbCamera[numCameras];
		
		for(int i=0;i<numCameras;i++) {
			camera[i] = CameraServer.startAutomaticCapture("cam" + i,i);
//			camera[i].setPixelFormat(PixelFormat.kMJPEG);
//			camera[i].setResolution(160, 120);
//			camera[i].setFPS(15);
			camera[i].setVideoMode(PixelFormat.kMJPEG, 160, 120, 15);
			
			camera[i].setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
		}
		server = CameraServer.addSwitchedCamera("drivercams");
		server.setSource(camera[0]);
//		camera1 = new UsbCamera("cam0",0);
 //    	camera2 = new UsbCamera("cam1",1);
/*    	CvSink cvSink1 = new CvSink("csCamera1");
    	cvSink1.setSource(camera1);
    	cvSink1.setEnabled(true);

    	CvSink cvSink2 = new CvSink("csCamera2");
    	cvSink2.setSource(camera2);
    	cvSink2.setEnabled(true);
*/
    	setCamera(0,false);
    	
	}
	
	//@SuppressWarnings("deprecation")
	public void setCamera(int camIndex, boolean resetCamera) {
//		if(camIndex > numCameras-1) camIndex = 0;
		if(resetCamera) {
		//	camera[camIndex].close();
//			camera[camIndex].start			
		}
		server.setSource(camera[camIndex]);
		//SmartDashboard.putString("Camera","cam " + camIndex);
		NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection").setString(camera[camIndex].getName());
//		NetworkTable.getTable("").putString("CameraSelection", camera[camIndex].getName());
		curCamera = camIndex;
	}
	public void nextCamera(){
		setCamera((curCamera+1)%numCameras,true);
	}
}
