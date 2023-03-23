//package org.firstinspires.ftc.teamcode.THISIS10111.Camera;
//
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.THISIS10111.hardwaremaps.Robot10111;
////import org.firstinspires.ftc.teamcode.hardwaremaps.Robot;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//
//public class Camera {
//    public OpenCvInternalCamera phoneCam;
//    public TSEDetectorPipeline pipeline;
//
//    public Camera(final HardwareMap hwMap)
//    {
//        Robot10111 robot = Robot10111.getInstance();
//
//        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
//        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
//        pipeline = new TSEDetectorPipeline();
//        phoneCam.setPipeline(pipeline);
//
//        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
//        // out when the RC activity is in portrait. We do our actual image processing assuming
//        // landscape orientation, though.
//        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
//        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//            }
//        });
//    }
//    public TSEPosition getPosition() {
//        return pipeline.getAnalysis();
//    }
//
//    public void endCamera()
//    {
//        phoneCam.closeCameraDevice();
//
//    }
//
//
//
//
//
//
//
//}