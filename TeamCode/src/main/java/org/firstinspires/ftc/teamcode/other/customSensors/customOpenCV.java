package org.firstinspires.ftc.teamcode.other.customSensors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class customOpenCV {
    // CONSTRUCT
    emptyPipeline pipeline = new emptyPipeline();
    // DECLARE NULL
    OpenCvCamera webcam;
    int cameraMonitorViewId;
    // DECLARE CUSTOM

    // METHODS

    public void init(@NonNull OpMode opMode, @NonNull WebcamName webcamName, int webcamWidth, int webcamHeight, OpenCvCameraRotation rotation) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                start(webcamWidth, webcamHeight, rotation);

            }
            @Override
            public void onError(int errorCode)
            {
                 // This will be called if the camera could not be opened
            }
        });
    }

    public void start(int webcamWidth, int webcamHeight, OpenCvCameraRotation rotation) {
        webcam.startStreaming(webcamWidth, webcamHeight, rotation);
        webcam.setPipeline(pipeline);
    }

    public void pause() {

    }
}

// https://github.com/OpenFTC/EasyOpenCV/blob/master/doc/user_docs/pipelines_overview.md
class emptyPipeline extends OpenCvPipeline {
    @Override
    public Mat processFrame(Mat input)
    {
        return input;
    }
}