package org.firstinspires.ftc.teamcode.action;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.other.customSensors.imageDetectionTF;

import java.util.List;

/** This is an interface for this year's webcam. */
public class webcam {
    // CONSTRUCT
    org.firstinspires.ftc.teamcode.other.customSensors.imageDetectionTF imageDetectionTF = new imageDetectionTF();
    // DECLARE NULL
    Telemetry telemetry;
    // DECLARE CUSTOM
    Boolean hasAlreadyRun = false;
    // METHODS

    /** Initializes the webcam.
     * @param opMode If you are constructing from an Auto or TeleOp, type in "this" without the quotation marks.
     * @param tensorFlow "True" if you want to initialize TensorFlow.
     * @param vuforia "True" if you want to initialize Vuforia.
     * @param teamColor Leave "null" if not initializing TensorFlow.
     * @apiNote "teamColor" specifies which model to use for object detection.
     * "Default" uses the default model.
     * "Blue" Uses the blue cone bias model.
     * "Red", "null", or any other value uses the model trained on just the image with no cone.
     * If no team color is specified when TensorFlow is initialized, the "Red" model is used.
     */
    public void init(@NonNull OpMode opMode, @NonNull Boolean tensorFlow, @NonNull Boolean vuforia, String teamColor) {
        HardwareMap hardwareMap = opMode.hardwareMap;
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        if (vuforia || tensorFlow) {
            if(!hasAlreadyRun) {
                imageDetectionTF.init(opMode);
                hasAlreadyRun = true;
            }
        }
        if (vuforia) {
            imageDetectionTF.initVuforia(webcamName);
        }
        if (tensorFlow) {
            imageDetectionTF.initTfod(teamColor);
        }
        telemetry = opMode.telemetry;
    }

    /** Returns the label of a new image, if any. */
    public String tf_FindNewImages() {
        return imageDetectionTF.imageReturn();
    }

    /** Outputs data about all images seen to the driver station phone. */
    public void tf_OutputTelemetry() {
        if (imageDetectionTF.tfod != null) { // If the image detector has started up...
            List<Recognition> recognitionsList = imageDetectionTF.tfod.getRecognitions(); // Creates a list with every image detected
            if (recognitionsList != null) { // This will run if at least one image has been detected
                telemetry.addData("Number Of Images Detected", recognitionsList.size());

                for (Recognition recognition : recognitionsList) {
                    // This "for" statement will run for every image detected
                    double col = (recognition.getLeft() + recognition.getRight()) / 2; // Finds the "X" position of the image on the webcam stream
                    double row = (recognition.getTop() + recognition.getBottom()) / 2; // Finds the "Y" position of the image on the webcam stream
                    double width = Math.abs(recognition.getRight() - recognition.getLeft()); // Finds the width of the image on the webcam stream
                    double height = Math.abs(recognition.getTop() - recognition.getBottom()); // Finds the heights of the image on the webcam stream

                    // Adds a blank line between every image detected
                    telemetry.addData("", " ");
                    // Displays to the phone screen the name of the image and how confident that it is correct
                    telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    // Displays to the phone screen the X and Y coordinates of the image. The coordinates are the image's position on the webcam stream
                    telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                    // Displays to the phone screen the size of the image. The size of the image output is the size on the webcam stream
                    telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                }
            }
        }
    }

    /** Shuts down the TensorFlow image detection. */
    public void tf_shutdown() {
        imageDetectionTF.tfod.shutdown();
    }
}
