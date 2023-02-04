package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.other.secrets;

/**
 * This Linear OpMode is an Autonomous.
 * This autonomous is to test our image detection.
 * This is the same as tF8417Main.java but:
 * 1.) It is linear unlike tF8417Main.java
 * 2.) It is all in one file
 *
 * THIS CAN BE USED AS A BASE TEMPLATE FOR ANY TENSORFLOW AUTONOMOUS
 */

@Disabled
@Autonomous(name = "TensorFlow Test", group = "VisionTest")
public class tensorFlow8417 extends LinearOpMode {
    /* The models to be used for image detection and their labels
    It is assumed that every model has three labels (this is modifiable)


    DISPLAY_NAME, PATH_TO_MODEL, ASSET_OR_FILE,
    LABEL_NAME_1, LABEL_NAME_2, LABEL_NAME_3


    The display name can be whatever you want
    If the model is provided (built into the code), "ASSET_OR_FILE" should be "Asset"
    If the model is custom made (your team imported it onto your robot), "ASSET_OR_FILE" should be "File"
     */
    private static final String[] modelArray = {
            "Default TF Model", "PowerPlay.tflite", "Asset",
            "Bolt", "Light", "Panel",

            "Custom TF Model (V2)", "/sdcard/FIRST/tflitemodels/model_FTC8417_V2.tflite", "File",
            "Handsaw", "Robot", "Turtle",

            "Custom TF Model (V4)", "/sdcard/FIRST/tflitemodels/model_FTC8417_V4.tflite", "File",
            "Handsaw", "Robot", "Turtle"
    };

    private int currentModelIndex = 0;

    static org.firstinspires.ftc.teamcode.other.secrets secrets = new secrets();
    private static final String VUFORIA_KEY = secrets.REPLACE_ME_WITH_YOUR_OWN_VUFORIA_KEY;

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() { // init()
        initVuforia(); // Initialises Vuforia

        while(!gamepad1.y) { // init_loop()
            // Allows the driver to select the model to use

            // Gives the driver instructions
            telemetry.addData("Model Selected", modelArray[currentModelIndex * 6] + "\nUse the left and right D-Pad buttons to scroll through the models.\nConfirm your selection by pressing Y.");

            // Displays on the phone screen the labels of the currently selected model
            telemetry.addData("Labels", modelArray[(currentModelIndex * 6) + 3] + ", " + modelArray[(currentModelIndex * 6) + 4] + ", " + modelArray[(currentModelIndex * 6) + 5]);

            if(gamepad1.dpad_left && currentModelIndex != 0){ // Scrolls left through the models but stops at the first model
                currentModelIndex -= 6;
                sleep(200);
            } else if (gamepad1.dpad_right && (currentModelIndex + 5) != modelArray.length) { // Scrolls right through the models but stops at the last model
                currentModelIndex += 6;
                sleep(200);
            }
        }

        telemetry.addData("Status", "Loading...");

        initTfod(); // Initialises TensorFlow

        if (tfod != null) { // If TensorFlow has been initialised...
            tfod.activate(); // Activate TensorFlow

            /* The TensorFlow software will scale the input images from the camera to a lower resolution.
            This can result in lower detection accuracy at longer distances (> 55cm or 22").
            If your target is at distance greater than 50 cm (20") you can increase the magnification value
            to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            should be set to the value of the images used to create the TensorFlow Object Detection model
            (typically 16/9). */
            tfod.setZoom(1.0, 16.0/9.0);
        }

        telemetry.addData("Status", "Ready!");

        waitForStart(); // start()

        if (opModeIsActive()) {
            while (opModeIsActive()) { // loop()
                if (tfod != null) { // If TensorFlow has been initialised...
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions(); // Creates a list with all newly found images
                    if (updatedRecognitions != null) { // Runs if a new image has been found
                        telemetry.addData("# Objects Detected", updatedRecognitions.size()); // Displays the number of objects detected on the phone screen

                        // This "for" loop will output data about every image found to the phone screen
                        for (Recognition recognition : updatedRecognitions) {
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
        }
    }


    private void initVuforia() { // Initialises Vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(); // Creates a new list of parameters to use for Vuforia
        parameters.vuforiaLicenseKey = VUFORIA_KEY; // Obtains a valid Vuforia key that is stored elsewhere in the code
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1"); // Tells Vuforia which camera to use

        vuforia = ClassFactory.getInstance().createVuforia(parameters); // Starts Vuforia
    }

    private void initTfod() { // Initialises TensorFlow
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()); // Gets the ID it should use
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId); // Create a new list of parameters to use for TensorFlow
        tfodParameters.minResultConfidence = 0.75f; // Anything above this percentage will be shown as an image. Any images below this percentage are removed
        tfodParameters.isModelTensorFlow2 = true; // Which major version of TensorFlow is being used
        tfodParameters.inputSize = 300; // Size of the camera stream in pixels?
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia); // Obtains the already running Vuforia instance and uses it

        // Gets the labels for the specific model
        String[] tempLabels = {modelArray[(currentModelIndex * 6) + 3], modelArray[(currentModelIndex * 6) + 4], modelArray[(currentModelIndex * 6) + 5]};
        if(modelArray[(currentModelIndex * 6) + 2].equalsIgnoreCase("Asset")) { // If the selected model is an asset...
            tfod.loadModelFromAsset(modelArray[(currentModelIndex * 6) + 1], tempLabels); // Loads the model as an asset
        } else { // If the selected model is NOT an asset...
            tfod.loadModelFromFile(modelArray[(currentModelIndex * 6) + 1], tempLabels); // Loads the model as a file
        }
    }
}
