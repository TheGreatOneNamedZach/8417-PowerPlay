package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.other.secrets;

import java.util.ArrayList;
import java.util.List;

/**
 * This Linear OpMode is an Autonomous.
 * It uses TensorFlow and Vuforia to detect the signal cone in the FTC PowerPlay season.
 * You can use this OpMode to test your image detection.
 * To get started, you have to do six things:
 * 1.) Add your image models into "modelArray" following the instructions above "modelArray"
 * 2.) Remove the "@Disabled" line. It is the first line of actual code below these comments
 * 3.) Remove the line right above the "VUFORIA_KEY" line (the line to remove ends with "new secrets();")
 * 4.) "VUFORIA_KEY" needs a free Vuforia key from https://developer.vuforia.com/vui/develop/licenses
 * 5.) "cameraName" needs to be the name of the camera you are using on your robot. By default this is "Webcam 1"
 * 6.) After you are done using the example models in the "modelArray" to make sure you correctly
 * imported your model into this autonomous, delete the "Custom TF Model (V2)" and "Custom TF Model (V4)" examples
 * as they may crash the code.
 *
 * That's it! You should not need to modify any other lines of code.
 * In the event that you do need to modify something, most lines of code explain what they do.
 *
 * Want to see what your image detection is detecting in real time?
 * If you are using a control hub, plug a monitor into the HDMI port.
 */

/* How to find the name of the built in asset model:
1.) Navigate to
FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/

2.) In this folder there are two folders. Open the one called "external"
3.) Open "ConceptTensorFlowObjectDetection.java"
4.) Find the class line. This is below the "@Disabled" line. For me (the author), this is line 54.
5.) Find the first line of actual code below the class line. It will look like this:

private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";

6.) In the example above, "PowerPlay.tflite" is the name of the asset-type model.
This image detection model is made by FTC and is built into your code.
You do not need to import the model into your code. You can simply call it.
For the purposes of this autonomous, "TRUE_IF_ASSET_FALSE_IF_FILE" should be "true".

Guide Last Updated: Feb 4, 2023
 */




/* How to import a .tflite model that your team made:
NOTE: TFLITE MODELS FROM TEACHABLE MACHINE WILL NOT WORK!!!
You can make a tflite model at https://ftc-ml.firstinspires.org/
NOTE: This assumes your team is using a Control Hub in place of a RC phone.
This is not tested for RC phones and may or may not work with them.

1.) Turn on your robot.
2.) Get on a PC that has the .tflite model.
3.) Open your PC WiFi settings and connect to your robot's WiFi.
This may require a password. The password is the exact same as what you use to connect your Driver Station to your robot.
4.) Open your internet browser of preference. This has only been tested with Chrome but should work on any modern browser.
5.) Go to http://192.168.43.1:8080/?page=FtcTFLiteModels.html
6.) Click the "Upload Models" button. You should now see text that says "No file chosen".
7.) Click "Choose Files".
8.) Navigate to the .tflite model you want to upload. Click on it then press "Open".
If you have done it correctly, the "No file chosen" text should now be replaced with the name of your .tflite model.
9.) Click "Ok"
It may take a moment to upload your .tflite model to your robot.
That's it! Keep in mind that this image detection model is NOT an asset. It is a file and should be imported as such in your code.
For the purposes of this autonomous, "TRUE_IF_ASSET_FALSE_IF_FILE" should be "false".

Guide Last Updated: Feb 4, 2023
*/




/* How to change the name of a model you just uploaded following the guide above:
1.) Do steps 1-5 in the guide "How to import a .tflite model that your team made".
2.) Find the specific model you want to change the name of.
3.) To the left of its name, there is an unmarked checkbox. Click it.
4.) Click the "Rename Selected Model" button.
5.) In the popup, replace the model name with whatever you want.
NOTE: Make sure that the name ends with ".tflite".
6.) Click "Ok" when you are done.
If the model name now ends with ".tflite.tflite" it is strongly recommended you do steps 2-6 again and delete the duplicate ".tflite".
That's it!

Guide Last Updated: Feb 4, 2023
 */

@Disabled
@Autonomous(name = "TensorFlow Test", group = "VisionTest")
public class tensorFlow8417 extends LinearOpMode {
    /* "modelArray" contains all information needed for image detection.
    The following is an example of what needs to be added:


    DISPLAY_NAME, PATH_TO_MODEL, TRUE_IF_ASSET_FALSE_IF_FILE, AMOUNT_OF_LABELS,
    LABEL_NAME_1, LABEL_NAME_2, LABEL_NAME_3


    The display name can be whatever you want. Make it something you can identify the model by.
    Examples of "PATH_TO_MODEL" are in "modelArray". If you are using the default one, it should look like the first example.
    If you are using a custom model, the "PATH_TO_MODEL" will look something like the 2nd and 3rd examples.
    If the model is provided by FTC (built into the code), "TRUE_IF_ASSET_FALSE_IF_FILE" should be "true".
    If the model is custom made (your team imported it onto your robot), "TRUE_IF_ASSET_FALSE_IF_FILE" should be "false".
    "AMOUNT_OF_LABELS" should be the amount of labels you have for that SPECIFIC model. This is usually 3.
    The label names are the names of the labels used by the model.
    The label names are in a specific order and may take trial and error to get right order.
    There is a comma at the end of each line WITH THE EXCEPTION OF THE FINAL LINE OF THE FINAL MODEL!
    */
    private static final Object[] modelArray = {
            "Default TF Model", "PowerPlay.tflite", true, 3,
            "Bolt", "Light", "Panel",

            "Custom TF Model (V2)", "/sdcard/FIRST/tflitemodels/model_FTC8417_V2.tflite", false, 3,
            "Handsaw", "Robot", "Turtle",

            "Custom TF Model (V4)", "/sdcard/FIRST/tflitemodels/model_FTC8417_V4.tflite", false, 3,
            "Handsaw", "Robot", "Turtle"
    };

    static org.firstinspires.ftc.teamcode.other.secrets secrets = new secrets(); // If you are not team 8417, delete this line
    private static final String VUFORIA_KEY = secrets.REPLACE_ME_WITH_YOUR_OWN_VUFORIA_KEY; // This string should be your Vuforia key
    // An example of what the VUFORIA_KEY line of code should look like:
    // private static final String VUFORIA_KEY = "pcFBrqfDjokqVuRntctuAKXzTgoayQOKzrUbqqpXYSyVDrgsYdHytCQqTXmUeYPwgUcXlufTNnTxjcfELzmZaGYyYlnuwrmePRycYtkupohEUdIieyOsrELd";
    private static final String cameraName = "Webcam 1";

    // Only detects images that the image detector is at LEAST 75% confidant is the correct image
    // 75% = 0.75f
    // The lower this number, the greater the amount of images there will be detected
    // It is recommended that you use 80% (0.80f) especially if the model is detecting 4+ images when it should only detect 1
    private final float minConfidenceFloat = 0.75f;

    private int currentModelIndex = 0;
    StringBuilder labels = new StringBuilder(); // Makes a new StringBuilder

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() { // init()

        // Displays on the phone screen the labels of the currently selected model
        for(int i = 1; i <= (int)modelArray[currentModelIndex + 3]; i++) { // Adds each label for the current model to each other in a string
            labels.append(modelArray[currentModelIndex + 3 + i]);
            if(i < (int)modelArray[currentModelIndex + 3]) { // Adds spacing
                labels.append(", ");
            }
        }

        while(!gamepad1.y && opModeInInit()) { // init_loop()
            // Allows the driver to select the model to use

            // Gives the driver instructions
            telemetry.addData("Model Selected", modelArray[currentModelIndex]);
            telemetry.addData("Labels", labels.toString() + "\nUse the B button on gamepad 1 to scroll through the models.\nConfirm your selection by pressing Y on gamepad 1.");

            if(gamepad1.b) { // Cycles through the models from start to finish. Then, loops back to the first model
                currentModelIndex = currentModelIndex + 4 + (int)modelArray[currentModelIndex + 3];
                if(currentModelIndex >= (int)modelArray.length) {
                    currentModelIndex -= (int)modelArray.length;
                }
                labels = new StringBuilder();
                sleep(350);
                // Displays on the phone screen the labels of the currently selected model
                for(int i = 1; i <= (int)modelArray[currentModelIndex + 3]; i++) { // Adds each label for the current model to each other in a string
                    labels.append(modelArray[currentModelIndex + 3 + i]);
                    if(i < (int)modelArray[currentModelIndex + 3]) { // Adds spacing
                        labels.append(", ");
                    }
                }
            }
            telemetry.update();
        }

        telemetry.addData("Status", "Loading...\nDo not press the PLAY button!");
        telemetry.update();

        initVuforia(); // Initialises Vuforia
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

        telemetry.addData("Status", "Ready!\nYou may now start the autonomous.");
        telemetry.update();

        waitForStart(); // start()

        if (opModeIsActive()) {
            while (opModeIsActive()) { // loop()
                if (tfod != null) { // If TensorFlow has been initialised...
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions(); // Creates a list with all newly found images
                    if (updatedRecognitions != null) { // Runs if a new image has been found
                        telemetry.addData("Images Detected", updatedRecognitions.size()); // Displays the number of objects detected on the phone screen

                        // This "for" loop will output data about every image found to the phone screen
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2; // Finds the "X" position of the image on the webcam stream
                            double row = (recognition.getTop() + recognition.getBottom()) / 2; // Finds the "Y" position of the image on the webcam stream
                            double width = Math.abs(recognition.getRight() - recognition.getLeft()); // Finds the width of the image on the webcam stream
                            double height = Math.abs(recognition.getTop() - recognition.getBottom()); // Finds the heights of the image on the webcam stream

                            // Displays to the phone screen the name of the image and how confident that it is correct
                            telemetry.addData("\nImage", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            // Displays to the phone screen the X and Y coordinates of the image. The coordinates are the image's position on the webcam stream
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            // Displays to the phone screen the size of the image. The size of the image output is the size on the webcam stream
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                        }
                    }
                }
                telemetry.update();
            }
        }
    }

    private void initVuforia() { // Initialises Vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(); // Creates a new list of parameters to use for Vuforia
        parameters.vuforiaLicenseKey = VUFORIA_KEY; // Obtains a valid Vuforia key that is stored elsewhere in the code
        parameters.cameraName = hardwareMap.get(WebcamName.class, cameraName); // Tells Vuforia which camera to use

        vuforia = ClassFactory.getInstance().createVuforia(parameters); // Starts Vuforia
    }

    private void initTfod() { // Initialises TensorFlow
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName()); // Gets the ID it should use
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId); // Create a new list of parameters to use for TensorFlow
        tfodParameters.minResultConfidence = minConfidenceFloat; // Anything above this percentage will be shown as an image. Any images below this percentage are removed
        tfodParameters.isModelTensorFlow2 = true; // Which major version of TensorFlow is being used
        tfodParameters.inputSize = 300; // Size of the camera stream in pixels?
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia); // Obtains the already running Vuforia instance and uses it

        // Gets the labels for the specific model
        ArrayList<String> labelsArrayList = new ArrayList<>(); // Creates a new String array list
        for(int i = 1; i <= (int)modelArray[currentModelIndex + 3]; i++) { // Adds each label to the String array list
            labelsArrayList.add(String.valueOf(modelArray[currentModelIndex + 3 + i]));
        }
        String[] labelsArray = new String[labelsArrayList.size()]; // Creates a new String array
        labelsArrayList.toArray(labelsArray); // The String array now has the labels

        if((boolean)modelArray[currentModelIndex + 2]) { // If the selected model is an asset...
            tfod.loadModelFromAsset(String.valueOf(modelArray[currentModelIndex + 1]), labelsArray); // Loads the model as an asset
        } else { // If the selected model is NOT an asset...
            tfod.loadModelFromFile(String.valueOf(modelArray[currentModelIndex + 1]), labelsArray); // Loads the model as a file
        }
    }
}