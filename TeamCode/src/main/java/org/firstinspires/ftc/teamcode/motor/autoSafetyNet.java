package org.firstinspires.ftc.teamcode.motor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.vision.tF8417Main;

import java.util.List;
import java.util.Objects;

/**
 * This OpMode is an Autonomous.
 * It uses Vuforia and TensorFlow to detect a custom (or default) sleeve on the signal cone.
 * The robot moves on a timer. This autonomous is a backup to our RoadRunner autonomous.
 */

@Autonomous
public class autoSafetyNet extends OpMode {
    DcMotor fR = null; // All four drive motors
    DcMotor fL = null;
    DcMotor bR = null;
    DcMotor bL = null;
    public ElapsedTime autoRuntime = new ElapsedTime(); // How long the autonomous has run for
    ElapsedTime actionRuntime = new ElapsedTime(); // How long the current action has run for
    int robotAction = 0; // Keeps track of which action the bot is currently doing
    double length = 1.0; // The time the action runs for
    double slow = .5; // Power multiplier for the robot's wheel speed
    org.firstinspires.ftc.teamcode.vision.tF8417Main detector = new tF8417Main(); // Image detector
    String duck = "not found."; // Stores the name of the found image that has the highest confidence. This is the same as "tempDuck" but is never null
    String tempDuck = null; // Stores the name of any newly found image. This will be null when no NEW image is found
    public String teamColor = "Red"; // Which alliance we are currently on
    Boolean teamSelected = false; // Has the primary driver selected an alliance?
    Boolean tFInitHasRun = false; // Has the TensorFlow initialise method run already?

    public void init(){
        fR = hardwareMap.get(DcMotor.class, "Front Right"); // Hardware maps the motors
        fL = hardwareMap.get(DcMotor.class, "Front Left");
        bR = hardwareMap.get(DcMotor.class, "Back Right");
        bL = hardwareMap.get(DcMotor.class, "Back Left");

        bL.setDirection(DcMotorSimple.Direction.REVERSE); // Sets the correct direction for the motors
        fL.setDirection(DcMotorSimple.Direction.REVERSE);

        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Sets the mode of the motors to run WITHOUT encoders
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Gives the image detector the information to access OpMode specific objects (e.g. motors, webcam, and telemetry)
        detector.init(this);

        // Initialises Vuforia
        detector.initVuforia();

        // TODO: Start OpenCV here.
    }

    // The primary driver will select which image detection model they want to use for the match
    // This is determined by the alliance they are currently on
    // In addition, the primary driver can choose to use the default model made by FTC instead of a custom sleeve
    public void init_loop(){
        if(gamepad1.x) { // If the blue button (X) is pressed, select the blue alliance
            teamColor = "Blue";
        } else if (gamepad1.b) { // If the red button (B) is pressed, select the red alliance
            teamColor = "Red";
        } else if (gamepad1.a) { // If the green button (A) is pressed, use the default model
            teamColor = "Default";
        } else if (gamepad1.y) { // If the Y button is pressed, the driver has selected an alliance
            teamSelected = true;
        }
        if(teamSelected) { // When the team has been selected...
            if(!tFInitHasRun) { // Initialise TensorFlow if it has not been started already
                detector.initTfod(teamColor); // Tell TensorFlow to use the model corresponding to the alliance
                tFInitHasRun = true;
            }

            tempDuck = detector.imageReturn(); // The variable "tempDuck" contains the latest detected image name (if any)
            if(tempDuck != null){ // If "tempDuck" has a detected image name, it is more recent than what is currently in "duck"
                duck = tempDuck; // Since we know the image name in "duck" is outdated, we update it to whatever image name is in "tempDuck"
            }

            // TELEMETRY
            telemetry.addData("Image", duck); // Displays the name of the latest image to the phone screen
            if(duck == null) { // If no image has been found, display "Loading..." on the phone screen
                telemetry.addData("Status", "Loading...");
            } else { // If an image has been found, tell the drivers that autonomous can now be run
                telemetry.addData("Status", "Ready! You can now run Autonomous.");
            }
            tFTelemetry(detector.tfod); // Outputs data about all detected images to the phone screen
        } else { // If the team has not been selected by the driver...
            if (Objects.equals(teamColor, "Blue")) { // Tell them which team is currently selected
                telemetry.addData("Team Alliance", "You are on the " + teamColor + " alliance.\nUse B to change to the red alliance.\nPress A to use no custom sleeve.");
            } else if (Objects.equals(teamColor, "Red")) {
                telemetry.addData("Team Alliance", "You are on the " + teamColor + " alliance.\nUse X to change to the blue alliance.\nPress A to use no custom sleeve.");
            } else {
                telemetry.addData("Team Alliance", "You are using the " + teamColor + " signal cone.\nUse X to change to the blue alliance.\nUse B to change to the red alliance.");
            }

            // TELEMETRY
            // Tells the primary driver how to confirm their selection
            telemetry.addData("Status", "Press Y to confirm your team alliance.");
        }

    }

    public void start() {
        // Runs one time when the autonomous is started
        autoRuntime.reset(); // Resets both timers
        actionRuntime.reset();

        // TODO: Shutdown the image detector here (if it does not cause lag)

    }

    @Override
    public void loop() {
        // TELEMETRY
        telemetry.addData("Time Elapsed For Autonomous", autoRuntime.seconds()); // Time since the autonomous has started
        telemetry.addData("Time Elapsed For Action", actionRuntime.time()); // Displays the current time since the action has started
        telemetry.addData("Robot Action", robotAction); // Displays the current action the robot is on

        if(Objects.equals(duck, "Turtle") || Objects.equals(duck, "Bolt")) {
            // Parks in the left zone if the image is a turtle or lightning bolt
            if(robotAction == 0){ // This action is reserved for initialising motors before the robot moves (e.g. grabbing the preloaded cone)
                //encoderDrive(length, 0, 0, 0);
                robotAction++;
            } else if(robotAction == 1){ // Moves forwards off of the wall
                encoderDrive(.2, 1, 0, 0);
            } else if(robotAction == 2){ // Waits for 0.2 seconds
                encoderDrive(.2, 0,0,0);
            } else if (robotAction == 3) { // Strafes left
                encoderDrive(length, 0,  1, 0);
            } else if(robotAction == 4){ // Waits for 0.2 seconds
                encoderDrive(.2, 0,0,0);
            } else if (robotAction == 5) { // Moves forwards
                encoderDrive(length,1, 0, 0);
            } else if(robotAction == 6){ // Waits 1 second
                encoderDrive(length, 0,0,0);
            }
        }

        if (Objects.equals(duck, "Robot") || Objects.equals(duck, "Light") || Objects.equals(duck, null)) {
            // Parks in the middle zone if the image is a robot or light bulb
            // If no image is ever found, this will run
            // If the driver does not wait for the image detector to load, this will run
            if(robotAction == 0){ // This action is reserved
                //encoderDrive(length, 0,0,0);
                robotAction++;
            } else if(robotAction == 1) { // Moves forward for 1.5 seconds
                encoderDrive(length + .5, 1, 0, 0);
            } else if(robotAction == 2){ // Waits 0.2 seconds
                encoderDrive(.2,0,0,0);
            }
        }

        if(Objects.equals(duck, "Handsaw") || Objects.equals(duck, "Panel")) {
            // Parks in the right zone if the image is the handsaw or solar panel
            if(robotAction == 0){ // This action is reserved
                //encoderDrive(length,0,0,0);
                robotAction++;
            } else if(robotAction == 1) { // Moves forwards off the wall
                encoderDrive(.2, 1, 0, 0);
            } else if(robotAction == 2){ // Waits 0.2 seconds
                encoderDrive(.2,0,0,0);
            } else if(robotAction == 3) { // Strafes right
                encoderDrive(length, 0, -1, 0);
            } else if(robotAction == 4){ // Waits 0.2 seconds
                encoderDrive(.2,0,0,0);
            } else if(robotAction == 5) { // Moves forwards
                encoderDrive(length, 1, 0, 0);
            } else if(robotAction == 6) { // Waits 0.2 seconds
                encoderDrive(.2, 0, 0, 0);
            }
        }
    }

    public void encoderDrive(double secondsToRunFor, double xAxisPower, double yAxisPower, double rotationInDegrees) {
        xAxisPower = xAxisPower * 1.1; // The robot needs more power to strafe (x axis) than to go straight (y axis)
        yAxisPower = -yAxisPower; // The Y axis is flipped
        rotationInDegrees = -rotationInDegrees; // The rotation direction is flipped

        // Calculates motor power
        double ratio = Math.max((Math.abs(yAxisPower) + Math.abs(xAxisPower) + Math.abs(rotationInDegrees)), 1); // "ratio" can be x, y, or rot. Whichever is higher
        double fRMotorPwr = (yAxisPower - xAxisPower + rotationInDegrees) / ratio; // The motor power scaled to be under 1 or greater than -1
        double fLMotorPwr = (-yAxisPower - xAxisPower + rotationInDegrees) / ratio;
        double bRMotorPwr = (-yAxisPower - xAxisPower - rotationInDegrees) / ratio;
        double bLMotorPwr = (yAxisPower - xAxisPower - rotationInDegrees) / ratio;

        // TELEMETRY
        telemetry.addData("fRMotorPwr", fRMotorPwr); // Displays the motor power on the phone screen
        telemetry.addData("fLMotorPwr", fLMotorPwr);
        telemetry.addData("bRMotorPwr", bRMotorPwr);
        telemetry.addData("bLMotorPwr", bLMotorPwr);

        fR.setPower(fRMotorPwr * slow); // Makes the motors go slower
        fL.setPower(fLMotorPwr * slow);
        bR.setPower(bRMotorPwr * slow);
        bL.setPower(bLMotorPwr * slow);
        if(actionRuntime.time() >= secondsToRunFor){ // If this action runs longer than it should...
            robotAction++; // Go to the next action
            actionRuntime.reset(); // Reset the timer
        }
    }

    public void tFTelemetry(TFObjectDetector tfod) { // Displays data about images detected to the phone screen
        if (tfod != null) { // If the image detector has started up...
            List<Recognition> recognitionsList = tfod.getRecognitions(); // Creates a list with every image detected
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
}