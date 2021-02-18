package org.firstinspires.ftc.teamcode.teamcode.Subsystems.ForMainRobot;


import com.qualcomm.robotcore.hardware.Servo;
import system.config.ConfigParam;
import system.config.TeleopConfig;
import system.robot.Robot;
import system.robot.SubSystem;
import util.control.Button;
import util.control.CustomizableGamepad;
import util.control.Toggle;

public class IntakeSubSystemServoNew2Servos extends SubSystem {
    private CustomizableGamepad inputs;
    Servo IntakeServoVertical;
    Servo IntakeServoGrabber;
    private final double UP = 0;
    private final double DOWN = .6;
    int down = 0;
    int up = 0;
    long startTime;

    Toggle toggleVertical = new Toggle(Toggle.ToggleTypes.flipToggle, false);
    Toggle toggleGrabberUp = new Toggle(Toggle.ToggleTypes.trueOnceToggle, false);
    Toggle toggleGrabberDown = new Toggle(Toggle.ToggleTypes.trueOnceToggle, false);

    static final String VERTICALBUTTON = "VerticalButton";
    static final String GRABBERUPBUTTON = "GrabberButtonGood";
    static final String GRABBERDOWNBUTTON = "GrabberButton";
    static final String RELEASEBUTTON = "ReleaseButton";



    public IntakeSubSystemServoNew2Servos(Robot r, String servoVertical, String servoGrabber) {
        super(r);
        IntakeServoVertical = robot.hardwareMap.servo.get(servoVertical);
        IntakeServoGrabber = robot.hardwareMap.servo.get(servoGrabber);
        usesConfig = true;
    }


    @Override
    public void init()  {
        IntakeServoVertical.setPosition(UP);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start()  {
        inputs = robot.pullControls(this);
    }
    @Override
    public void handle ()  {
        robot.telemetry.addData("vertical", toggleVertical.getCurrentState());
        robot.telemetry.addData("down", toggleGrabberDown.getCurrentState());
        robot.telemetry.addData("up", toggleGrabberUp.getCurrentState());
        robot.telemetry.update();
        toggleVertical.updateToggle(inputs.getInput(VERTICALBUTTON));
        toggleGrabberDown.updateToggle(inputs.getInput(GRABBERDOWNBUTTON));
        toggleGrabberUp.updateToggle(inputs.getInput(GRABBERUPBUTTON));


        if(toggleVertical.getCurrentState()){
            intakeDown();
        }
        else{
            intakeUp();
        }

        if (robot.gamepad2.b) {
            //Log.wtf("Down", "Yes");
            //IntakeServoGrabber.setDirection(CRServo.Direction.FORWARD);
            IntakeServoGrabber.setPosition(-60);
                //down++;
                //up = 0;
        }
        if (robot.gamepad2.y) {
            //Log.wtf("Up", "Yes");
            //IntakeServoGrabber.setDirection(CRServo.Direction.REVERSE);
                IntakeServoGrabber.setPosition(80);
                //up++;
                //down = 0;
        }
        /*if (down == 2 || up == 2){
            IntakeServoGrabber.setPosition(0);
            down = 0;
            up = 0;
        }*/

    }

    @Override
    public void stop ()  {

    }

    public void intakeDown () { IntakeServoVertical.setPosition(DOWN); }

    public void intakeUp() {
        IntakeServoVertical.setPosition(UP);
    }




    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[]{
                new ConfigParam(VERTICALBUTTON, Button.BooleanInputs.x,2),
                new ConfigParam(GRABBERUPBUTTON, Button.BooleanInputs.y,2),
                new ConfigParam(GRABBERDOWNBUTTON, Button.BooleanInputs.b, 2)
        };
    }
    public void GrabberOpen() {
        //IntakeServoGrabber.setDirection(CRServo.Direction.FORWARD);
        IntakeServoVertical.setPosition(UP);
        IntakeServoGrabber.setPosition(UP);
    }
    public void GrabberClose() {
        //IntakeServoGrabber.setDirection(CRServo.Direction.REVERSE);
        IntakeServoVertical.setPosition(DOWN);
        IntakeServoGrabber.setPosition(DOWN);
    }
    public void buttonPressUp() {

    }
}

