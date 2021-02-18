package org.firstinspires.ftc.teamcode.teamcode.Subsystems.ForMainRobot;



import com.qualcomm.robotcore.hardware.Servo;
import system.robot.Robot;
import system.robot.SubSystem;
import util.control.Button;
import util.control.CustomizableGamepad;
import util.control.Toggle;

public class LinearSlidesServosSubsystem extends SubSystem {
    CustomizableGamepad inputs;
    Servo servoGrab;
    Servo servoRotate;
    Toggle grabToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
    int servoRotateState = 0; //0 = back, 1 = up, 2 = down
    int servoPosition = 0; //servo position idk what number it is we will probably have to change it
    public LinearSlidesServosSubsystem(Robot r, String servoG, String servoR) {
        super(r);
        servoGrab = robot.hardwareMap.servo.get(servoG);
        servoRotate = robot.hardwareMap.servo.get(servoR);
        inputs = new CustomizableGamepad(r);
        usesConfig = false;
    }

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        inputs.addButton("SetBackButton", Button.BooleanInputs.dpad_left, 2);
        inputs.addButton("SetUpButton", Button.BooleanInputs.dpad_up,2);
        inputs.addButton("SetDownButton", Button.BooleanInputs.dpad_right,2);
        inputs.addButton("GrabButton", Button.BooleanInputs.dpad_down,2);
        servoGrab.setPosition(60);
        servoRotate.setPosition(30);//30

    }

    @Override
    public void handle() {

        //grabToggle.updateToggle(inputs.getBooleanInput("GrabButton"));
        /*if(grabToggle.getCurrentState()) {
            servoGrab.setPosition(60);
        }
        else {
            servoGrab.setPosition(0);
        }
        if(inputs.getBooleanInput("SetBackButton")) {
            servoRotateState = 0;
        }
        if(inputs.getBooleanInput("SetUpButton")) {
            servoRotateState = 1;
        }
        if(inputs.getBooleanInput("SetDownButton")) {
            servoRotateState = 2;
        }
        switch(servoRotateState) {
            case 0:
                servoRotate.setPosition(-50);//225
                break;
            case 1:

                servoRotate.setPosition(-30);//30
                break;
            case 2:

                servoRotate.setPosition(300);//300
                break;
        }*/

        while(robot.gamepad2.left_stick_y > 0.2){
            servoGrab.setPosition(servoPosition);
            servoPosition = servoPosition + 2;
        }
        while(robot.gamepad2.left_stick_y < -0.2){
            servoGrab.setPosition(servoPosition);
            servoPosition = servoPosition - 2;
        }

    }

    @Override
    public void stop() {
    }

}
