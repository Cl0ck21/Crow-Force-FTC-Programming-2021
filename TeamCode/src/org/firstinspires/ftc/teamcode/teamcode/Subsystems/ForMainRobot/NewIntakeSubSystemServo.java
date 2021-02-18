package org.firstinspires.ftc.teamcode.teamcode.Subsystems.ForMainRobot;



import com.qualcomm.robotcore.hardware.Servo;
import system.config.ConfigParam;
import system.config.TeleopConfig;
import system.gui.menus.TelemetryMenu;
import system.robot.Robot;
import system.robot.SubSystem;
import util.control.Button;
import util.control.CustomizableGamepad;
import util.control.Toggle;

public class NewIntakeSubSystemServo extends SubSystem {
    private CustomizableGamepad inputs;
    Servo IntakeServo;
    private final int DOWN = 1;
    private final double UP = 0.75;
    Toggle toggle = new Toggle(Toggle.ToggleTypes.flipToggle, true);

    static final String INTAKEBUTTON = "IntakeButton";

    TelemetryMenu dMenu = new TelemetryMenu();
    public NewIntakeSubSystemServo(Robot r, String servo) {
        super(r);
        IntakeServo = robot.hardwareMap.servo.get(servo);
        usesConfig = true;
        robot.gui.addRootMenu(dMenu);
    }



    @Override
    public void init()  {
    }

    @Override
    public void init_loop()  {

    }

    @Override
    public void start()  {
        inputs = robot.pullControls(this);
    }
    @Override
    public void handle ()  {
        dMenu.addData("IntakeButton", inputs.getInput(INTAKEBUTTON));
        toggle.updateToggle(inputs.getInput(INTAKEBUTTON));
        if (toggle.getCurrentState()) {
            IntakeServo.setPosition(DOWN);
        }
        else {
            IntakeServo.setPosition(UP);
        }
    }

    @Override
    public void stop ()  {

    }

    public void intake () {
        IntakeServo.setPosition(DOWN);
    }


    public void output() {
        IntakeServo.setPosition(UP);
    }


    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[]{
                new ConfigParam(INTAKEBUTTON, Button.BooleanInputs.right_bumper),
        };
    }
}

