package org.firstinspires.ftc.teamcode.teamcode.Subsystems.ForMainRobot;




import com.qualcomm.robotcore.hardware.Servo;
import system.config.ConfigParam;
import system.config.TeleopConfig;
import system.robot.Robot;
import system.robot.SubSystem;
import util.control.Button;
import util.control.CustomizableGamepad;
import util.control.Toggle;

public class PlopperSubsystem extends SubSystem {

    CustomizableGamepad inputs;
    public Servo armServo;
    public Servo clawServo;
    static final String ARMBUTTON = "ArmButton";
    static final String CLAWBUTTON = "ClawButton";
    Toggle armToggle;
    Toggle clawToggle;
    public PlopperSubsystem(Robot r, String armServo, String clawServo) {
        super(r);
        this.armServo = robot.hardwareMap.servo.get(armServo);
        this.clawServo = robot.hardwareMap.servo.get(clawServo);
        armToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
        clawToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
        usesConfig = true;
    }

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        inputs = robot.pullControls(this);
    }

    @Override
    public void handle() {
        armToggle.updateToggle(inputs.getInput("armToggleButton"));
        clawToggle.updateToggle(inputs.getInput("armToggleButton"));
        if(armToggle.getCurrentState()){
            armServo.setPosition(1);
        }
        else {
            armServo.setPosition(-1);
        }
        if(clawToggle.getCurrentState()){
            clawServo.setPosition(1);
        }
        else {
            clawServo.setPosition(-1);
        }
    }

    @Override
    public void stop() {

    }

    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[]{
                new ConfigParam(ARMBUTTON, Button.BooleanInputs.b, 2),
                new ConfigParam(CLAWBUTTON, Button.BooleanInputs.x, 2)
        };
    }
}
