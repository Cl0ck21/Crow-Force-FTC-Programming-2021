package org.firstinspires.ftc.teamcode.teamcode.Subsystems.ForMainRobot;


import com.qualcomm.robotcore.hardware.DcMotor;
import system.gui.menus.TelemetryMenu;
import system.robot.Robot;
import system.robot.SubSystem;


public class EncoderSubsystem extends SubSystem {

    TelemetryMenu dMenu;
    public DcMotor strafeMotor;
    public DcMotor forwardMotor;

    public EncoderSubsystem(Robot r, String fMotor, String sMotor) {
        super(r);
        dMenu = new TelemetryMenu();
        robot.gui.addRootMenu(dMenu);
        strafeMotor = robot.hardwareMap.dcMotor.get("strafeEncoder");
        forwardMotor = robot.hardwareMap.dcMotor.get("forwardEncoder");

        forwardMotor = robot.hardwareMap.dcMotor.get(fMotor);
        strafeMotor = robot.hardwareMap.dcMotor.get(sMotor);
    }

    public int fEncoders() {
        return forwardMotor.getCurrentPosition();
    }

    public int sEncoders() {
        return strafeMotor.getCurrentPosition();
    }

    @Override
    public void init(){

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start(){
        forwardMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forwardMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void handle() {
        robot.telemetry.addData("forward", fEncoders());
        robot.telemetry.addData("strafe", sEncoders());
        robot.telemetry.update();
    }

    @Override
    public void stop() {

    }
}
