package org.firstinspires.ftc.teamcode.TeleOperado;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Auxiliar.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Localizer;

@TeleOp
public class TeleOpMovimentacao extends OpMode
{
    DcMotor fr_esq;
    DcMotor fr_dir;
    DcMotor ba_esq;
    DcMotor ba_dir;

    Localizer localizer;
    @Override
    public void init()
    {
        fr_esq = hardwareMap.get(DcMotor.class,"left_front_drive");
        fr_dir = hardwareMap.get(DcMotor.class,"right_front_drive");
        ba_esq = hardwareMap.get(DcMotor.class,"left_back_drive");
        ba_dir = hardwareMap.get(DcMotor.class,"right_back_drive");

        ba_dir.setDirection(DcMotorSimple.Direction.REVERSE);
        fr_dir.setDirection(DcMotorSimple.Direction.REVERSE);

        localizer = new Localizer(hardwareMap);
    }

    @Override
    public void loop()
    {
        double anal_esq_x = gamepad1.left_stick_x;
        double anal_esq_y = -gamepad1.left_stick_y;
        double anal_dir_x = -gamepad1.right_stick_x;

        double pfr_esq = anal_esq_y + anal_esq_x - anal_dir_x;
        double pfr_dir = anal_esq_y - anal_esq_x + anal_dir_x;
        double pba_esq = anal_esq_y - anal_esq_x - anal_dir_x;
        double pba_dir = anal_esq_y + anal_esq_x + anal_dir_x;

        double factor = Math.max(Math.max(Math.max(Math.abs(pfr_esq),Math.abs(pfr_dir)),Math.abs(pba_esq)),Math.abs(pba_dir));

        fr_esq.setPower(pfr_esq / (2 * factor));
        fr_dir.setPower(pfr_dir / (2 * factor));
        ba_esq.setPower(pba_esq / (2 * factor));
        ba_dir.setPower(pba_dir / (2 * factor));

        localizer.update();

        Vector2D currentPosition = localizer.getCurrentPosition();
        telemetry.addData("x ", currentPosition.x);
        telemetry.addData("y ", currentPosition.y);
        telemetry.addData("angle ", Math.toDegrees(localizer.currentAngle));
        telemetry.update();
    }
}
