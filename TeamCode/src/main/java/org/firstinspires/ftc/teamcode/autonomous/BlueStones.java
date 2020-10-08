package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.game.Alliance;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Blue SD", group="Phoebe")
@Disabled
public class BlueStones extends AutonomousHelperStones {

    @Override
    public void init() {
        super.init(Alliance.Color.BLUE);
    }
}