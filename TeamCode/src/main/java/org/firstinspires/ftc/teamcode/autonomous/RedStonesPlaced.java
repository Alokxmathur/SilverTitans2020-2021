package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.game.Alliance;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Red SP", group="Phoebe")
@Disabled
public class RedStonesPlaced extends AutonomousHelperStonesPlaced {

    @Override
    public void init() {
        super.init(Alliance.Color.RED);
    }
}