package frc.robot.commands.climber;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberR2;

public class ClimbManual extends Command{

    ClimberR2 climber;
    double testL;
    double testR;

    public ClimbManual(ClimberR2 climber, double testL, double testR){
        this.climber = climber;
        this.testL = testL;
        this.testR = testR;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        this.climber.testPower(testL, testR);
    }

    @Override
    public void end(boolean interrupt) {
        this.climber.testPower(0, 0);
    }
}
