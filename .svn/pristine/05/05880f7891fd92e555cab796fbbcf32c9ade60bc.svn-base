package Chassis;

//import MotorControlSolution.*;
import MotorControl.*;

import javax.swing.*;
import java.awt.*;
import java.awt.event.*;

/**
 * <p>Entry point for chassis lab.</p>
 **/
public class Chassis {

  /**
   * <p>The robot.</p>
   **/
  public static OdometryRobot robot = new OdometryRobot();

  /**
   * <p>Entry point for the chassis lab.</p>
   *
   * @param args command line arguments
   **/
  public static void main(String [] args) {

    //////////////////// create velocity controller //////////////////////    

    RobotVelocityController robotVelocityController =
      new RobotVelocityControllerBalanced();

    robot.setRobotVelocityController(robotVelocityController);


    //////////////////// create position controller /////////////////////    

    RobotPositionController robotPositionController =
      new RobotPositionController(robot);

    robot.setRobotPositionController(robotPositionController);

    //////////////////// config controllers /////////////////////////////    

    //if your velocity and/or position controllers need to be configured
    //(i.e. gains set, etc), then do it here

    //this block to configures *our* solution to lab 2 (yours may or
    //may not be configured the same way)
    final double VELOCITY_BALANCE_GAIN = 1.0; 
    final double VELOCITY_WHEEL_GAIN = 10.0;
    robotVelocityController.setGain(VELOCITY_BALANCE_GAIN);
    robotVelocityController.getWheelVelocityController(RobotBase.LEFT). 
      setGain(VELOCITY_WHEEL_GAIN);
    robotVelocityController.getWheelVelocityController(RobotBase.RIGHT).
      setGain(VELOCITY_WHEEL_GAIN);



    //////////////////// display estop button //////////////////////////    

    EstopButton estop = new EstopButton(Thread.currentThread());


    //////////////////// command motion ////////////////////////////////    

    // Begin Student Code
//    // testing translation
//    robotPositionController.translate(0.3, 1);
//    robotPositionController.translate(-0.3, 1);
//    // robotPositionController.translate(0.3, 0.5);
    
//    // testing rotation
//    robotPositionController.rotate(1, 6.28);
//    robotPositionController.rotate(-1, 3.14);
//    robotPositionController.rotate(1, 3.14);
    
    // out and back test
    robotPositionController.translate(0.3, 1);
    robotVelocityController.setDesiredAngularVelocity(0.,0.);
    try{Thread.sleep(500);} catch (InterruptedException e) {}
    robotPositionController.rotate(0.3, Math.PI/2);
    robotVelocityController.setDesiredAngularVelocity(0.,0.);
    try{Thread.sleep(500);} catch (InterruptedException e) {}
    robotPositionController.translate(0.3, 1);
    robotVelocityController.setDesiredAngularVelocity(0.,0.);
    try{Thread.sleep(500);} catch (InterruptedException e) {}
    robotPositionController.rotate(0.3, Math.PI/2);
    robotVelocityController.setDesiredAngularVelocity(0.,0.);
    try{Thread.sleep(500);} catch (InterruptedException e) {}
    robotPositionController.translate(0.3, 1);
    robotVelocityController.setDesiredAngularVelocity(0.,0.);
    try{Thread.sleep(500);} catch (InterruptedException e) {}
    robotPositionController.rotate(0.3, Math.PI/2);
    robotVelocityController.setDesiredAngularVelocity(0.,0.);
    try{Thread.sleep(500);} catch (InterruptedException e) {}
    robotPositionController.translate(0.3, 1);
    
//    // driving in a square
//    robotPositionController.translate(0.3, 1);
//    robotPositionController.rotate(0.7, Math.PI/2);
//    robotPositionController.translate(0.3, 1);
//    robotPositionController.rotate(0.7, Math.PI/2);
//    robotPositionController.translate(0.3, 1);
//    robotPositionController.rotate(0.7, Math.PI/2);
//    robotPositionController.translate(0.3, 1);
//    robotPositionController.rotate(0.7, Math.PI/2);
    
    // End Student Code

    //////////////////// shutdown //////////////////////////////////////    

    //robot should already be stopped, but just in case
    robot.estop();
    System.exit(0);
  }

  /**
   * <p>The estop button.</p>
   **/
  protected static class EstopButton extends JDialog {
    
    EstopButton(final Thread mainThread) {

      JButton eb = new JButton("ESTOP");
      eb.setBackground(Color.RED);
      eb.setPreferredSize(new Dimension(200, 200));
      eb.addActionListener(new ActionListener() {
          public void actionPerformed(ActionEvent e) {
            mainThread.interrupt();
            robot.estop();
            System.exit(-1);
          }
        });

      setContentPane(eb);
      setTitle("Emergency Stop");
      setDefaultCloseOperation(JDialog.DO_NOTHING_ON_CLOSE);
      pack();
      setVisible(true);
    }
  }
} 
