package edu.wpi.first.wpilibj.drive;

import edu.wpi.first.wpilibj.SpeedController;

public interface ISpeedControllerFactory {
	public SpeedController getController(int id);
	public SpeedController getController(int port, int id);
	public SpeedController getController(SpeedController defaultSpeeedController, SpeedController ...);
	public boolean supportsIDInput();
	public boolean supportsControllerInput();
}