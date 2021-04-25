package edu.uidaho.phantomlord.desktop;

import com.badlogic.gdx.backends.lwjgl.LwjglApplication;
import com.badlogic.gdx.backends.lwjgl.LwjglApplicationConfiguration;

import edu.uidaho.phantomlord.PhantomLord;

public class DesktopLauncher 
{
	public static void main (String[] arg)
	{
		LwjglApplicationConfiguration config = new LwjglApplicationConfiguration();
		config.width = 1200;
		config.height = 624;
		// Force exit when exiting the game
		config.forceExit = true;
		new LwjglApplication(new PhantomLord(), config);
	}
}
