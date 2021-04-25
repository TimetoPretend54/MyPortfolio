package edu.uidaho.phantomlord.Events;

import com.badlogic.gdx.audio.Sound;
import com.badlogic.gdx.maps.MapObject;
import com.badlogic.gdx.math.Vector2;

import edu.uidaho.phantomlord.PhantomLord;
import edu.uidaho.phantomlord.Scenes.PLordHud;
import edu.uidaho.phantomlord.Screens.PLordPlayScreen;
import edu.uidaho.phantomlord.Sprites.PhantomLordChar;
import edu.uidaho.phantomlord.Sprites.Items.HatCandy;
import edu.uidaho.phantomlord.Sprites.Items.ItemDefinition;

public class Goal extends EventArea
{
	/* 
	* Idea from
	* http://www.gamefromscratch.com/post/2014/09/25/LibGDX-LibGDX-Tutorial-13-Physics-with-Box2D-Part-4-Controlling-collisions-using-filters.aspx
	*/ 
	public Goal(PLordPlayScreen screen, MapObject object, Boolean sensor) 
	{
		super(screen, object, sensor);
		
		// Set user data to object itself
		fixture.setUserData(this);
		
		// Set category filter for 'Object' Fixture
		setCategoryFilter(PhantomLord.EVENT_BIT);
	}
	
	public void goalReached(PhantomLordChar pLord)
	{
		// Run the function to have pLord run to end of stage and finish level
		pLord.pLordFinishedLevel(object);
	}

}