package edu.uidaho.phantomlord.Sprites.Objects;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.audio.Sound;
import com.badlogic.gdx.maps.MapObject;
import com.badlogic.gdx.math.Rectangle;

import edu.uidaho.phantomlord.PhantomLord;
import edu.uidaho.phantomlord.Screens.PLordPlayScreen;
import edu.uidaho.phantomlord.Sprites.PhantomLordChar;

public class Object extends TileObject
{
	/* 
	* Idea from
	* http://www.gamefromscratch.com/post/2014/09/25/LibGDX-LibGDX-Tutorial-13-Physics-with-Box2D-Part-4-Controlling-collisions-using-filters.aspx
	*/ 
	public Object(PLordPlayScreen screen, MapObject object, Boolean sensor)
	{
		// Super calls parent constructor, thus when we create a Candle class instance
		// it will pass it parameters to the InteractiveTileObject to create an 
		// instance of the object specifically for this class
		super(screen, object, sensor);
		
		// Set user data to object itself
		fixture.setUserData(this);
		
		// Set category filter for 'Object' Fixture
		setCategoryFilter(PhantomLord.OBJECT_BIT);
	}
	
	/*
	 * Idea for Plord Colliison from
	 * http://www.hobbygamedev.com/adv/2d-platformer-advanced-collision-detection/
	 */
	@Override
	public void onPLordHeadHit(PhantomLordChar pLord) 
	{
		//
		// Want to play "dud" sound when we hit an Object (if on the head)
		//
		
		// Add sound file for when hitting the 'Stone' from AssetManager and then play it (in same line, since we don't need to loop it)
		PhantomLord.manager.get("assets/audio/sounds/StoneBump.mp3", Sound.class).play();
	}

}
