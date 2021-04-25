package edu.uidaho.phantomlord.Sprites.Objects;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.audio.Sound;
import com.badlogic.gdx.maps.MapObject;
import com.badlogic.gdx.maps.tiled.TiledMap;
import com.badlogic.gdx.math.Rectangle;
import com.badlogic.gdx.physics.box2d.World;

import edu.uidaho.phantomlord.PhantomLord;
import edu.uidaho.phantomlord.Scenes.PLordHud;
import edu.uidaho.phantomlord.Screens.PLordPlayScreen;
import edu.uidaho.phantomlord.Sprites.PhantomLordChar;

public class CrumbleStone extends TileObject
{
	/* 
	* Idea from
	* http://www.gamefromscratch.com/post/2014/09/25/LibGDX-LibGDX-Tutorial-13-Physics-with-Box2D-Part-4-Controlling-collisions-using-filters.aspx
	*/ 
	public CrumbleStone(PLordPlayScreen screen, MapObject object, Boolean sensor)
	{
		// Super calls parent constructor, thus when we create a Candle class instance
		// it will pass it parameters to the InteractiveTileObject to create an 
		// instance of the object specifically for this class
		super(screen, object, sensor);
		
		// Set user data to object itself
		fixture.setUserData(this);
		
		// Set category filter for Candle Fixture
		setCategoryFilter(PhantomLord.OBJECT_BIT);
	}
	
	/*
	 * Idea for Plord Colliison from
	 * http://www.hobbygamedev.com/adv/2d-platformer-advanced-collision-detection/
	 * 
	 * and Idea for finding cell in Tiled from 
	 * https://stackoverflow.com/questions/27790191/how-to-get-the-position-of-a-tiledmap-cell-in-libgdx
	 */
	@Override
	public void onPLordHeadHit(PhantomLordChar pLord) 
	{
		// Only averagePLord or powerUpPLord can break CStone, check if current PLord is 'average' or powered Up
		if(pLord.isPLordAverage() || pLord.isPLordPower())
		{
			// When head hit on object set category filter to destroyed_bit to 'destroy' the CStone and not collide with it
			setCategoryFilter(PhantomLord.DESTROYED_BIT);
			
			//Add score when we break the crumbleStone
			PLordHud.addScore(100);
			
			// DONT NEED TO DO ABSOLUTE PATH "Gdx.files.internal("assets/stuff...").file().getAbsolutePath()" for AssetManager.get()
			// Add sound file for when hitting the CrumbleStone from AssetManager and then play it (in same line, since we don't need to loop it)
			PhantomLord.manager.get("assets/audio/sounds/SmashCStone.mp3", Sound.class).play();
		
			// Get cell from Tiled map, where the object we hit was and remove it from graphics
			// Since we want to "destroy" the crumbleStone when PLord hits it with his head
			getCell().setTile(null);
		}
		else
		{
			// Else Want to play "dud" sound when we hit an it, since PLord is not average
			
			// Add sound file for when hitting the 'Stone' from AssetManager and then play it (in same line, since we don't need to loop it)
			PhantomLord.manager.get("assets/audio/sounds/StoneBump.mp3", Sound.class).play();
		}
		
	}

}
