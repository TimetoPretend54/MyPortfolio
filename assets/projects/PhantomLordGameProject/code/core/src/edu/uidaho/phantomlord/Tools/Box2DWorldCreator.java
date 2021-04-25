package edu.uidaho.phantomlord.Tools;

import com.badlogic.gdx.maps.MapObject;
import com.badlogic.gdx.maps.objects.RectangleMapObject;
import com.badlogic.gdx.maps.tiled.TiledMap;
import com.badlogic.gdx.math.Rectangle;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.BodyDef;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.badlogic.gdx.physics.box2d.PolygonShape;
import com.badlogic.gdx.physics.box2d.World;
import com.badlogic.gdx.utils.Array;

import edu.uidaho.phantomlord.PhantomLord;
import edu.uidaho.phantomlord.Events.Goal;
import edu.uidaho.phantomlord.Screens.PLordPlayScreen;
import edu.uidaho.phantomlord.Sprites.Enemies.Bat;
import edu.uidaho.phantomlord.Sprites.Enemies.Enemy;
import edu.uidaho.phantomlord.Sprites.Enemies.HatGhost;
import edu.uidaho.phantomlord.Sprites.Enemies.Pumpkin;
import edu.uidaho.phantomlord.Sprites.Items.Candle;
import edu.uidaho.phantomlord.Sprites.Items.Item;
import edu.uidaho.phantomlord.Sprites.Objects.CandleBlock;
import edu.uidaho.phantomlord.Sprites.Objects.CrumbleStone;
import edu.uidaho.phantomlord.Sprites.Objects.Object;
import edu.uidaho.phantomlord.Sprites.Objects.VegetationOnCStone;

/*
* Idea to convert Tield map objects to box2D world objects
* https://gamedev.stackexchange.com/questions/66924/how-can-i-convert-a-tilemap-to-a-box2d-world
* &
* https://github.com/libgdx/libgdx/wiki/Tile-maps
* &
* https://gamedev.stackexchange.com/questions/103710/static-spawning-locations-for-players-creatures-items-in-tiled-map-editor
*/

public class Box2DWorldCreator 
{
	// Private arrays for enemies (idea from http://badlogicgames.com/forum/viewtopic.php?f=11&t=4510)
	private static Array<Pumpkin> pumpkins;
	private static Array<HatGhost> hatGhosts;
	private static Array<Bat> bats;
	private static Array<Candle> candles;
	
	public Box2DWorldCreator(PLordPlayScreen screen)
	{
		World world = screen.getWorld();
		TiledMap map = screen.getMap();
		/* **************************************
		 * Create local variables for box2d
		 ***************************************/
		// define body (what it consists of)
		BodyDef  bDef = new BodyDef();
		// Polygon shape for 'fixture'
		PolygonShape shape = new PolygonShape();
		// Fixture (define fixture first, and then add it to body)
		FixtureDef fDef = new FixtureDef();
		// Create the body itself
		Body body;
		
		/*
		* Code for looping though Tiled object modified from
		* https://gamedev.stackexchange.com/questions/66924/how-can-i-convert-a-tilemap-to-a-box2d-world
		*/
		/*********************************************************************************
		*  Create bodies and fixtures in every corresponding 'object' in tiled map layers
		*  Skip the tile layers, which correspond to indexes 0 and 1, 
		**********************************************************************************/
		
		//so start at 2 for 'ground' object layer in  ".get(2).getObjects()..."
		for (MapObject object : map.getLayers().get(2).getObjects().getByType(RectangleMapObject.class))
		{
			// Typecast rectangle object to rectangle map object
			Rectangle rect = ((RectangleMapObject) object).getRectangle();
			
			// Three type of bodies in box2d (dynamic body, kinematic body, and static body)
			// Dynamic = affected by forces, gravity, velocity, etc.
			// Static = does not move, not affected by forces in physics, must move them by
			//		programming
			// Kinematic = aren't affect by forces like gravity, but can be manipulated by 
			//		velocity
			bDef.type = BodyDef.BodyType.StaticBody;
			// Set position of current rect object, getX starts at bottom left of rectangle, thus
			// We need half the width to find the 'center' of the rectangle position, and a similar
			// idea follows for getY.
			bDef.position.set((rect.getX() + rect.getWidth() / 2) / PhantomLord.PIXELS_PER_METER, (rect.getY() + rect.getHeight() / 2) / PhantomLord.PIXELS_PER_METER );
			
			// Add body to box2d World variable
			body = world.createBody(bDef);
			
			/********************
			 * Create fixture
			 ********************/
			// Define polygon shape itself (divide by 2, as setAsBox starts in center and moves
			// vertically and horizontally in both LR, and Up Down at same time, son only need
			// half the distance

			shape.setAsBox(rect.getWidth() / 2 / PhantomLord.PIXELS_PER_METER, rect.getHeight() / 2 / PhantomLord.PIXELS_PER_METER);
			// Set fixture shape to shape we just created
			fDef.shape = shape;
			// Set category bit for fixture
			fDef.filter.categoryBits = PhantomLord.OBJECT_BIT;
			//Add fixture to body
			body.createFixture(fDef);
		}
		
		// Start at index 3 at "get(3)" for 'crumbleStone' object layer (From bottom up in Tiled,
		// background was 0, graphics was 1, ground was 2, crumbleStone is 3 and so on...
		for (MapObject object : map.getLayers().get(3).getObjects().getByType(RectangleMapObject.class))
		{
			// Add object itself to crumbleStone class
			// Create 'crumbleStone' bodies and fixtures, false means we are not a 'sensor'
			new CrumbleStone(screen, object, false);
		}
		
		// Start at index 4 at "get(4)" for 'stone' object layer
		for (MapObject object : map.getLayers().get(4).getObjects().getByType(RectangleMapObject.class))
		{
			// Add object itself to  to Object Class
			// Create 'stone' bodies and fixtures, false means we are not a 'sensor'
			// Use "new Object" class because we don't have any functionality of object besides making
			// a sound when(or if) PLord Hits it with his head and providing collision
			new Object(screen, object, false);
		}
		
		// Start at index 5 at "get(5)" for 'candleBlock' object layer
		for (MapObject object : map.getLayers().get(5).getObjects().getByType(RectangleMapObject.class))
		{
			// Add object itself to candleBlock class
			// Create 'candleBlock' bodies and fixtures, false means we are not a 'sensor'
			new CandleBlock(screen, object, false);
		}
		
		// Start at index 6 at "get(6)" for 'topOfArchitecture' object layer
		for (MapObject object : map.getLayers().get(6).getObjects().getByType(RectangleMapObject.class))
		{
			// Add object itself to Object Class
			// Create 'object' bodies and fixtures, false means we are not a'sensor'
			// Use 'Object' because we don't have any functionality of object besides making
			// a sound when(or if) PLord Hits it with his head and providing collision
			new Object(screen, object, false);
		}
		
		// Start at index 7 at "get(7)" for 'vegetationOnCStone' object layer
		for (MapObject object : map.getLayers().get(7).getObjects().getByType(RectangleMapObject.class))
		{
			// Add object itself to Object class
			// Create 'vegetationOnCStone' bodies and fixtures, true means we are a 'sensor'
			new VegetationOnCStone(screen, object, true);
		}
		
		// Create all Pumpkins, start at index 8, or 'pumpkinEnemy' object layer
		pumpkins = new Array<Pumpkin>();
		for (MapObject object : map.getLayers().get(8).getObjects().getByType(RectangleMapObject.class))
		{
			// Typecast rectangle object to rectangle map object
			Rectangle rect = ((RectangleMapObject) object).getRectangle();
			
			// Create pumpkins, we get the x and y coordinates from reading in the MapObject object, setting each item in the object layer
			// to a rectangle object and then using rect.getX and rect.getY(), that gives us those x and y coordinate, BUT WE MUST scale it!
			// Also added +10 to vertical position because pumpkin seemed stuck in the ground a bit, and sometimes got stuck 
			pumpkins.add(new Pumpkin(screen, rect.getX() / PhantomLord.PIXELS_PER_METER, rect.getY() / PhantomLord.PIXELS_PER_METER + 10 / PhantomLord.PIXELS_PER_METER));
		}
		
		// Create all HatGhosts, start at index 9, or 'hatGhostEnemy' object layer
		hatGhosts = new Array<HatGhost>();
		for (MapObject object : map.getLayers().get(9).getObjects().getByType(RectangleMapObject.class))
		{
			// Typecast rectangle object to rectangle map object
			Rectangle rect = ((RectangleMapObject) object).getRectangle();
			
			// Create HatGhosts, we get the x and y coordinates from reading in the MapObject object, setting each item in the object layer
			// to a rectangle object and then using rect.getX and rect.getY(), that gives us those x and y coordinate, BUT WE MUST scale it!
			// Also added +10 to vertical position because pumpkin seemed stuck in the ground a bit, and sometimes got stuck 
			hatGhosts.add(new HatGhost(screen, rect.getX() / PhantomLord.PIXELS_PER_METER, rect.getY() / PhantomLord.PIXELS_PER_METER + 10 / PhantomLord.PIXELS_PER_METER));
		}
		
		// Create all bats, start at index 10, or 'batEnemy' object layer
		bats = new Array<Bat>();
		for (MapObject object : map.getLayers().get(10).getObjects().getByType(RectangleMapObject.class))
		{
			// Typecast rectangle object to rectangle map object
			Rectangle rect = ((RectangleMapObject) object).getRectangle();
			
			// Create pumpkins, we get the x and y coordinates from reading in the MapObject object, setting each item in the object layer
			// to a rectangle object and then using rect.getX and rect.getY(), that gives us those x and y coordinate, BUT WE MUST scale it!
			// Also added +10 to vertical position because pumpkin seemed stuck in the ground a bit, and sometimes got stuck 
			bats.add(new Bat(screen, rect.getX() / PhantomLord.PIXELS_PER_METER + 7 / PhantomLord.PIXELS_PER_METER, rect.getY() / PhantomLord.PIXELS_PER_METER + 10 / PhantomLord.PIXELS_PER_METER));
		}
		
		// start at 10 for 'finishGoal' object layer in  ".get(11).getObjects()..."
		for (MapObject object : map.getLayers().get(11).getObjects().getByType(RectangleMapObject.class))
		{
			// Add object itself to Goal class
			// Create 'Goal' bodies and fixtures, true means we are a sensor
			new Goal(screen, object, true);
		}
		
		// Create all Candles, start at index 12, or 'pumpkinEnemy' object layer
		candles = new Array<Candle>();
		for (MapObject object : map.getLayers().get(12).getObjects().getByType(RectangleMapObject.class))
		{
			// Typecast rectangle object to rectangle map object
			Rectangle rect = ((RectangleMapObject) object).getRectangle();
			
			// Create pumpkins, we get the x and y coordinates from reading in the MapObject object, setting each item in the object layer
			// to a rectangle object and then using rect.getX and rect.getY(), that gives us those x and y coordinate, BUT WE MUST scale it!
			// Also added +10 to vertical position because pumpkin seemed stuck in the ground a bit, and sometimes got stuck 
			candles.add(new Candle(screen, rect.getX() / PhantomLord.PIXELS_PER_METER + 8.5f / PhantomLord.PIXELS_PER_METER, rect.getY() / PhantomLord.PIXELS_PER_METER + 10 / PhantomLord.PIXELS_PER_METER));
		}
	}
	
	public static void removeHatGhost(HatGhost hatGhost)
	{
		hatGhosts.removeValue(hatGhost, true);
	}
	
	public static void removePumpkin(Pumpkin pumpkin)
	{
		pumpkins.removeValue(pumpkin, true);
	}
	
	public static void removeBat(Bat bat)
	{
		bats.removeValue(bat, true);
	}
	
	public static void removeCandle(Candle candle)
	{
		candles.removeValue(candle, true);
	}
	
	// return all enemies, not just a type
	public Array<Enemy> getEnemies()
	{
		// Create an array of enemies
		Array<Enemy> enemies = new Array<Enemy>();
		
		// Add the pumpkins to the enemy array
		enemies.addAll(pumpkins);
		
		// Add the hatGhosts to the enemy array
		enemies.addAll(hatGhosts);
		
		// Add the bats to the enemy array
		enemies.addAll(bats);
		
		// Return the enemies array
		return enemies;
	}
	
	public Array<Item> getItems()
	{
		Array<Item> items = new Array<Item>();
		
		items.addAll(candles);
		
		return items;
	}
}
