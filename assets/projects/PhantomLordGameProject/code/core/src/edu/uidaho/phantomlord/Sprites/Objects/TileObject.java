package edu.uidaho.phantomlord.Sprites.Objects;

import com.badlogic.gdx.maps.MapObject;
import com.badlogic.gdx.maps.tiled.TiledMap;
import com.badlogic.gdx.maps.tiled.TiledMapTile;
import com.badlogic.gdx.maps.tiled.TiledMapTileLayer;
import com.badlogic.gdx.math.Rectangle;
import com.badlogic.gdx.maps.objects.RectangleMapObject;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.BodyDef;
import com.badlogic.gdx.physics.box2d.Filter;
import com.badlogic.gdx.physics.box2d.Fixture;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.badlogic.gdx.physics.box2d.PolygonShape;
import com.badlogic.gdx.physics.box2d.World;

import edu.uidaho.phantomlord.PhantomLord;
import edu.uidaho.phantomlord.Screens.PLordPlayScreen;
import edu.uidaho.phantomlord.Sprites.PhantomLordChar;

//Make an abstract class, not planning on making any instances of it, 'object' classes with 'super' to this
public abstract class TileObject
{
	// Can be accessed by subclasses (i.e. our tile classes)
	protected World world;
	protected TiledMap map;
	protected TiledMapTile tile;
	protected Rectangle bounds;
	protected Body box2DBody;
	protected Fixture fixture;
	protected MapObject object;
	
	// Gives access to our tiles the PlayScreen
	protected PLordPlayScreen screen;
	
	/*
	* Idea from
	* https://gamedev.stackexchange.com/questions/101127/libgdx-using-box2d-with-tiledmap-to-create-collision-detection
	* &
	* https://gamedev.stackexchange.com/questions/124210/check-collision-between-two-rectangles-libgdx
	* &
	* http://www.iforce2d.net/b2dtut/bodies
	* &
	* http://www.gamefromscratch.com/post/2014/08/27/LibGDX-Tutorial-13-Physics-with-Box2D-Part-1-A-Basic-Physics-Simulations.aspx
	*/
	// Constructor
	public TileObject(PLordPlayScreen screen, MapObject object, Boolean sensor)
	{
		this.screen = screen;
		this.object = object;
		this.world = screen.getWorld();
		this.map = screen.getMap();
		// object needs to be be cast to RectangleMapObject to get bounds/rectangle
		this.bounds = ((RectangleMapObject) object).getRectangle();
		
		/* **************************************
		 * Create local variables for box2d
		 ***************************************/
		BodyDef bDef = new BodyDef();
		FixtureDef fDef = new FixtureDef();
		PolygonShape shape = new PolygonShape();
		
		// If the tile is not supposed to have physics collision (just sensor detection)
		if (sensor)
		{
			fDef.isSensor = true;
		}
		
		bDef.type = BodyDef.BodyType.StaticBody;
		// Set position of current rect object, getX starts at bottom left of rectangle, thus
		// We need half the width to find the 'center' of the rectangle position, and a similar
		// idea follows for getY.
		bDef.position.set((bounds.getX() + bounds.getWidth() / 2) / PhantomLord.PIXELS_PER_METER, (bounds.getY() + bounds.getHeight() / 2) / PhantomLord.PIXELS_PER_METER );
		
		// Add body to box2d World variable
		box2DBody = world.createBody(bDef);
		
		/********************
		 * Create fixture
		 ********************/
		// Define polygon shape itself (divide by 2, as setAsBox starts in center and moves
		// vertically and horizontally in both LR, and Up Down at same time, son only need
		// half the distance
		shape.setAsBox(bounds.getWidth() / 2 / PhantomLord.PIXELS_PER_METER, bounds.getHeight() / 2 / PhantomLord.PIXELS_PER_METER);
		// Set fixture shape to shape we just created
		fDef.shape = shape;
		
		// Add fixture to body
		// When we create the fixture that surrounds the stone, candle, etc., capture that in a
		// variable called fixture
		fixture = box2DBody.createFixture(fDef);
	}
	
	// Classes that extend this InteractiveTileObject class with implement this method
	public abstract void onPLordHeadHit(PhantomLordChar pLord);
	
	public void setCategoryFilter(short filterBit)
	{
		// How to make/set category filter if you don't have access to fixture definition itself
		Filter filter = new Filter();
		filter.categoryBits = filterBit;
		fixture.setFilterData(filter);
	}
	
	public TiledMapTileLayer.Cell getCell()
	{
		// Get layer that cells are on, the "graphics cell", as index 1 (0 is background , graphics is 1, ground is 2 and so on)
		TiledMapTileLayer layer = (TiledMapTileLayer) map.getLayers().get(1);
		
		// Remember to scale position up (we previously scaled it down, the PIXELS_PER_METER)
		// x coordinate of cell in Tiled: Position of x of body object * PhantomLord.Pixels_Per_Meter / (tile size from tiled, which was 16 here)
		// y coordinate of cell in Tiled: Position of y of body object * PhantomLord.Pixels_Per_Meter / (tile size from tiled, which was 16 here)
		return layer.getCell((int)(box2DBody.getPosition().x * PhantomLord.PIXELS_PER_METER / 16), 
				(int)(box2DBody.getPosition().y * PhantomLord.PIXELS_PER_METER / 16));
	}
	
}
