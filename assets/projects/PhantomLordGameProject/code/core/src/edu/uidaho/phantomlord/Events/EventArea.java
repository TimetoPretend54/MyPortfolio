package edu.uidaho.phantomlord.Events;

import com.badlogic.gdx.audio.Sound;
import com.badlogic.gdx.maps.MapObject;
import com.badlogic.gdx.maps.objects.RectangleMapObject;
import com.badlogic.gdx.maps.tiled.TiledMap;
import com.badlogic.gdx.maps.tiled.TiledMapTile;
import com.badlogic.gdx.math.Rectangle;
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
import edu.uidaho.phantomlord.Sprites.Objects.TileObject;

public class EventArea
{
	protected World world;
	protected TiledMap map;
	protected TiledMapTile tile;
	protected Rectangle bounds;
	protected Body box2DBody;
	protected Fixture fixture;
	protected MapObject object;

	// Gives access to our tiles the PlayScreen
	protected PLordPlayScreen screen;
	
	/* Copied from my "TileObject Class"
	* WHich I found idea from
	* https://gamedev.stackexchange.com/questions/101127/libgdx-using-box2d-with-tiledmap-to-create-collision-detection
	*/
	public EventArea(PLordPlayScreen screen, MapObject object, Boolean sensor)
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

	public void setCategoryFilter(short filterBit)
	{
		// How to make/set category filter if you don't have access to fixture definition itself
		Filter filter = new Filter();
		filter.categoryBits = filterBit;
		fixture.setFilterData(filter);
	}
}

