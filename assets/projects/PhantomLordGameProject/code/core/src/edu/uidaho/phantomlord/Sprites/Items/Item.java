package edu.uidaho.phantomlord.Sprites.Items;

import com.badlogic.gdx.graphics.g2d.Batch;
import com.badlogic.gdx.graphics.g2d.Sprite;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.World;

import edu.uidaho.phantomlord.PhantomLord;
import edu.uidaho.phantomlord.Screens.PLordPlayScreen;
import edu.uidaho.phantomlord.Sprites.PhantomLordChar;

/*
 * code modified/idea from
 * https://stackoverflow.com/questions/28812776/game-character-items-libgdx/28986192
 */

public abstract  class Item extends Sprite
{
	// Variables
	protected PLordPlayScreen screen;
	protected World world;
	protected Vector2 velocity;
	protected boolean toDestroy;
	protected boolean destroyed;
	protected Body box2DBody;
	
	// Constructor
	public Item(PLordPlayScreen screen, float x, float y)
	{
		this.screen = screen;
		this.world = screen.getWorld();
		
		// Set position of sprite 
		setPosition(x, y);
		
		// Position and size of Sprite 25x25 seems like good size
		setBounds(getX(), getY(), 25 / PhantomLord.PIXELS_PER_METER, 25 / PhantomLord.PIXELS_PER_METER);
		
		// Call define item function
		defineItem();
		
		// Item should not be initially destroyed
		toDestroy = false;
		destroyed = false;
	}
	
	// Each item has to be able to define itself in box
	public abstract void defineItem();
	
	// Use the specified item
	public abstract void use(PhantomLordChar pLord);
	
	// All items need to update themselves
	public void update(float dt)
	{
		// if we need to destroy item
		if(toDestroy && !destroyed)
		{
			world.destroyBody(box2DBody);
			destroyed = true;
		}
	}
	
	public void draw(Batch batch)
	{
		// only draw item if its not destoryed
		if (!destroyed)
		{
			super.draw(batch);
		}
	}
	
	// Need this because we cant destroy box2D object inside of world.step simulator
	public void destroy()
	{
		toDestroy = true;
	}
	
    public boolean isDestroyed() {
        return destroyed;
    }
	
	
	// Every item has ability to reverse velocity
	public void reverseVelocity(boolean x, boolean y)
	{
		if(x)
			velocity.x = -velocity.x;
		if(y)
			velocity.y = -velocity.y;
	}
}
