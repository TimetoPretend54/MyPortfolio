package edu.uidaho.phantomlord.Sprites.Enemies;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g2d.Sprite;
import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.World;

import edu.uidaho.phantomlord.Screens.PLordPlayScreen;
import edu.uidaho.phantomlord.Sprites.PhantomLordChar;

/*
 * Code modified from
 * https://gamedev.stackexchange.com/questions/116127/my-characters-do-not-reverse-their-velocity-why-libgdxhttps://gamedev.stackexchange.com/questions/116127/my-characters-do-not-reverse-their-velocity-why-libgdx
 */
// Make an abstract class, not planning on making any instances of it, 'enemy' classes with 'super' to this
public abstract class Enemy extends Sprite
{
	protected World world;
	protected PLordPlayScreen screen;
	
	// Each enemy will need a box2D Body
	public Body box2DBody;
	
	// Create Vector2 variable for Pumpkin Velocity
	public Vector2 velocity;
	
	public Enemy(PLordPlayScreen screen, float x, float y)
	{
		this.world = screen.getWorld();
		this.screen = screen;
		
		// Set position of sprite of enemy
		setPosition(x, y);
		defineEnemy();
		
		// Define Enemy Velocity (1 in x axis, -1.7 in y axis, so they can fall if need be)
		velocity = new Vector2(1f, -1.7f);
		
		// By default, out box2DBody's setActive is set to false, so to begin with, enemy doesn't move
		box2DBody.setActive(false); 				// Puts box2DBody to sleep, doesn't allow it to be 
													// calculated in simulation until woken up
	}
	
	// Each enemy has to define itself in the box2D World
	protected abstract void defineEnemy();
	
	// Every enemy has too have a hitOnHead function, public so we can access it from PLordWorldCOntactListener 
	public abstract void hitOnHead(PhantomLordChar pLord);
	
	// Update our box2DBody position and animation frame for Enemy (so all enemies must have an update method)
	public abstract void update(float dt, PLordPlayScreen screen);
	
	// Determine what to do if two enemies hit, depending on who they are
	public abstract void onEnemyHit(Enemy enemy);
	
	public abstract void killEnemy();
	
	// If this method is called it will check to check if either boolean is true, and
	// if so reverse the velocity in that axis related to boolean
	public void reverseVelocity(boolean x, boolean y)
	{
		if(x)
			velocity.x = -velocity.x;
		if(y)
			velocity.y = -velocity.y;
	}
	
	public void fallVeloity()
	{
		velocity.y = -10;
	}
}
