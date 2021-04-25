package edu.uidaho.phantomlord.Sprites.Enemies;

import com.badlogic.gdx.audio.Sound;
import com.badlogic.gdx.graphics.g2d.Animation;
import com.badlogic.gdx.graphics.g2d.Batch;
import com.badlogic.gdx.graphics.g2d.TextureRegion;
import com.badlogic.gdx.math.MathUtils;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.BodyDef;
import com.badlogic.gdx.physics.box2d.CircleShape;
import com.badlogic.gdx.physics.box2d.EdgeShape;
import com.badlogic.gdx.physics.box2d.Filter;
import com.badlogic.gdx.physics.box2d.Fixture;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.badlogic.gdx.physics.box2d.PolygonShape;
import com.badlogic.gdx.utils.Array;

import edu.uidaho.phantomlord.PhantomLord;
import edu.uidaho.phantomlord.Scenes.PLordHud;
import edu.uidaho.phantomlord.Screens.PLordPlayScreen;
import edu.uidaho.phantomlord.Sprites.PhantomLordChar;
import edu.uidaho.phantomlord.Tools.Box2DWorldCreator;

public class HatGhost extends Enemy
{
	// Variables for the speed of kicking hat of ghost
	public static final int HIT_LEFT_SPEED = -2;
	public static final int HIT_RIGHT_SPEED = 2;
	
	// HatGhost can have more than one state
	public enum State{MOVING, STATIC_HIDEINHAT, MOVING_HIDEINHAT, DEAD}
	public State currentState;
	public State previousState;
	// Keep track of time we are in any given state
	private float stateTimer;
	// Animations
	private Animation<TextureRegion> floatAnimation;
	private Animation<TextureRegion> hideInHatStaticAnimation;
	private Animation<TextureRegion> hideInHatMovingAnimation;
	// Frames array to hold the animation
	private Array<TextureRegion> frames;
	// Keep track of the rotation of HatGhost, so we know how many little summersaults he need to do when he dies
	private float killRotationDegrees;
	// Boolean variable to determine if we need to destroy HatGhost box2DBody outside of the simulation
	private boolean setToDestroy;
	private boolean destroyed;
	
	public HatGhost(PLordPlayScreen screen, float x, float y) 
	{
		super(screen, x, y);
		
		/*
		 * Animation idea/structure from
		 * https://github.com/libgdx/libgdx/wiki/2D-Animation
		 */
		
		// Create an array of texture regions to pass the constructor for the animation
		//Array<TextureRegion> frames = new Array<TextureRegion>();
		frames = new Array<TextureRegion>();
		
		/********************************************************
		 *  Initialize Move Animation for HatGhost
		 ********************************************************/
		// Bounce is at animation indexes 0-2 for "HatGhost" at row 1 (row index 0) this index# and row# is given by "GhostWithHat" texture region, which means we only focus on coordinates of
		// "GhostWithHat" region, thus the top left of just that spriteSheet will be (0,0), image that we are looking at the "GhostWithHat.png" from "AllSpriteSheets (For libGDX Texture Packer)" folder
		// And getting the coordinate from there, which we can use a for loop to quickly navigate though and get sprite frame, since each sprite is 35x35 
		for (int i = 0; i < 3; i++)
			frames.add(new TextureRegion(screen.getAtlas().findRegion("GhostWithHat"), i*35, 0 * 37, 35, 35));
		floatAnimation = new Animation<TextureRegion>(0.2f, frames);					// Might need higher or lower time (1st parameter, depending on animation)
		frames.clear();
		
		/********************************************************
		 *  Initialize HatHideStatic Animation for HatGhost
		 ********************************************************/
		// HatHide animations at indexes 5-6 for GhostWithHat texture region
		for (int i = 5; i < 7; i++)
			frames.add(new TextureRegion(screen.getAtlas().findRegion("GhostWithHat"), i*35, 0 * 37, 35, 35));
		hideInHatStaticAnimation = new Animation<TextureRegion>(0.4f, frames);					// Might need higher or lower time (1st parameter, depending on animation)
		frames.clear();
		
		/********************************************************
		 *  Initialize HatHideMoving Animation for HatGhost
		 ********************************************************/
		// HatHide animations at indexes e3-4 for GhostWithHat texture region
		for (int i = 3; i < 5; i++)
			frames.add(new TextureRegion(screen.getAtlas().findRegion("GhostWithHat"), i*35, 0 * 37, 35, 35));
		hideInHatMovingAnimation = new Animation<TextureRegion>(0.4f, frames);					// Might need higher or lower time (1st parameter, depending on animation)
		frames.clear();
		
		// Declare States
		currentState = previousState = State.MOVING;
		
		// Set the initial rotation degrees
		killRotationDegrees = 0;
		
		// Set size and initial position of Pumpkin Sprite, 27X27 seems like good size for HatGhost (both have to be divided by our Pixels_per_meter 1st)
		setBounds(getX(), getY(), 27 / PhantomLord.PIXELS_PER_METER, 27 / PhantomLord.PIXELS_PER_METER);
	
		// Assume initially that Pumpkin is not destroyed and not about to be destroyed
		setToDestroy = false;
		destroyed = false;
	}
	
	/*
	* statring code/idea from
	* http://www.iforce2d.net/b2dtut/bodies
	* &
	* http://www.gamefromscratch.com/post/2014/08/27/LibGDX-Tutorial-13-Physics-with-Box2D-Part-1-A-Basic-Physics-Simulations.aspx
	*/
	@Override
	protected void defineEnemy() 
	{
		// Define body for HatGhost and assign body to world
		BodyDef bDef = new BodyDef();
		bDef.position.set(getX(), getY());																// Box2D Body is at position of created Pumpkin Sprite, after this the
		bDef.type = BodyDef.BodyType.DynamicBody;														// sprite/animation stays in line with the actual physics collider Also, 
		box2DBody = world.createBody(bDef);																// when we Pumpkin moves, the box2DBody gets moved to correspond to that,
																										// so the Spite needs to be rendered wherever the box2DBody is
		// Define fixture for HatGhost (make radius appropriate size that matches the HatGhost sprite)
		FixtureDef fDef = new FixtureDef();
		CircleShape shape = new CircleShape();
		shape.setRadius(6 / PhantomLord.PIXELS_PER_METER);
		
		// Category for HatGhost, aka, what is this fixture's category
		fDef.filter.categoryBits = PhantomLord.ENEMY_BIT;
		// What can Enemy collide with
		fDef.filter.maskBits = PhantomLord.OBJECT_BIT | PhantomLord.PLORD_BIT | PhantomLord.ENEMY_BIT | PhantomLord.WEAPON_BIT;
		// Set this fixture to our body for Pumpkin and setUserData for WorldContactLister
		fDef.shape = shape;
		box2DBody.createFixture(fDef).setUserData(this);
		
		// Create the Head for Pumpkin
		// PolygonShape takes in a list of Vector2 vertices
		PolygonShape head = new PolygonShape();
		// Create the variable to hold list of Vector2 Vertices
		Vector2[] vertice = new Vector2[4];
		// Create Vector2 Vertices
		// Position of vertice related to origin (middle) of box2DBody (radius is 6, so this will go 2 pixel above it, since y = 7)
		vertice[0] = new Vector2(-5, 7).scl(1/PhantomLord.PIXELS_PER_METER);
		vertice[1] = new Vector2(5, 7).scl(1/PhantomLord.PIXELS_PER_METER);
		vertice[2] = new Vector2(-3, 3).scl(1/PhantomLord.PIXELS_PER_METER);
		vertice[3] = new Vector2(3, 3).scl(1/PhantomLord.PIXELS_PER_METER);
		// Set our Vector2 Vertices to that Polygon shape
		head.set(vertice);
		// Set our shape using prior fDef
		fDef.shape = head;
		// Add Bounciness, so when PLord hits head the HatGhost, he bounces up a bit
		// This is done by setting the restitution of the fixture
		fDef.restitution = 0.9f;			// Half bounciness, so if was 10 pixels from top of head, then hit, PLord will bounce 5 pixels off
		// Set filter category for fixture of enemy head
		fDef.filter.categoryBits = PhantomLord.ENEMYHEAD_BIT;
		// Create the Head fixture and use setUsedData to have access to this class from collision handler
		box2DBody.createFixture(fDef).setUserData(this);
		
		// Fix issue where HatGhost will respond to any collision of the ground beneath it, only care about the ground that
		// relate to walls, so create 'Feet' for HatGhost so it doesn't ever touch the bottom part of ground
		FixtureDef fDef2 = new FixtureDef();
		EdgeShape feet = new EdgeShape();
		// Coordinates relative to GhostHat body, edgeShape acts as feet to glide across tiles (since radius of HatGhost is 6, go -7 below the
		// center, and the 'feet' will be exactly 1 pixels below the body of HatGhost, which seems good
		feet.set(new Vector2(-2 / PhantomLord.PIXELS_PER_METER, -7 / PhantomLord.PIXELS_PER_METER),
				new Vector2(2 / PhantomLord.PIXELS_PER_METER, -7 / PhantomLord.PIXELS_PER_METER));
		fDef2.shape = feet;
		box2DBody.createFixture(fDef2);
	}
	
	/*
	* idea from
	* https://gamedev.stackexchange.com/questions/103609/libgdx-colllisions-between-bullets-and-enemies-in-arrays
	*/
	public void onEnemyHit(Enemy enemy)
	{
		// if HatGhost collided with another HatGhost
		if(enemy instanceof HatGhost)
		{
			// If the HatGhost enemy we hit is just moving in its hat, and our HatGhost is NOT moving inside its hat, our HatGhost should die
			if(((HatGhost) enemy).currentState == State.MOVING_HIDEINHAT && currentState != State.MOVING_HIDEINHAT)
			{
				// Play StompOrHitEnemy sound
				PhantomLord.manager.get("assets/audio/sounds/StompOrHitEnemy.mp3", Sound.class).play();
				
				// Add score to HUD, as PLord managed to hit a GhostHat on head, and have hat kill this GhostHat
				PLordHud.addScore(500);
				
				// Kill our HatGhost (not the one we collided with)
				killEnemy();
			}
			// If we are a moving hat, the OTHER enemy's "onEnemyHit" method will determine if we killed it, only if it is MOVING though (NOT HIDING IN HAT)
			else if(currentState == State.MOVING_HIDEINHAT && ((HatGhost) enemy).currentState == State.MOVING)
				return;
			// Else Neither or both of the Ghost Hats are in the Moving_HIDEINHAT state, thus if they hit each other, they should reverse their velocities
			else
				reverseVelocity(true, false);			// Reverse velocity on x axis
		}
		// If HatGhost is not in its MOVING_HIDEINHAT state
		// and we didn't hit another HatGhost (then we hit a pumpkin for now)
		// Just reverse the velocity
		else if (currentState != State.MOVING_HIDEINHAT)
			reverseVelocity(true, false);
	}
	
	/*
	* idea from
	* http://www.emanueleferonato.com/2017/07/08/the-basics-behind-jumping-on-enemies-feature-explained-with-phaser-and-arcade-physics/
	*/
	@Override
	public void hitOnHead(PhantomLordChar pLord) 
	{
		// Play StompOrHitEnemy sound
		PhantomLord.manager.get("assets/audio/sounds/StompOrHitEnemy.mp3", Sound.class).play();
		
		// when HatGhost gets hit on head, want HatGhost to stop and hide/turn into the hat
		
		// If the current state does not have the Ghost hiding in hat already
		if(currentState != State.STATIC_HIDEINHAT)
		{
			// Set the current state to that the ghost is hiding in the hat
			currentState = State.STATIC_HIDEINHAT;
			
			// Need to also stop the HatGhost's velocity now that its hiding in its hat
			velocity.x = 0;
		}	
		// Static hide in hat state
		else
		{
			// If plord on left side of hat, hit hat to the right, else vice versa
			hitHat(pLord.getX() <= this.getX() ? HIT_RIGHT_SPEED : HIT_LEFT_SPEED);
		}
	}
	
	public State getCurrentState()
	{
		return currentState;
	}
	
	public void hitHat(int speed)
	{
		// set velocity in x axis equal to speed on shell
		velocity.x = speed;
		
		// Change current state since now we hit the hat
		currentState = State.MOVING_HIDEINHAT;
	}
	
	/*
	* Idea
	* https://gamedev.stackexchange.com/questions/80908/libgdx-how-to-change-animations
	*/
	public TextureRegion getFrame(float dt)
	{
		// Define variable to hold texutre resgions for animations
		TextureRegion region;
		
		// What state is pLord in?
		//currentState = getState();
		
		// If true, stateTimer = stateTimer + dt, otherwise stateTimer = 0
		// aka, If it doesn't equal to previous state, it must have transition to new state and need to reset the timer
		// THIS NEED TO GO BEFORE SWITCH STATMENT, as switch statement depends on its current ifno
		stateTimer = currentState == previousState ? stateTimer + dt : 0;
		previousState = currentState;
		
		/*********************************************************************
		 *  Return animation frame depending on current state and stateTimer
		 *********************************************************************/
		// stateTimer decides what frames gets rendered/pulled from animation (cycles through,
		// if it is passed to individual frame time that we sent to individual
		// constructor (0.1f), then it will advance to the next frame, if in a // looping animation if it gets to 
		// the end then it will return back to the very first frame Since Running is loopable animation, we also
		// need to return true as 2nd parameter (boolean for looping)
		switch(currentState)
		{
		case STATIC_HIDEINHAT:
			region = hideInHatStaticAnimation.getKeyFrame(stateTimer, true);		// true means loopable
			break;
		case MOVING_HIDEINHAT:
			region = hideInHatMovingAnimation.getKeyFrame(stateTimer, true);
			break;
		case MOVING:
		default:
			region = floatAnimation.getKeyFrame(stateTimer, true);
			break;
		}
		
		// HatGhost Texture by default faces left, so if he is walking to the right, 
		//yet facing left, need to flip him
		if(velocity.x > 0 && region.isFlipX() == false)
		{
			// Only flipping HatGhost on x axis
			region.flip(true, false);
		}
		// If walking to the left, and facing right, flip
		if(velocity.x < 0 && region.isFlipX() == true)
		{
			// Only flipping HatGhost on x axis
			region.flip(true, false);
		}
		
		// Return the texture region
		return region;
	}
	
	@Override
	public void update(float dt, PLordPlayScreen screen) 
	{
		if(destroyed)
			return;
		
		// Set region of texture
		setRegion(getFrame(dt));
		
	      // die when falling below ground
        if (box2DBody.getPosition().y < -20f) 
        {
        	killEnemy();
        }
		
		
		// If the Ghost has hid in its hat for more than 5 seconds, it will begin moving again
		if(currentState == State.STATIC_HIDEINHAT && stateTimer > 5)
		{
			// Now change state back to moving
			currentState = State.MOVING;
			
			// Set the walking x velocity
			velocity.x = MathUtils.random(.8f, 1.2f);
		}
		
		// Set sprite to position of box2Dbody
		// Note: I added "+ (2 / PhantomLord.PIXELS_PER_METER)" to vertical position cause Pumpkin seem slightly sunk into the ground, so I moved him up by 5 pixels relative to his box2DBody
		setPosition(box2DBody.getPosition().x - getWidth() / 2 + (1 / PhantomLord.PIXELS_PER_METER), box2DBody.getPosition().y - getHeight() / 2 + (5 / PhantomLord.PIXELS_PER_METER) );
		
		// Check if HatGhost is dead, if so, begin rotating it (so it looks like its doing a little spin when hopping up and dieing)
		if(currentState == State.DEAD)
		{
			// Flip each update
			killRotationDegrees = 5;
			
			// Rotate the sprite
			rotate(killRotationDegrees);
			
			// Destroy the body after 5 seconds (which also destroys the sprite)
			if(stateTimer > 2 && !destroyed)
			{
				world.destroyBody(box2DBody);
				destroyed = true;
				
				box2DBody = null;
				
				// Fix for HatGhost sprite attaching to new box2DBody that is spawned (the items),
				// Put "Remove HatGhost when we destroy anything and remove it from array of enemies)
				Box2DWorldCreator.removeHatGhost(this);
			}
		}
		// Set thebox2DBody velocity to the declared variable 'velocity' if not dead
		else
			box2DBody.setLinearVelocity(velocity);
	}
	
	/*
	 * Idea from
	 * http://www.gamefromscratch.com/post/2014/09/25/LibGDX-LibGDX-Tutorial-13-Physics-with-Box2D-Part-3-Collisions.aspx
	 */
	public void killEnemy()
	{
		if(currentState == State.DEAD)
			return;
		// Want HatGhost to do a little hop in the air then fall through the ground when dead
		currentState = State.DEAD;
		
		// Create new box2D Filter for dead HatGhost
		Filter filter = new Filter();
		
		// Want HatGhost to fall through ground when dead
		filter.maskBits = PhantomLord.NOTHING_BIT;
		
		// Get all the current fixtures and set them to the filter we just made
		for(Fixture fixture : box2DBody.getFixtureList())
			fixture.setFilterData(filter);
		
		// Make HatGhost jump up a little bit, then fall down through the ground
		// Applies an impulse in a certain direction, apply the Vector2 impulse 
		// to the center of mass of HatGhost body, and wake up body if asleep (should never be, but still will put as true)
		box2DBody.applyLinearImpulse(new Vector2(0, 1.5f), box2DBody.getWorldCenter(), true);
	}
	
	@Override
	public void reverseVelocity(boolean x, boolean y) 
	{
		super.reverseVelocity(x, y);
	}
	
	@Override
	public void fallVeloity() 
	{
		super.fallVeloity();
	}
}
