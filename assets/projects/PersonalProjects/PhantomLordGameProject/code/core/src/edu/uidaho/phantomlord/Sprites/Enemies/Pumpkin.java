package edu.uidaho.phantomlord.Sprites.Enemies;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.audio.Sound;
import com.badlogic.gdx.graphics.g2d.Animation;
import com.badlogic.gdx.graphics.g2d.Batch;
import com.badlogic.gdx.graphics.g2d.TextureRegion;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
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

public class Pumpkin extends Enemy
{
	// Keep track of time we are in any given state
	private float stateTimer;
	
	// Animations
	private Animation<TextureRegion> bounceAnimation;
	
	// Frames array to hold the animation
	private Array<TextureRegion> frames;
	
	// Boolean variable to determine if we need to destroy Pumpkin box2DBody outside of the simulation
	private boolean setToDestroy;
	private boolean destroyed;
	public boolean waitToRemoveSprite;
	public boolean bodyDestroyed;
	
	private boolean nonStompKill;
	private float killRotationDegrees;
	private Filter filter;
	
	public Pumpkin(PLordPlayScreen screen, float x, float y) 
	{
		super(screen, x, y);
		
		/*
		 * Animation idea from
		 * https://github.com/libgdx/libgdx/wiki/2D-Animation
		 */
		
		// Create an array of texture regions to pass the constructor for the animation
		//Array<TextureRegion> frames = new Array<TextureRegion>();
		frames = new Array<TextureRegion>();
		
		/*
		/********************************************************
		 *  Initialize Bounce Animation for Pumpkin
		 ********************************************************/
		// Bounce is at animation indexes 1-2 for "Pumpkin" at row 1 (row index 0) this index# and row# is given by "Pumpkin" texture region, which means we only focus on coordinates of
		// "Pumpkin" region, thus the top left of just that spriteSheet will be (0,0), image that we are looking at the "Pumpkin.png" from "AllSpriteSheets (For libGDX Texture Packer)" folder
		// And getting the coordinate from there, which we can use a for loop to quickly navigate though and get sprite frame, since each sprite is 35x35 
		for (int i = 1; i < 3; i++)
			frames.add(new TextureRegion(screen.getAtlas().findRegion("Pumpkin"), i*35, 0 * 37, 35, 35));
		bounceAnimation = new Animation<TextureRegion>(0.4f, frames);					// Might need higher or lower time (1st parameter, depending on animation)
		stateTimer = 0;
		frames.clear();
		
		// Set the initial rotation degrees
		killRotationDegrees = 0;
		nonStompKill = false;
		
		// Set size and initial position of Pumpkin Sprite, 27X27 seems like good size for pumpkin (both have to be divided by our Pixels_per_meter 1st)
		setBounds(getX(), getY(), 27 / PhantomLord.PIXELS_PER_METER, 27 / PhantomLord.PIXELS_PER_METER);
	
		// Assume initially that Pumpkin is not destroyed and not about to be destroyed
		setToDestroy = false;
		destroyed = false;
		waitToRemoveSprite = false;
		bodyDestroyed = false;
	}
	
	public void update(float dt, PLordPlayScreen screen)
	{
		if(bodyDestroyed)
			return;
		
		stateTimer += dt;
		
		// Set new position of sprite on screen, get x and y position of the box2DBody, then subtract the 
		// x and y by HALF of the sprite's width and height, respectively, to move sprite to center position of
		// box2DBody
		// Note: I added "+ (2 / PhantomLord.PIXELS_PER_METER)" to vertical position cause Pumpkin seem slightly sunk into the ground, so I moved him up by 4 pixels relative to his box2DBody
		setPosition(box2DBody.getPosition().x - getWidth() / 2, box2DBody.getPosition().y - getHeight() / 2 + (4 / PhantomLord.PIXELS_PER_METER) );
		
		
	      // die when falling below ground
        if (box2DBody.getPosition().y < -20f) 
        {
        	killEnemy();
        }
		
        if(nonStompKill)
        {
    		// Flip each update
    		killRotationDegrees = 5;
    		
    		// Rotate the sprite
    		rotate(killRotationDegrees);
    		
			// Deterime if we have already started counting the time before removing pumpkin sprite (need it to show for a second for disappearing)
			if (waitToRemoveSprite == false)
			{
				waitToRemoveSprite = true;
				
				//Reset timer to know how long the Pumpkin has been dead
				stateTimer = 0;
			}
    		
    		// Destroy the body after 5 seconds (which also destroys the sprite)
    		if(stateTimer > 2 && !destroyed)
    		{
				// Remove the Pumpkin box2DBody (still need to override draw mehtod to remove sprite, method "draw" is in this class)
				if(box2DBody != null)
					world.destroyBody(box2DBody);
				
				box2DBody = null;
				
				bodyDestroyed = true;
				
				Box2DWorldCreator.removePumpkin(this);
				
				// set the destroyed variable to true
				destroyed = true;
				nonStompKill = false;
				
				// set back to false
				waitToRemoveSprite = false;
    		}
        }
		// Need to check if Pumpkin is being setToDestroy, if it is destroy box2DBody for pumpkin and show the smashed pumpkin sprite
        else if(setToDestroy && !destroyed)
		{
			// Deterime if we have already started counting the time before removing pumpkin sprite (need it to show for a second for disappearing)
			if (waitToRemoveSprite == false)
			{
				waitToRemoveSprite = true;
				
				//Reset timer to know how long the Pumpkin has been dead
				stateTimer = 0;
			}
			
			// if body already destroyed, DONT TRY TO DESTORY IT AGAIN
			if (bodyDestroyed == false)
			{
				// Create new box2D Filter for pLord
				filter = new Filter();
				// Want Body to not collide with anything besdies object
				filter.maskBits = PhantomLord.OBJECT_BIT;
				// Get all the current fixtures for PLord's box2DBody and set them to the filter we just made
				for(Fixture fixture : box2DBody.getFixtureList())
					fixture.setFilterData(filter);
			}
			
			// Need to set Sprite visual texture to "Smashed Pumpkin" (since it was now hit)
			// Use the Region for "Pumpkin" find the sprite frame and enter its x & y coordinates, and its width and height
			// Can find it in the "Pumpkin.png" from "AllSpriteSheets (For libGDX Texture Packer)" and use MediBangPaint to find coordinate if need be
			// But since each sprite frame is 35X35, just find index frame, which in this case is index 0, and row_index 0, with width and height 35
			setRegion(new TextureRegion(screen.getAtlas().findRegion("Pumpkin"), 0*35, 0*35, 35, 35));
			
			
			// Fix for HatGhost sprite attaching to new box2DBody that is spawned (the items),
			// Put "Remove Pumplin when we destroy anything and remove it from array of enemies)
			if(stateTimer > 1 && !destroyed)
			{
				// Remove the Pumpkin box2DBody (need to override draw mehtod to remove sprite, method "draw" is in this class)
				if(box2DBody != null)
					world.destroyBody(box2DBody);
				
				box2DBody = null;
				bodyDestroyed = true;
				
				Box2DWorldCreator.removePumpkin(this);
				
				// set the destroyed variable to true
				destroyed = true;
				
				// set back to false
				waitToRemoveSprite = false;
			}
		}
		
		// play bounding Pumpkin animation and set position of sprite relative to box2DBody
		else if(!destroyed)
		{
			// Set linear velocity (movement) if pumpkin not destroyed, set the velocity to the velocity Vector2 object from the 'enemy' class
			box2DBody.setLinearVelocity(velocity);
			
			// Set Our animation texture and position, for getKeyFrame, remember put true in 2nd parameter for looping animation
			setRegion(bounceAnimation.getKeyFrame(stateTimer, true));
		}
	}

	// For killing enemy when it falls of ledge
	public void killEnemy()
	{
		setToDestroy = true;
		
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
	
		nonStompKill = true;
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
		// Define body for Pumpkin and assign body to world
		BodyDef bDef = new BodyDef();
		bDef.position.set(getX(), getY());																// Box2D Body is at position of created Pumpkin Sprite, after this the
		bDef.type = BodyDef.BodyType.DynamicBody;														// sprite/animation stays in line with the actual physics collider Also, 
		box2DBody = world.createBody(bDef);																// when we Pumpkin moves, the box2DBody gets moved to correspond to that,
																										// so the Spite needs to be rendered wherever the box2DBody is
		// Define fixture for Pumpkin (make radius appropriate size that matches the Pumpkin sprite)
		FixtureDef fDef = new FixtureDef();
		CircleShape shape = new CircleShape();
		shape.setRadius(6 / PhantomLord.PIXELS_PER_METER);
		
		// Category for Pumpkin, aka, what is this fixture's category
		fDef.filter.categoryBits = PhantomLord.ENEMY_BIT;
		// What can Enemy collide with
		fDef.filter.maskBits =  PhantomLord.OBJECT_BIT | PhantomLord.PLORD_BIT | PhantomLord.ENEMY_BIT | PhantomLord.WEAPON_BIT;;
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
		// Add Bounciness, so when PLord hits head o Pumpkin, he bounces up a bit
		// This is done by setting the restitution of the fixture
		fDef.restitution = 0.75f;			// Half bounciness, so if was 10 pixels from top of head, then hit, PLord will bounce 5 pixels off
		// Set filter category for fixture of enemy head
		fDef.filter.categoryBits = PhantomLord.ENEMYHEAD_BIT;
		// Create the Head fixture and use setUsedData to have access to this class from collision handler
		box2DBody.createFixture(fDef).setUserData(this);
		
		// Fix issue where Pumpkin will respond to any collision of the ground beneath it, only care about the ground that
		// relate to walls, so create 'Feet' for Pumpkin so it doesn't ever touch the bottom part of ground
		// Also allows it so the main box2DBody doesn't have to be hitting with ground, but instead  the feet (which
		// will touch the ground now),where the 'feet' we put -7 pixels from the main body (as shown below in both Vector2's 2nd parameter)
		// if the 'feet' weren't far 'enough' away from body (usually only needs to be 1 pixel away from it, so if radius is 6, make sure 
		// y = -7 for Vector2 parameters, the fDef.restitution wouldn't work and PLord wouldn't bounce off enemy
		FixtureDef fDef2 = new FixtureDef();
		EdgeShape feet = new EdgeShape();
		// Coordinates relative to Pumpkin body, edgeShape acts as feet to glide across tiles (since radius of pumpkin is 6, go -7 below the
		// center, and the 'feet' will be exactly 1 pixels below the body of pumpkin, which seems good
		feet.set(new Vector2(-2 / PhantomLord.PIXELS_PER_METER, -7 / PhantomLord.PIXELS_PER_METER),
				new Vector2(2 / PhantomLord.PIXELS_PER_METER, -7 / PhantomLord.PIXELS_PER_METER));
		fDef2.shape = feet;
		box2DBody.createFixture(fDef2);
	}
	
	public void draw(Batch batch)
	{
		// if Pumpkin is not destroyed or the stateTimer is less than 1 second
		// Textures are ONLY drawn if NOT destroyed OR stateTimer is less than 1 second
		
		//System.out.println("StateTime: " + stateTimer);
		
		if(!destroyed || stateTimer < 1)
		{
			//System.out.println("Not destroyed");
			super.draw(batch);
		}
	}
	
	public void onEnemyHit(Enemy enemy)
	{
		// If gets hit by enemmy, want to check if Pumpkin gets hit by HatGhost, and if so, is it in MOVING HIDEINHAT state?
		// if so that hat should hit and kill Pumpkin
		if(enemy instanceof HatGhost && ((HatGhost) enemy).currentState == HatGhost.State.MOVING_HIDEINHAT)
		{
			// Play StompOrHitEnemy sound
			PhantomLord.manager.get("assets/audio/sounds/StompOrHitEnemy.mp3", Sound.class).play();
			
			// Add score to HUD, as PLord managed to hit a GhostHat on head, and have hat kill Pumpkin
			PLordHud.addScore(300);
			
			// Destroy pumpkins
			killEnemy();
		}
		// Just collide with each other and reverse their velocity
		else
		{
			// Reverse velocity in x axis
			reverseVelocity(true, false);
		}
	}
	
	@Override
	public void hitOnHead(PhantomLordChar pLord)
	{
		// logic for what happens when Pumpkin gets hit on the head by PLord
		
		// Add score to HUD, pLOrd killed Pumpkin Enemy
		PLordHud.addScore(100);
		
		// Cannot destroy the box2DBody within this method, as this method is being called
		// Technically within the box2D simulation (world.step) which will Not allow you to 
		// delete a collision while it is simulating it
		// So need to make boolean variable to do it outside of simulation
		setToDestroy = true;
		
		// Play StompOrHitEnemy sound
		PhantomLord.manager.get("assets/audio/sounds/StompOrHitEnemy.mp3", Sound.class).play();
	}
}
