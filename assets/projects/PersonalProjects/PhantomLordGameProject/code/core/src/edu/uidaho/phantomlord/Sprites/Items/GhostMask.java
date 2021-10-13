package edu.uidaho.phantomlord.Sprites.Items;

import com.badlogic.gdx.audio.Sound;
import com.badlogic.gdx.graphics.g2d.Animation;
import com.badlogic.gdx.graphics.g2d.TextureRegion;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.BodyDef;
import com.badlogic.gdx.physics.box2d.CircleShape;
import com.badlogic.gdx.physics.box2d.EdgeShape;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.badlogic.gdx.utils.Array;

import edu.uidaho.phantomlord.PhantomLord;
import edu.uidaho.phantomlord.Scenes.PLordHud;
import edu.uidaho.phantomlord.Screens.PLordPlayScreen;
import edu.uidaho.phantomlord.Sprites.PhantomLordChar;

public class GhostMask extends Item
{
	// Animations 
	private Animation<TextureRegion> ghostMaskSpawn;			// Initial PLord health/state
	// Keep track of time we are in any given state
	private float stateTimer;
	
	/*
	 * code modified/idea from
	 * https://stackoverflow.com/questions/28812776/game-character-items-libgdx/28986192
	 */
	public GhostMask(PLordPlayScreen screen, float x, float y)
	{
		super(screen, x, y);
		
		// Create an array of texture regions to pass the constructor for the animation
		Array<TextureRegion> frames = new Array<TextureRegion>();
		
		// Set Region of sprite (Look at "CandyCorn.png" from "AllSpriteSheets (For libGDX Texture Packer)"
		// Since that is the x and y coordinates we need, but since only one image and starts at top left,
		// its just 0,0 and all the sprites I made have a width and height of 35x35,
		// Don't need anything fancier since there is no animation
		setRegion(screen.getAtlas().findRegion("GhostMask"), 0, 0, 35, 35);
		
		// Set size and initial position of Pumpkin Sprite, 27X27 seems like good size for pumpkin (both have to be divided by our Pixels_per_meter 1st)
		setBounds(getX(), getY(), 20 / PhantomLord.PIXELS_PER_METER, 20 / PhantomLord.PIXELS_PER_METER);
		
		// No velocity, ghost mask is static item
		velocity = new Vector2(0, 0);
		
		/*
		 *  Make spawning animation
		 */
		for (int k = 0; k < 70; k++)
		{
			// Blank slate frame, so it looks like the Ghost Mask is blinking when first spawing, put it first so once we get done with animation, the ghost mask 
			// sprite stays on screen, I could of also just loaded the region after getting done running the animation, but eh.
			frames.add(new TextureRegion(screen.getAtlas().findRegion("GhostMask"), 0, 0, 0, 0));
			
			frames.add(new TextureRegion(screen.getAtlas().findRegion("GhostMask"), 0, 0, 35, 35));
		}
		ghostMaskSpawn = new Animation<TextureRegion>(0.009f, frames);						// 0.1f good for time between each frame for jumping animation
		
		frames.clear();
		
		/* set timer to zero */
		stateTimer = 0;
	}
	
	@Override
	public void update(float dt)
	{
		super.update(dt);
		
		if(isDestroyed())
			return;
		
		stateTimer += dt;
		
		//Set the position of the sprite to the center of the box2DBody, need to add
		// Note: I added "+ (4 / PhantomLord.PIXELS_PER_METER)" to vertical position cause CandyCorn seemed slightly 
		// sunk into the ground, so I moved him up by 4 pixels relative to his box2DBody
		setPosition(box2DBody.getPosition().x - getWidth() / 2, box2DBody.getPosition().y - getHeight() / 2 + (4 / PhantomLord.PIXELS_PER_METER) );
		
		// Set velocity of y relative to box2D (we are not forcing/needing a certain y velocity, so let physics simulation figure out y velocity)
		velocity.y = box2DBody.getLinearVelocity().y;
		// Set body to linear velocity on every update
		box2DBody.setLinearVelocity(velocity);
		
		// Set Our animation texture and position, for getKeyFrame, remember put false in 2nd parameter, so Ghost mask only plays animation when spawning
		setRegion(ghostMaskSpawn.getKeyFrame(stateTimer, false));
	}
	
	@Override
	public void use(PhantomLordChar pLord ) 
	{
		// Turn item into sensor when PLord collects it (so it doesn't push him)
		box2DBody.getFixtureList().first().setSensor(true);
		
		// Destroy the CandyCOrn when its gets used
		destroy();
		
		// Add score to HUD, as PLord managed to hit a GhostHat on head, and have hat kill Pumpkin
		PLordHud.addScore(1000);
		
		// isPLordPower() returns false if pLord is not powerUp state, thus he can acquire health,
		// otherwise if he is powered Up state, do nothing to him when he gets Ghost Mask
		if(pLord.isPLordPower())
		{
			// Still Play Power Up sound though
			PhantomLord.manager.get("assets/audio/sounds/PowerUp.mp3", Sound.class).play();
		}
		else
		{
			// Give powerUp health/ability to PLord
			pLord.pLordPowerUpHealth();
		}
	}

	/*
	 * statring code/idea from
	 * http://www.iforce2d.net/b2dtut/bodies
	 * &
	 * http://www.gamefromscratch.com/post/2014/08/27/LibGDX-Tutorial-13-Physics-with-Box2D-Part-1-A-Basic-Physics-Simulations.aspx
	 */
	@Override
	public void defineItem() 
	{
		// Creating a new body at position where we set it during the constructor 9floatx, and float y)
		// Define body for CandyCorn and assign body to world
		BodyDef bDef = new BodyDef();
		bDef.position.set(getX(), getY());																// Box2D Body is at position of created Pumpkin Sprite, after this the
		bDef.type = BodyDef.BodyType.DynamicBody;														// sprite/animation stays in line with the actual physics collider Also, 
		box2DBody = world.createBody(bDef);																// when we Pumpkin moves, the box2DBody gets moved to correspond to that,
													
		// so the Spite needs to be rendered wherever the box2DBody is
		// Define fixture for CandyCorn (make radius appropriate size that matches the Pumpkin sprite)
		FixtureDef fDef = new FixtureDef();
		CircleShape shape = new CircleShape();
		shape.setRadius(3 / PhantomLord.PIXELS_PER_METER);
		// Category for CandyCorn, aka, what is this fixture's category
		fDef.filter.categoryBits = PhantomLord.ITEM_BIT;
		// What can CandyCorn collide with
		fDef.filter.maskBits =  PhantomLord.OBJECT_BIT |
				PhantomLord.PLORD_BIT | PhantomLord.PLORDHEAD_BIT;
		// Set this fixture to our body for CandyCorn and setUserData for WorldContactLister
		fDef.shape = shape;
		box2DBody.createFixture(fDef).setUserData(this);
		
		// Fix issue where CandyCorn will respond to any collision of the ground beneath it, only care about the ground that
		// relate to walls, so create 'Feet' for CandyCorn so it doesn't ever touch the bottom part of ground		// Also allows it so the main box2DBody doesn't have to be hitting with ground, but instead  the feet (which
		FixtureDef fDef2 = new FixtureDef();
		EdgeShape feet = new EdgeShape();
		// Coordinates relative to CandyCorn body, edgeShape acts as feet to glide across tiles (since radius of pumpkin is 6, go -8 below the
		// center, and the 'feet' will be exactly 1 pixels below the body of pumpkin, which seems good
		feet.set(new Vector2(-2 / PhantomLord.PIXELS_PER_METER, -4 / PhantomLord.PIXELS_PER_METER),
				new Vector2(2 / PhantomLord.PIXELS_PER_METER, -4 / PhantomLord.PIXELS_PER_METER));
		fDef2.shape = feet;
		box2DBody.createFixture(fDef2);
		
		feet.dispose();
		shape.dispose();
	}

}
