package edu.uidaho.phantomlord.Sprites.Items;

import com.badlogic.gdx.audio.Sound;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.BodyDef;
import com.badlogic.gdx.physics.box2d.CircleShape;
import com.badlogic.gdx.physics.box2d.EdgeShape;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.badlogic.gdx.physics.box2d.PolygonShape;

import edu.uidaho.phantomlord.PhantomLord;
import edu.uidaho.phantomlord.Scenes.PLordHud;
import edu.uidaho.phantomlord.Screens.PLordPlayScreen;
import edu.uidaho.phantomlord.Sprites.PhantomLordChar;

public class CandyCorn extends Item
{
	/*
	 * code modified/idea from
	 * https://stackoverflow.com/questions/28812776/game-character-items-libgdx/28986192
	 */
	public CandyCorn(PLordPlayScreen screen, float x, float y)
	{
		super(screen, x, y);
		
		// Set Region of sprite (Look at "CandyCorn.png" from "AllSpriteSheets (For libGDX Texture Packer)"
		// Since that is the x and y coordinates we need, but since only one image and starts at top left,
		// its just 0,0 and all the sprites I made have a width and height of 35x35,
		// Don't need anything fancier since there is no animation
		setRegion(screen.getAtlas().findRegion("CandyCorn"), 0, 0, 35, 35);
		
		// Set velocity for CandyCorn
		velocity = new Vector2(0.7f, 0);
	}
	
	@Override
	public void update(float dt)
	{
		super.update(dt);
		
		if(isDestroyed())
			return;
		
		//Set the position of the sprite to the center of the box2DBody, need to add
		// Note: I added "+ (4 / PhantomLord.PIXELS_PER_METER)" to vertical position cause CandyCorn seemed slightly 
		// sunk into the ground, so I moved him up by 4 pixels relative to his box2DBody
		setPosition(box2DBody.getPosition().x - getWidth() / 2, box2DBody.getPosition().y - getHeight() / 2 + (4 / PhantomLord.PIXELS_PER_METER) );
	
		// Set velocity of y relative to box2D (we are not forcing/needing a certain y velocity, so let physics simulation figure out y velocity)
		velocity.y = box2DBody.getLinearVelocity().y;
		// Set body to linear velocity on every update
		box2DBody.setLinearVelocity(velocity);
	}
	
	@Override
	public void use(PhantomLordChar pLord ) 
	{
		// Turn item into sensor when PLord collects it (so it doesn't push him)
		box2DBody.getFixtureList().first().setSensor(true);
		
		// Destroy the CandyCOrn when its gets used
		destroy();
		
		// Add score to HUD, as PLord managed to hit a GhostHat on head, and have hat kill Pumpkin
		PLordHud.addScore(500);
		
		// isPLordAverage() returns true if pLord is average, thus he can acquire health,
		// otherwise if he is average, do nothing to him when he gets Candy Corn
		if(pLord.isPLordAverage())
		{
			// Still Play Power Up sound though
			PhantomLord.manager.get("assets/audio/sounds/PowerUp.mp3", Sound.class).play();
		}
		else if(pLord.isPLordPower())
		{
			// Still Play Power Up sound though
			//PhantomLord.manager.get("assets/audio/sounds/PowerUp.mp3", Sound.class).play();
		}
		else
		{
			// Give health to PLord
			pLord.pLordAvgHealth();
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
		shape.setRadius(6 / PhantomLord.PIXELS_PER_METER);
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
		feet.set(new Vector2(-2 / PhantomLord.PIXELS_PER_METER, -7 / PhantomLord.PIXELS_PER_METER),
				new Vector2(2 / PhantomLord.PIXELS_PER_METER, -7 / PhantomLord.PIXELS_PER_METER));
		fDef2.shape = feet;
		box2DBody.createFixture(fDef2);
	}
}
