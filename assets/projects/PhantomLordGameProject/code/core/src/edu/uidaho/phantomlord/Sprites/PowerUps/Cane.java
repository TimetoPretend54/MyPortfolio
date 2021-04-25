package edu.uidaho.phantomlord.Sprites.PowerUps;

import com.badlogic.gdx.graphics.g2d.Animation;
import com.badlogic.gdx.graphics.g2d.Batch;
import com.badlogic.gdx.graphics.g2d.Sprite;
import com.badlogic.gdx.graphics.g2d.TextureAtlas;
import com.badlogic.gdx.graphics.g2d.TextureRegion;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.BodyDef;
import com.badlogic.gdx.physics.box2d.CircleShape;
import com.badlogic.gdx.physics.box2d.EdgeShape;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.badlogic.gdx.physics.box2d.World;
import com.badlogic.gdx.utils.Array;
import edu.uidaho.phantomlord.Tools.PLordWorldContactListener;
import edu.uidaho.phantomlord.Sprites.Enemies.Enemy;
import edu.uidaho.phantomlord.PhantomLord;
import edu.uidaho.phantomlord.Screens.PLordPlayScreen;

public class Cane extends Sprite
{
	public World world;
	public Body box2DBody;
	
	private Animation<TextureRegion> caneSpinning;
	private TextureRegion hitting;
	
    public boolean toBeDestroyed;
    protected boolean destroyed;
	
	private boolean movingRight;
	private boolean hit;
	private float stateTime;
	private float prevX;
	private PLordPlayScreen screen;
	
	public Cane(PLordPlayScreen screen, float x, float y, boolean movingRight)
	{
		this.screen = screen;
		this.world = screen.getWorld();
		this.movingRight = movingRight;
		
        toBeDestroyed = false;
        destroyed = false;
		
		// Create an array of texture regions to pass the constructor for the animation
		Array<TextureRegion> frames = new Array<TextureRegion>();
		
		/*********************************************************
		 *  Initialize Death Animation for LPhantom Lord Death
		 *********************************************************/
		// Death frame animation at index 6 for "PhantomLordLow" texture region
		for (int i = 0; i < 8; i++)
		{
			frames.add(new TextureRegion(screen.getAtlas().findRegion("CanePower"), i * 35, 0, 35, 35));
		}
		caneSpinning = new Animation<TextureRegion>(0.03f, frames);		
		
		frames.clear();
		
		/* Sprite/Aniamtion for cane hitting enemy */
		hitting = new TextureRegion(screen.getAtlas().findRegion("CanePower"), 3*35, 0, 35, 35);
	
		// Use 0, as we don't have to worry about changing states
		//setRegion(caneSpinning.getKeyFrame(0, true));
		setBounds(getX(), getY(), 24 / PhantomLord.PIXELS_PER_METER, 24 / PhantomLord.PIXELS_PER_METER);
		
		stateTime = 0;
		hit = false;
		
		setPosition(x, y);
		defineCane();
		
		// Have to put this after define cane, as we don't have box2DBody for it before then
		prevX = box2DBody.getPosition().x;
	}
	
	public void defineCane(){
		// Creating a new body at position where we set it during the constructor 9floatx, and float y)
		// Define body for CandyCorn and assign body to world
		BodyDef bDef = new BodyDef();
		bDef.position.set(getX(), getY());																// Box2D Body is at position of created Pumpkin Sprite, after this the
		bDef.type = BodyDef.BodyType.DynamicBody;														// sprite/animation stays in line with the actual physics collider Also, 
		box2DBody = world.createBody(bDef);		
		box2DBody.setGravityScale(0);
		// when we Pumpkin moves, the box2DBody gets moved to correspond to that,
													
		// so the Spite needs to be rendered wherever the box2DBody is
		// Define fixture for CandyCorn (make radius appropriate size that matches the Pumpkin sprite)
		FixtureDef fDef = new FixtureDef();
		CircleShape shape = new CircleShape();
		shape.setRadius(4 / PhantomLord.PIXELS_PER_METER);
		// Category for CandyCorn, aka, what is this fixture's category
		fDef.filter.categoryBits = PhantomLord.WEAPON_BIT;
		// What can CandyCorn collide with
		fDef.filter.maskBits =  PhantomLord.OBJECT_BIT |
				PhantomLord.PLORD_BIT | PhantomLord.PLORDHEAD_BIT |
				PhantomLord.ENEMY_BIT | PhantomLord.ENEMYHEAD_BIT;
		fDef.restitution = 0.1f;			// Half bounciness, so if was 10 pixels from top of head, then hit, PLord will bounce 5 pixels off
		// Set this fixture to our body for CandyCorn and setUserData for WorldContactLister
		fDef.shape = shape;
		box2DBody.createFixture(fDef).setUserData(this);
		
		// Fix issue where CandyCorn will respond to any collision of the ground beneath it, only care about the ground that
		// relate to walls, so create 'Feet' for CandyCorn so it doesn't ever touch the bottom part of ground		// Also allows it so the main box2DBody doesn't have to be hitting with ground, but instead  the feet (which
		FixtureDef fDef2 = new FixtureDef();
		EdgeShape feet = new EdgeShape();
		// Coordinates relative to CandyCorn body, edgeShape acts as feet to glide across tiles (since radius of pumpkin is 6, go -8 below the
		// center, and the 'feet' will be exactly 1 pixels below the body of pumpkin, which seems good
		feet.set(new Vector2(-2 / PhantomLord.PIXELS_PER_METER, -5 / PhantomLord.PIXELS_PER_METER),
				new Vector2(2 / PhantomLord.PIXELS_PER_METER, -5 / PhantomLord.PIXELS_PER_METER));
		fDef2.shape = feet;
		box2DBody.createFixture(fDef2);
		
		feet.dispose();
		shape.dispose();
	}
	
	public void update(float dt){

        stateTime += dt;
        
		if(isDestroyed())
			return;
		
		// if we need to destroy item		
        if (toBeDestroyed && !destroyed) {
	            if (stateTime > 0.01f) {
		                world.destroyBody(box2DBody);
		                box2DBody = null;
		                destroyed = true;
		                return;
			}
        }
        else if(!destroyed) {
            // We collided with something if the cane's x position didn't increase (if moving right) or decrease (if moving left)
            if ((movingRight && prevX > box2DBody.getPosition().x) || (!movingRight && prevX < box2DBody.getPosition().x)) {
                hit = true;
            }
            
            if(!hit) {
            	//System.out.println("Spinning X: " + box2DBody.getPosition().x + " Y: " + box2DBody.getPosition().y );
            	// Set cane throwing animation, set true to loop
            	setRegion(caneSpinning.getKeyFrame(stateTime, true));
            	
            	// Go faster than pLord
            	float speed = 3.0f; 
            	
            	// Make Cane go to the right if pLord is facing right when thrown
            	if(movingRight)
            		box2DBody.applyLinearImpulse(new Vector2((speed - box2DBody.getLinearVelocity().x) * box2DBody.getMass(), 0), box2DBody.getWorldCenter(), true);
            	// Make Cane go to the right if pLord is facing right when thrown
            	else{
            		box2DBody.applyLinearImpulse(new Vector2((-speed - box2DBody.getLinearVelocity().x) * box2DBody.getMass(), 0), box2DBody.getWorldCenter(), true);
            		setFlip(true, false);
            	}
            }
            // Otherwise we hit "something", destory the cane and reset stateTime
            else{
            	if(!toBeDestroyed){
            		// Play hit sprite/animation
            		setRegion(hitting);
            		stateTime = 0;
            		toBeDestroyed = true;
            	}
            }
            
            prevX = box2DBody.getPosition().x;
            
            //System.out.println("Box2DPosX: " + box2DBody.getPosition().x + " CameraX: " + screen.getCamera().position.x);
            //System.out.println("Box2DPosY: " + box2DBody.getPosition().y + " CameraY: " + screen.getCamera().position.y);
            // if the cane leaves the screen, queueDestroy
            if (Math.abs(box2DBody.getPosition().x - screen.getCamera().position.x) > ( PhantomLord.V_WIDTH / PhantomLord.PIXELS_PER_METER) / 2 )
            	toBeDestroyed = true;
            
            // Set new sprite position of cane after velcoity
            setPosition(box2DBody.getPosition().x - getWidth() / 2, box2DBody.getPosition().y - getHeight() / 2);
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
	
    public boolean isDestroyed() {
        return destroyed;
    }
}
