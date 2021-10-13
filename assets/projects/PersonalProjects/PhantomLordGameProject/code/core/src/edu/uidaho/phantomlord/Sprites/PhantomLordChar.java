package edu.uidaho.phantomlord.Sprites;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.audio.Music;
import com.badlogic.gdx.audio.Sound;
import com.badlogic.gdx.graphics.g2d.Animation;
import com.badlogic.gdx.graphics.g2d.Batch;
import com.badlogic.gdx.graphics.g2d.Sprite;
import com.badlogic.gdx.graphics.g2d.TextureRegion;
import com.badlogic.gdx.maps.MapObject;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.BodyDef;
import com.badlogic.gdx.physics.box2d.CircleShape;
import com.badlogic.gdx.physics.box2d.EdgeShape;
import com.badlogic.gdx.physics.box2d.Filter;
import com.badlogic.gdx.physics.box2d.Fixture;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.badlogic.gdx.physics.box2d.World;
import com.badlogic.gdx.utils.Array;

import edu.uidaho.phantomlord.PhantomLord;
import edu.uidaho.phantomlord.Scenes.PLordHud;
import edu.uidaho.phantomlord.Screens.PLordPlayScreen;
import edu.uidaho.phantomlord.Sprites.Enemies.Enemy;
import edu.uidaho.phantomlord.Sprites.Enemies.HatGhost;
import edu.uidaho.phantomlord.Sprites.Items.HatCandy;
import edu.uidaho.phantomlord.Sprites.Items.ItemDefinition;
import edu.uidaho.phantomlord.Tools.Box2DWorldCreator;

public class PhantomLordChar extends Sprite
{
	public enum State { FALLING, JUMPING, STANDING, RUNNING, FASTRUNNING, AVGUP, POWERUP, THROW, GOTHIT, DEAD, GOALREACHED, LEVELFINISHED };
	public State currentState;
	public State previousState;
	public World world;
	public Body box2DBody;
	
	public float keyPressedTime;
	
    public final float normalForce = 0.1f;
    public final float normalSpeedMax = 1.0f;
    public final float fastForce = 0.15f;
    public final float fastSpeedMax = 2.0f;
    
	// For Cane throw
	public final float caneInterval = 0.3f;
	public float caneTimer = 0f;
	// Boolean to tell which direction PLord is running
	public boolean runningRight;
	
	// texture region variables for Phantom Lord actions
	private TextureRegion pLordStand;
	
	public boolean runThrowingAnimation;
	
	// Animations 
	private Animation<TextureRegion> lowPLordRun;					// Initial PLord health/state
	private Animation<TextureRegion> lowPLordFastRun;			
	private Animation<TextureRegion> lowPLordJump;
	private Animation<TextureRegion> lowPLordStance;
	private Animation<TextureRegion> pLordDead;
	private Animation<TextureRegion> candyCornAquiredPLord;			// Animation for when PLord changes form low to average
	private Animation<TextureRegion> ghostMaskAquiredPLord;
	private Animation<TextureRegion> pLordPowerUpHit;
	private Animation<TextureRegion> pLordAvgUpHit;
	private Animation<TextureRegion> averagePLordRun;				// When PLord gets candycorn
	private Animation<TextureRegion> averagePLordFastRun;		
	private Animation<TextureRegion> averagePLordJump;
	private Animation<TextureRegion> averagePLordStance;
	private Animation<TextureRegion> powerPLordRun;					// When PLord gets GhostMask
	private Animation<TextureRegion> powerPLordFastRun;	
	private Animation<TextureRegion> powerPLordJump;
	private Animation<TextureRegion> powerPLordStance;
	private Animation<TextureRegion> throwCane;
	
	// Create Vector2 variable for PLord Velocity
	public Vector2 velocity;
	
	// Keep track of time we are in any given state
	private float stateTimer;		
	
	// Boolean to tell if Plord health is low or not
	private boolean pLordIsAverage;
	
	// Boolean to tell if Plord health is powerUp (high) or not
	private boolean pLordIsPowerUp;
	
	// Boolean to tell if we have run the CandyCornAcquireAnimatio
	private boolean runCandyCornAcquireAnimation;
	
	// Boolean to tell if we have run the GhostMaskAcquireAnimation
	private boolean runGhostMaskAcquireAnimation;
	
	// Boolean to tell if Plord is dead
	private boolean pLordIsDead;
	
	// Boolean to tell if PLord has reached goal
	public boolean pLordAtGoal;
	
	// Boolean to tell if PLord has finished Level
	private boolean pLordFinished;
	
	// Boolean to tell if Plord got hit, but didn't die
	public boolean pLordGotHit;
	
	// Filture for setting PLord's fixture filters when enemy hits him, so he can be invincible 
	// and not collide (be pushed by enemy once hit)
	private Filter filter;
	
	private PLordPlayScreen screen;
	
	// Constructor
	public PhantomLordChar(PLordPlayScreen screen)
	{
		this.screen = screen;
		this.world = screen.getWorld();
		
		// Initialize states
		currentState = State.STANDING;
		previousState = State.STANDING;
		stateTimer = 0;
		runningRight = true;
		
		// Create an array of texture regions to pass the constructor for the animation
		Array<TextureRegion> frames = new Array<TextureRegion>();
		
		// then we made new pLord for new level and he has different state than default, start him off like last update/state has was in before finishing the previosus level
		if(PhantomLord.pLordState != 1)
		{
			switch(PhantomLord.pLordState)
			{
				case 2:
					pLordIsPowerUp = false;
					pLordIsAverage = true;
					break;
				case 3:
					pLordIsPowerUp = true;
					pLordIsAverage = false;
					break;
			}
		}
		
		/*
		 * Animation idea/structure from
		 * https://github.com/libgdx/libgdx/wiki/2D-Animation
		 */
		
		/**************************************************************************
		 *  				HOW TO READ IN SPRITE FRAMES CORRECTLY!
		 *  
		 *  READ ALL OF IT BECAUSE LIBGDX  CAN BE VERY CONFUSING WHEN IT COMES TO 
		 *  TEXTURE REGIONS! 
		 *  
		 *  SINCE ALL SPRITES in "Phantom Lord" ARE 35x35, EACH SPRITE FRAME IS 
		 *  35 PIXELS HORIZONTALLY APART AND 35 PIXELS VERTICALLY APART
		 **************************************************************************/
		// Indexes & Rows are for their particular spriteSheet (Unless you DONT specify the region, but that a big pain if you have a big mainSpriteSheetPack), for instance here we have 
		// "PhantomLordAverage" that we specify in "TextureRegion(screen.getAtlas().findRegion("PhantomLordAverage")" below in the for loop for initializing the running animation,
		// which has 0-8 indexes, and only 1 row, which is row index 0, each spriteSheet has it own # of indexes, and y starts at 0, unless there is a 2nd row for that ONE sprite sheet 
		// (NOT THE COMBINED PACK SPITESHEET!)
		//
		// We have 6 running stances (animation indexes 0-5), there are 9 animations for "PhantomLordAverage", and they are laid out in a SINGLE ROW (thus we count each as a index)
		// at index 0, (thus 0-8 indexes) since running starts at 0, ex: our stance animation was index 7
		// each animation is about 35 pixels apart horizontally and 35 vertically, since PhantomLordAverage
		// animations are the specified region we are looking for, we are ONLY looking at their spriteSheet 
		// 
		// IMPORTANT IDEA: Imagine if we were just looking at the original "PhantomLordAverage.png" we made in "AllSpriteSheets (For libGDX Texture Packer)" folder  
		// and specifying the x and y coordinates for each sprite frame from that image (by using (index * sprite_width) for x, and (row_index * sprite_height) for y, 
		// getting that sprite with those coordinates, and then assigning each to the animation array (Can do this with a for loop like shown below), since we 
		// specified "findRegion("PhantomLordAverage")" in the for loop below
		// 
		// So for example, if there was another row (the row index = 1), JUST for the 
		// PhantomLordAverage spritesheet, our index would start at x = 0 for the  new row, and now y = (row index) * 37, while x = (index# for that row) * 35 ALL RELATIVE TO THE 
		// SPRITESHEET OF THAT PARTICULARE REGION!, NOT THE WHOLE SPRITSHEET! the reason index 0 for animation and index 0 for rows is because we starts at top left the spriteSheet
		// we indicated we want to look at for TextureRegion location, and then use width and height (3rd (35) and 4th parameters (35)) to figure out how far to read from starting
		// position (top left coordinate)
		//
		// For Loop for Animation Frames: (ONLY FOR DESIRED REGION)
		//		for (int i = begin_Animation_Index; i < end_Animation_Index + 1; i++)
		//		{
		//			frames.add(new TextureRegion(screen.getAtlas().findRegion("spriteSheetRegion"), currAnimationIndex * sprite_width, rowIndex * sprite_Height, sprite_width, sprite_height)); 
		//																												
		//		}
		
		/*************************************************************************************** 
		* NOTE: Use MediBang Paint for finding/checking spriteSheet region, as it will give me 
		* coordinates I need to double check sprite regions from "PhantomLordAverage" Region, or
		* if you want desired region, open the desired spritesheet in "AllSpriteSheets 
		* (For libGDX Texture Packer)" folder and use MediBang Paint to check
		****************************************************************************************/
		
		/*********************************************************
		 *  Initialize Death Animation for LPhantom Lord Death
		 *********************************************************/
		// Death frame animation at index 6 for "PhantomLordLow" texture region
		for (int i = 6; i < 7; i++)
		{
			frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordLow"), i * 35, 0, 35, 35));
		}
		pLordDead = new Animation<TextureRegion>(0.1f, frames);		
		
		frames.clear();
		
		/*********************************************************
		 *  Initialize Running Animation for LowPhantom Lord
		 *********************************************************/
		//  Add running animation textures to the array of texture regions to create running animation later, running animation at indexes 7-12, at row index 0 (relative to just "PhantomLordLow" coordinates)
		for (int i = 7; i < 13; i++)
		{
			frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordLow"), i * 35, 0, 35, 35));	// Go through each iteration, width and height for each animation is the same, 
																													// use for loop iterative variable to step through and add each frame to animation array
		}
		
		// Create Animation, 1st parameter is duration of each frame (in seconds), then 2nd parameter is array of frames (that we just created above)
		lowPLordRun = new Animation<TextureRegion>(0.1f, frames);						// 0.1f good for time between each frame for running animation
		
		// No longer need these animations inside frames since we assigned them to an animation
		frames.clear();
		
		/*********************************************************
		 *  Initialize Fast Running Animation for LowPhantom Lord
		 *********************************************************/
		//  Add running animation textures to the array of texture regions to create running animation later, running animation at indexes 7-12, at row index 0 (relative to just "PhantomLordLow" coordinates)
		for (int i = 7; i < 13; i++)
		{
			frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordLow"), i * 35, 0, 35, 35));	// Go through each iteration, width and height for each animation is the same, 
																													// use for loop iterative variable to step through and add each frame to animation array
		}
		
		// Create Animation, 1st parameter is duration of each frame (in seconds), then 2nd parameter is array of frames (that we just created above)
		lowPLordFastRun = new Animation<TextureRegion>(0.05f, frames);						// 0.1f good for time between each frame for running animation
		
		// No longer need these animations inside frames since we assigned them to an animation
		frames.clear();
		
		/********************************************************
		 *  Initialize Stance Animation for LowPhantom Lord
		 ********************************************************/
		// Stance is at animation indexes 14-16 for PhantomLordLow region
		for (int i = 14; i < 16; i++)
		{
			frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordLow"), i*35, 0, 35, 35));
		}
		lowPLordStance = new Animation<TextureRegion>(0.4f, frames);					// Need higher time between each frame (seconds) runs too fast if lower
		
		frames.clear();
		
		/*********************************************************
		 *  Initialize Jumping Animation for LowPhantom Lord
		 *********************************************************/
		// Jump is at animation index 13 for PhantomLordLow region
		for (int i = 13; i < 14; i++)
		{
			frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordLow"), i*35, 0, 35, 35));
		}
		lowPLordJump = new Animation<TextureRegion>(0.1f, frames);						// 0.1f good for time between each frame for jumping animation
		
		frames.clear();
		
		/*********************************************************
		 *  Initialize Running Animation for AveragePhantom Lord
		 *********************************************************/
		//  Add running animation textures to the array of texture regions to create running animation later, running animation at indexes 7-12, at row index 0 (relative to just "PhantomLordAverage" coordinates)
		for (int i = 7; i < 13; i++)
		{
			frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordAverage"), i * 35, 0, 35, 35));	// Go through each iteration, width and height for each animation is the same, 
																													// use for loop iterative variable to step through and add each frame to animation array
		}
		// Create Animation, 1st parameter is duration of each frame (in seconds), then 2nd parameter is array of frames (that we just created above)
		averagePLordRun = new Animation<TextureRegion>(0.1f, frames);						// 0.1f good for time between each frame for running animation
		
		// No longer need these animations inside frames since we assigned them to an animation
		frames.clear();
		
		/**************************************************************
		 *  Initialize Fast Running Animation for AveragePhantom Lord
		 **************************************************************/
		//  Add running animation textures to the array of texture regions to create running animation later, running animation at indexes 7-12, at row index 0 (relative to just "PhantomLordAverage" coordinates)
		for (int i = 7; i < 13; i++)
		{
			frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordAverage"), i * 35, 0, 35, 35));	// Go through each iteration, width and height for each animation is the same, 
																													// use for loop iterative variable to step through and add each frame to animation array
		}
		// Create Animation, 1st parameter is duration of each frame (in seconds), then 2nd parameter is array of frames (that we just created above)
		averagePLordFastRun = new Animation<TextureRegion>(0.05f, frames);						// 0.1f good for time between each frame for running animation
		
		// No longer need these animations inside frames since we assigned them to an animation
		frames.clear();
		
		/********************************************************
		 *  Initialize Stance Animation for AveragePhantom Lord
		 ********************************************************/
		// Stance is at animation indexes 14-16 for PhantomLordAverage region
		for (int i = 14; i < 16; i++)
		{
			frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordAverage"), i*35, 0, 35, 35));
		}
		averagePLordStance = new Animation<TextureRegion>(0.4f, frames);					// Need higher time between each frame (seconds) runs too fast if lower
		
		frames.clear();
		
		/*********************************************************
		 *  Initialize Jumping Animation for AveragePhantom Lord
		 *********************************************************/
		// Jump is at animation index 13 for PhantomLordAverage region
		for (int i = 13; i < 14; i++)
		{
			frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordAverage"), i*35, 0, 35, 35));
		}
		averagePLordJump = new Animation<TextureRegion>(0.1f, frames);						// 0.1f good for time between each frame for jumping animation
		
		frames.clear();
		
		/*********************************************************
		 *  Initialize Running Animation for PowerUpPhantom Lord
		 *********************************************************/
		//  Add running animation textures to the array of texture regions to create running animation later, running animation at indexes 7-12, at row index 0 (relative to just "PhantomLordLow" coordinates)
		for (int i = 7; i < 13; i++)
		{
			frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordPower"), i * 35, 0, 35, 35));	// Go through each iteration, width and height for each animation is the same, 
																													// use for loop iterative variable to step through and add each frame to animation array
		}
		
		// Create Animation, 1st parameter is duration of each frame (in seconds), then 2nd parameter is array of frames (that we just created above)
		powerPLordRun = new Animation<TextureRegion>(0.1f, frames);						// 0.1f good for time between each frame for running animation
		
		// No longer need these animations inside frames since we assigned them to an animation
		frames.clear();
		
		/**************************************************************
		 *  Initialize Fast Running Animation for PowerUpPhantom Lord
		 **************************************************************/
		//  Add running animation textures to the array of texture regions to create running animation later, running animation at indexes 7-12, at row index 0 (relative to just "PhantomLordLow" coordinates)
		for (int i = 7; i < 13; i++)
		{
			frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordPower"), i * 35, 0, 35, 35));	// Go through each iteration, width and height for each animation is the same, 
																													// use for loop iterative variable to step through and add each frame to animation array
		}
		
		// Create Animation, 1st parameter is duration of each frame (in seconds), then 2nd parameter is array of frames (that we just created above)
		powerPLordFastRun = new Animation<TextureRegion>(0.05f, frames);						// 0.1f good for time between each frame for running animation
		
		// No longer need these animations inside frames since we assigned them to an animation
		frames.clear();
		
		/********************************************************
		 *  Initialize Stance Animation for PowerUPhantom Lord
		 ********************************************************/
		// Stance is at animation indexes 14-16 for PhantomLordLow region
		for (int i = 14; i < 16; i++)
		{
			frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordPower"), i*35, 0, 35, 35));
		}
		powerPLordStance = new Animation<TextureRegion>(0.4f, frames);					// Need higher time between each frame (seconds) runs too fast if lower
		
		frames.clear();
		
		/*********************************************************
		 *  Initialize Jumping Animation for PowerUPhantom Lord
		 *********************************************************/
		// Jump is at animation index 13 for PhantomLordLow region
		for (int i = 13; i < 14; i++)
		{
			frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordPower"), i*35, 0, 35, 35));
		}
		powerPLordJump = new Animation<TextureRegion>(0.1f, frames);						// 0.1f good for time between each frame for jumping animation
		
		frames.clear();
		
		
		/**********************************************************
		 *  Initialize Acquiring Avg Health Animation 
		 *  from LowPhantom Lord to AveragePhantom Lord
		 *  Basically get the two alternating stances from each
		 *********************************************************/
		// Will get animation stances from both frames and add them to animation frames
		// DO nested for loop, so we have like 16 frames of animation (the inner for loop 
		// provided 4 animations frames, of alternating color stance animation, and want to that to play 4 times in a row
		for (int k = 0; k < 3; k++)
		{
			for (int i = 14; i < 16; i++)
			{
				frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordLow"), i*35, 0, 35, 35));
				frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordAverage"), i*35, 0, 35, 35));
			}
		}
		candyCornAquiredPLord = new Animation<TextureRegion>(0.07f, frames);						// 0.1f good for time between each frame for jumping animation
		
		frames.clear();
		
		/**********************************************************
		 *  Initialize Acquiring PowerUp Health Animation 
		 *  from Low/Avg Phantom Lord to PowerUp PLord
		 *  Basically get the two alternating stances from each
		 *********************************************************/
		// Will get animation stances from both frames and add them to animation frames
		// DO nested for loop, so we have like 16 frames of animation (the inner for loop 
		// provided 4 animations frames, of alternating color stance animation, and want to that to play 4 times in a row
		for (int k = 0; k < 3; k++)
		{
			for (int i = 14; i < 16; i++)
			{
				frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordAverage"), i*35, 0, 35, 35));
				frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordPower"), i*35, 0, 35, 35));
			}
		}
		ghostMaskAquiredPLord = new Animation<TextureRegion>(0.07f, frames);						// 0.1f good for time between each frame for jumping animation
		
		frames.clear();
		
		/*******************************************************************
		 *  Initialize Acquiring Getting hit wile in powerUp Health, Animation 
		 *  from Low/Avg Phantom Lord to PowerUp PLord
		 *  Basically get the two alternating stances from each
		 ******************************************************************/
		// Will get animation stances from both frames and add them to animation frames
		// DO nested for loop, so we have like 16 frames of animation (the inner for loop 
		// provided 4 animations frames, of alternating color stance animation, and want to that to play 4 times in a row
		for (int k = 0; k < 7; k++)
		{
			for (int i = 14; i < 16; i++)
			{
				frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordAverage"), i*35, 0, 35, 35));
				
				frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordAverage"), i*35, 0, 0, 0));
			}
		}
		pLordPowerUpHit = new Animation<TextureRegion>(0.07f, frames);						// 0.1f good for time between each frame for jumping animation
		
		frames.clear();
		
		/*******************************************************************
		 *  Initialize Acquiring Getting hit wile in Avg Uo health, Animation 
		 *  from Low/Avg Phantom Lord to PowerUp PLord
		 *  Basically get the two alternating stances from each
		 ******************************************************************/
		// Will get animation stances from both frames and add them to animation frames
		// DO nested for loop, so we have like 16 frames of animation (the inner for loop 
		// provided 4 animations frames, of alternating color stance animation, and want to that to play 4 times in a row
		for (int k = 0; k < 7; k++)
		{
			for (int i = 14; i < 16; i++)
			{
				frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordLow"), i*35, 0, 35, 35));
				
				frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordLow"), i*35, 0, 0, 0));
			}
		}
		pLordAvgUpHit = new Animation<TextureRegion>(0.07f, frames);						// 0.1f good for time between each frame for jumping animation
		
		frames.clear();
		
		/********************************************************
		 *  Initialize Throwing Cane Animation for PowerUp PLord
		 ********************************************************/
		// throw is at animation indexes 0-5 for PhantomLordAverage region
		for (int i = 0; i < 6; i++)
		{
			frames.add(new TextureRegion(screen.getAtlas().findRegion("PhantomLordPower"), i*35, 0, 35, 35));
		}
		throwCane = new Animation<TextureRegion>(.075f, frames);					// Need higher time between each frame (seconds) runs too fast if lower
		
		frames.clear();
		
		// Define body for PhamtomLord and assign body to world
		definePhantomLord();
		
		/************************************************************************
		 * Define Starting texture for Phantom Lord (this will be updated to the 
		 * correct animation during the update method, since we in constructor, 
		 * need to set bounds and such for the texture (NOTE NOT NEEDED IF WE
		 * define animations beforehand!)
		 ************************************************************************/
		
		/*************************************************************************************** 
		* NOTE: Don't actually need to specify starting texture, if we define all animations, 
		* the update method will correctly display the texture only need to define starting 
		* position and size of sprite using the "setBounds" function below
		****************************************************************************************/
		
		// define starting position (1st and 2nd parameter) and size (3rd and 4th parameter) of Phantom Lord sprite using the "setBounds" function below
		// 20X25 seems like good size for PLord "in-game" (remember we have to divide by PIXEL_PER_METER for size)
		setBounds(0 / PhantomLord.PIXELS_PER_METER, 0 / PhantomLord.PIXELS_PER_METER, 
				20 / PhantomLord.PIXELS_PER_METER, 25 / PhantomLord.PIXELS_PER_METER);
		
		keyPressedTime = 99.0f;
	}
	
	public void update(float dt)
	{
		caneTimer += dt;
		
		/****************************************************************
		 *  Set sprite to position to box2DBody
		 *	box2D body is in center of fixture
		 *  want coordinate system of bottom left hand corner of fixture
		 ****************************************************************/
		
		// GetPosition.x gives center point of fixture, getWidth gives width of sprite, divide it by 2
		
		// Set new position of sprite on screen, get x and y position of the box2DBody, then subtract the 
		// x and y by HALF of the sprite's width and height, respectively, to move sprite to center position of
		// box2DBody
		// Note: I added "+ (4 / PhantomLord.PIXELS_PER_METER)" to vertical (y) position cause PLord sprite seemed slightly sunk into the ground, 
		//		so I moved him up by 4 pixels relative to his box2DBody
		setPosition(box2DBody.getPosition().x - getWidth() / 2, box2DBody.getPosition().y - getHeight() / 2 + (3 / PhantomLord.PIXELS_PER_METER));
		
		// Set Our animation texture and position
		setRegion(getFrame(dt));
		
		keyPressedTime += dt;
		
        if (caneTimer > 2f) {
            runThrowingAnimation = false;
        }
		
        // die when falling below ground
        if (box2DBody.getPosition().y < 0.0f) 
        {
        	pLordDie();
        }
		
		// If our time is up and Plord is not already dead
		if(screen.getHud().isTimeUp() && !isPLordDead())
		{
			pLordDie();
		}
		// If at the goal, just move PLord Forward
		else if(pLordAtGoal)
		{
			// Make PLord come to dead halt and fall down when colliding with goal area
			box2DBody.setLinearVelocity(0f, -2f);
			
			// If half a second has passed since PLord landed on Goal
			if(stateTimer > .5f)
				box2DBody.setLinearVelocity(velocity);		// Set the velocity of PLord's box2dBody
		}
		// We reach end of level, make PLord disappear
		else if(pLordFinished)
		{
			// Remove the pLord box2DBody (need to override draw mehtod to remove sprite, method "draw" is in this class)
			world.destroyBody(box2DBody);
		}
		// PLord got hit (didnt die, but got hit)
		else if(pLordGotHit)
		{
			
			/********************************************************************************************
			 * Disable collision on Body of PLord for a while so enemy doesnt push him when he gets hurt
			 ********************************************************************************************/
			// Create new box2D Filter for pLord
			filter = new Filter();
			// Want Body to not collide with enemy for a while, make collidable with everything besides ENEMY_BIT and ENEMYHEAD_BIT
			filter.maskBits = PhantomLord.OBJECT_BIT | 
					PhantomLord.ITEM_BIT |
					PhantomLord.EVENT_BIT | PhantomLord.NOTHING_BIT;
			// Get all the current fixtures for PLord's box2DBody and set them to the filter we just made
			for(Fixture fixture : box2DBody.getFixtureList())
				fixture.setFilterData(filter);
			
			// If after we got hit we have been invincible (and enemies couldn't push/collide with us) for 1 second,
			//go back to being able to get hit
			if(stateTimer > 1f)
			{
				pLordGotHit = false;
				
				/********************************************************************************************
				 * Enable collision on Body of PLord for enemy so he can get hit
				 ********************************************************************************************/
				reDefinePhantomLord();
			}
		}
	}
	
	/*
	* Idea
	* https://gamedev.stackexchange.com/questions/80908/libgdx-how-to-change-animations
	*/
	// Return appropriate frame we need to display as sprite texture region
	public TextureRegion getFrame(float dt)
	{
		// What state is pLord in?
		currentState = getState();
		
		// Define variable to hold texutre resgions for animations
		TextureRegion region;
		
		// If true, stateTimer = stateTimer + dt, otherwise stateTimer = 0
		// aka, If it doesn't equal to previous state, it must have transition to new state and need to reset the timer
		// THIS NEED TO GO BEFORE SWITCH STATMENT, as switch statement depends on its current ifno
		// Thus we will have smooth, fast transitions between animations
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
		case DEAD:
			region = pLordDead.getKeyFrame(stateTimer);
			break;
		case GOTHIT:
			if(isPLordAverage())									// By this point, pLord health is already down if he is in powerUp state, so if he is not already in average state, then he is not losing health from power Up state
				region = pLordPowerUpHit.getKeyFrame(stateTimer);	    // NOT passing true, because animation is not supposed to be loopable
			else													// otherwise, since pLord didn't die, he was in avgerage health state, now low health
				region = pLordAvgUpHit.getKeyFrame(stateTimer);
			break;
		case AVGUP:
			region = candyCornAquiredPLord.getKeyFrame(stateTimer);	// NOT passing true, because animation is not supposed to be loopable
			if(candyCornAquiredPLord.isAnimationFinished(stateTimer))	// IF animation is finished, no longer in runHealthAcquireAnimation state
				runCandyCornAcquireAnimation = false;
			break;
		case POWERUP:
			region = ghostMaskAquiredPLord.getKeyFrame(stateTimer);	// NOT passing true, because animation is not supposed to be loopable
			if(ghostMaskAquiredPLord.isAnimationFinished(stateTimer))	// IF animation is finished, no longer in runHealthAcquireAnimation state
				runGhostMaskAcquireAnimation = false;
			 break;
		case THROW:
			region = throwCane.getKeyFrame(stateTimer);	// NOT passing true, because animation is not supposed to be loopable
			if(throwCane.isAnimationFinished(stateTimer))	// IF animation is finished, no longer in runHealthAcquireAnimation state
			{
				int x = runningRight ? 15 : -15;
				int y = 5;
				screen.addSpawnCane(box2DBody.getPosition().x + x / PhantomLord.PIXELS_PER_METER, box2DBody.getPosition().y + y / PhantomLord.PIXELS_PER_METER, runningRight);
				runThrowingAnimation = false;
	            PhantomLord.manager.get("assets/audio/sounds/CaneThrow.mp3", Sound.class).play();
			}
			break;
		case JUMPING:
			if(pLordIsAverage)
				region = averagePLordJump.getKeyFrame(stateTimer, true);
			else if(pLordIsPowerUp)
				region = powerPLordJump.getKeyFrame(stateTimer, true);
			else
				region = lowPLordJump.getKeyFrame(stateTimer, true);	
			break;							
		//case GOALREACHED:	// Want pLord to appear as though he is moving when level is complete
		case GOALREACHED:	// When goal is reached, want Plord to do same animation as running
		case RUNNING:		
			// If PLord is average, run 'average' running animation, run 'low' running animation otherwise, remember to return true to make animation loopable
			if(pLordIsAverage)
				region = averagePLordRun.getKeyFrame(stateTimer, true);
			else if(pLordIsPowerUp)
				region = powerPLordRun.getKeyFrame(stateTimer, true);
			else
				region = lowPLordRun.getKeyFrame(stateTimer, true);	
			break;				
		case FASTRUNNING:
			// If PLord is average, run 'average' running animation, run 'low' running animation otherwise, remember to return true to make animation loopable
			if(pLordIsAverage)
				region = averagePLordFastRun.getKeyFrame(stateTimer, true);
			else if(pLordIsPowerUp)
				region = powerPLordFastRun.getKeyFrame(stateTimer, true);
			else
				region = lowPLordFastRun.getKeyFrame(stateTimer, true);	
			break;
		case FALLING:									
		case STANDING:
		default:
			if(pLordIsAverage)
				region = averagePLordStance.getKeyFrame(stateTimer, true);
			else if(pLordIsPowerUp)
				region = powerPLordStance.getKeyFrame(stateTimer, true);
			else
				region = lowPLordStance.getKeyFrame(stateTimer, true);	
			break;						
		}
		
		/***********************************************************************************************
		 * If Phantom Lord is standing still, need to determine which way he should face left or right?
		 ***********************************************************************************************/
		
		// If PLord is running to the left and the region isn't facing left
		// region.isFlipX returns true if the texture is flipped over (pLord would be facing left, then it would be true)
		if((box2DBody.getLinearVelocity().x < 0 || !runningRight) && !region.isFlipX())
		{
			region.flip(true, false);		// Only want to flip x axis, not y (don't want him upside down)
			runningRight = false;			// PLord is running left now (as we turned him to the left)
		}
		// If PLord is running to the right and the region is facing left
		else if ((box2DBody.getLinearVelocity().x > 0 || runningRight) && region.isFlipX())
		{
			region.flip(true, false);  		// Only want to flip x axis, not y (don't want him upside down)
			runningRight = true;
		}
		
		// Return the texture region
		return region;
	}
	
	/*
	 * Beginning code/idea from
	 * https://github.com/Suvitruf/libgdx/blob/master/Libgdxtutorial-lesson2.1/src/suvitruf/libgdxtutorial/lesson2_1/model/Player.java
	 */
	public State getState()
	{
		/*******************************************************
		 * Base state off of what box2DBody is currently doing
		 *******************************************************/
		
		// 1st check if pLord is dead
		if(pLordIsDead)
			return State.DEAD;
		// PLord has reached the goal
		else if(pLordAtGoal)
			return State.GOALREACHED;
		// PLord has completed the level
		else if(pLordFinished)
			return State.LEVELFINISHED;
		// PLord getting health animation trumps everything else up to this point
		else if(runCandyCornAcquireAnimation)
			return State.AVGUP;
		else if(runGhostMaskAcquireAnimation)
			return State.POWERUP;
		// if PLord had enough health to get hit and not die
		else if(pLordGotHit)
			if(pLordIsPowerUp)
				return State.GOTHIT;
			else 						// else Plord is average health
				return State.GOTHIT;
		else if(runThrowingAnimation)
			return State.THROW;
		// pLord is jumping
		// So if he is falling but was previously jumping up right before he started to fall, then continue with the jumping animation
		else if(box2DBody.getLinearVelocity().y > 0 || (box2DBody.getLinearVelocity().y < 0 && previousState == State.JUMPING))
		{
			return State.JUMPING;
		}
		// Plord is falling (issue with falling, being be tied to linear velocity being tied to less than 0 on y axis
		//, if he is running off edge and falls want different animation than jumping just want to display stance animation,
		// need OR statement to make that work in the jumping if statement above)
		else if(box2DBody.getLinearVelocity().y < 0)
		{
			return State.FALLING;
		}
		// Moving to the left or right
		else if(box2DBody.getLinearVelocity().x != 0)
		{	
			if(box2DBody.getLinearVelocity().x > 0)
			{
				if(box2DBody.getLinearVelocity().x < normalSpeedMax + .5f)
					return State.RUNNING;
				else
					return State.FASTRUNNING;
			}
			else
			{
				if(box2DBody.getLinearVelocity().x > -normalSpeedMax - .5f )
					return State.RUNNING;
				else
					return State.FASTRUNNING;
			}
		}
		else
			return State.STANDING;
	}
	
	public void pLordAvgHealth()
	{
		// We have acquired the CandyCorn, health goes up
		runCandyCornAcquireAnimation = true;
		pLordIsAverage = true;
		
		PhantomLord.pLordState = 2;			// to avg health
		
		// Play Power Up sound
		PhantomLord.manager.get("assets/audio/sounds/PowerUp.mp3", Sound.class).play();
	}
	
	public void pLordPowerUpHealth()
	{
		// We have acquired the CandyCorn, health goes up
		runGhostMaskAcquireAnimation = true;
		pLordIsPowerUp = true;
		pLordIsAverage = false;
		
		PhantomLord.pLordState = 3;			// to power up health
		
		// Play Power Up sound
		PhantomLord.manager.get("assets/audio/sounds/PowerUp.mp3", Sound.class).play();
	}
	
	public boolean isPLordDead()
	{
		return pLordIsDead;
	}
	
	public float getStateTimer()
	{
		return stateTimer;
	}
	
	public boolean isPLordAverage()
	{
		return pLordIsAverage;
	}
	
	public boolean isPLordPower()
	{
		return pLordIsPowerUp;
	}
	
	/*
	 * Beginning code/idea from
	 * https://github.com/Suvitruf/libgdx/blob/master/Libgdxtutorial-lesson2.1/src/suvitruf/libgdxtutorial/lesson2_1/model/Player.java
	 */
	public void pLordDie()
	{
		// If pLord isn't already dead, kill him
		if(!isPLordDead())
		{
			switch(PhantomLord.LevelNumber)
			{
				// Level 1
				case 1:
					// Stop the background music
					PhantomLord.manager.get("assets/audio/music/Menu-Upbeat.mp3", Music.class).stop();
					break;
				// Level 2
				case 2:
					PhantomLord.manager.get("assets/audio/music/Love Upon Differences.mp3", Music.class).stop();
					break;
				// Level 3
				case 3:
					PhantomLord.manager.get("assets/audio/music/Intense(Compressed).mp3", Music.class).stop();
					break;
				// Otherwise level 4
				default:
					// Stop the background music
					PhantomLord.manager.get("assets/audio/music/Winter waltz.mp3", Music.class).stop();
					break;
			}
			
			// Play GameOver sound
			PhantomLord.manager.get("assets/audio/sounds/GameOver.mp3", Sound.class).play();
			
			// Set pLordDead state to true
			pLordIsDead = true;
			
			// Want Plord to fall through ground, make Plord NOT COLLIDE with anything
			// create new filter
			Filter filter = new Filter();
			// Set it to the NOTHING BIT
			filter.maskBits = PhantomLord.NOTHING_BIT;
			// Set all filters on PLORD to NOTHING_BIT, so he collides with nothing
			// For every fixture in box2DBody, set current filter data to the filter we set above
			for(Fixture fixture : box2DBody.getFixtureList())
				fixture.setFilterData(filter);
			
			// Make PLord jump up a little bit, then fall down through the ground
			// Applies an impulse in a certain direction, apply the Vector2 impulse 
			//to the center of mass of pLord body, and wake up body if asleep (should never be, but still will put as true)
			box2DBody.applyLinearImpulse(new Vector2(0, 4f), box2DBody.getWorldCenter(), true);
		}
	}
	
	/*
	 * Beginning code/idea from
	 * https://github.com/Suvitruf/libgdx/blob/master/Libgdxtutorial-lesson2.1/src/suvitruf/libgdxtutorial/lesson2_1/model/Player.java
	 */
	public void pLordHit(Enemy enemy)
	{
		// Don't want pLord to die when hitting the HatGhost when its hiding in the hat
		if(enemy instanceof HatGhost && ((HatGhost) enemy).getCurrentState() == HatGhost.State.STATIC_HIDEINHAT)
		{
			// pLord is the the left, need to move hat to the right, else vice versa
			((HatGhost) enemy).hitHat(this.getX() <= enemy.getX() ? HatGhost.HIT_RIGHT_SPEED : HatGhost.HIT_LEFT_SPEED);
		}
		// PLord will get hurt
		else
		{
			// If pLord is average, then we just lose health and are not "average" health anymore
			if(pLordIsAverage)
			{
				// Keep track of how long PLord has been hit for, about 2 seconds invicibility once hit
				stateTimer = 0;
				
				// Plord had enough health to got hurt
				pLordGotHit = true;
				
				// But PLord was average and got hurt, so not average health anymore
				pLordIsAverage = false;
				
				pLordIsPowerUp = false;
				
				PhantomLord.pLordState = 1;			// Back to low health
				
				// Play Power Down sound
				PhantomLord.manager.get("assets/audio/sounds/PLordHitPowerDown.mp3", Sound.class).play();
			}
			// If pLord is average, then we just lose health and are not "average" health anymore
			else if(pLordIsPowerUp)
			{
				// Keep track of how long PLord has been hit for, about 2 seconds invicibility once hit
				stateTimer = 0;
				
				// Plord had enough health to got hurt
				pLordGotHit = true;
				
				// But PLord was average and got hurt, so not power up anymore
				pLordIsPowerUp = false;
				
				// Now average health
				pLordIsAverage = true;
				
				PhantomLord.pLordState = 2;			// Back to average health
				
				// Play Power Down sound
				PhantomLord.manager.get("assets/audio/sounds/PLordHitPowerDown.mp3", Sound.class).play();
			}
			// Plord is low, and since health at low state needs to die now
			else
			{
				// Call pLordDie() method, which will kill pLord
				pLordDie();
			}
		}
	}
	
	// Finish the Level for PLord (this is called each time we hit a "Goal" object, we have two one that is the "goal" (property)
	// and another that is the "door"(property)
	public void pLordFinishedLevel(MapObject object)
	{
		// Make sure PLord is facign right
		runningRight = true;
		
		// Reset state timer, so we know how much time has passed since pLord hit goal
		if(pLordAtGoal == false)
			stateTimer = 0;
		
		// pLord has hit end goal (STOP USER INPUT, by setting new state for pLord, this is done in the Play screen)
		if(object.getProperties().containsKey("goal"))
		{	
			switch(PhantomLord.LevelNumber)
			{
				// Level 1
				case 1:
					// Stop the background music
					PhantomLord.manager.get("assets/audio/music/Menu-Upbeat.mp3", Music.class).stop();
					break;
				// Level 2
				case 2:
					PhantomLord.manager.get("assets/audio/music/Love Upon Differences.mp3", Music.class).stop();
					break;
				// Level 3
				case 3:
					PhantomLord.manager.get("assets/audio/music/Intense(Compressed).mp3", Music.class).stop();
					break;
				// Otherwise level 4
				default:
					// Stop the background music
					PhantomLord.manager.get("assets/audio/music/Winter waltz.mp3", Music.class).stop();
					break;
			}
			
			if(!pLordAtGoal)
			{
				// Play LevelComplete Sound
				PhantomLord.manager.get("assets/audio/sounds/CompleteLevel.mp3", Sound.class).play();
			}
			
			// Boolean to set state of PLord to "Goal Reached"
			pLordAtGoal = true;
			
			// If we haven't reached the door,  manually move pLord
			if (object.getProperties().containsKey("door") != true)
			{
					// Define velcoity for PLord to move to the right, till he reaches the door at end to disappear, to finish the level,
					// PLord will be controlled without user input
					velocity = new Vector2(1.2f, -1.7f);
			}
		}
		// else if we have reached door now
		else if(object.getProperties().containsKey("door"))
		{
			// Don't want to keep moving, as we made it to door 
			pLordAtGoal = false;
			
			// Now we are finished
			pLordFinished = true;
		}
	}
	
	// make sure sprite is remove when PLord dies or finishes level
	public void draw(Batch batch)
	{
		// if (PLord is not dead or the stateTimer hasen't gone past 3 seconds (when PLord IS dead) )
		// AND PLord has finished the level
		if((currentState != State.DEAD || stateTimer < 3) && currentState != State.LEVELFINISHED)
			super.draw(batch);
	}
	
	/*
	* statring code/idea from
	* http://www.iforce2d.net/b2dtut/bodies
	* &
	* http://www.gamefromscratch.com/post/2014/08/27/LibGDX-Tutorial-13-Physics-with-Box2D-Part-1-A-Basic-Physics-Simulations.aspx
	*/
	public void definePhantomLord()
	{
		// Define body for PhamtomLord and assign body to world
		BodyDef bDef = new BodyDef();
		
		// 544 pixels (34 tiles 16x16 before ground begins) before ground, make SURE PLord is placed at lest 545 pixels, or he will start off falling and dieing
		// WHERE PLORD WILL BE SPAWNED
		// x = 700 is GOOD starting position if 34 tiles before ground starts (x = 4700 = end of stage)
		bDef.position.set(700 / PhantomLord.PIXELS_PER_METER, 32 / PhantomLord.PIXELS_PER_METER);		// Position of the box2DBody, which is important, as the sprite will go to this location
		bDef.type = BodyDef.BodyType.DynamicBody;														// So that the sprite/animation stays in line with the actual physics collider
		box2DBody = world.createBody(bDef);																// Also, when we move PLord, the box2DBody gets moved to correspond to that, so the Spite 
																										// needs to be rendered wherever the box2DBody is
		// Define fixture for PhantomLord (make radius appropriate size that matches the PLord sprite)													
		// 7 is good radius, BUT make sure PLord has head and foot bodies as well, since its a circle and
		// his body shape is rectangular
		FixtureDef fDef = new FixtureDef();
		CircleShape shape = new CircleShape();
		shape.setRadius(6 / PhantomLord.PIXELS_PER_METER);
		
		/***************************************************************************************************************************
		* Set the category and what PLord can collide with for fixture filters (see PhantomLord.java for more info)
		* Can assign this in fixture definition (as I am doing) or could also do it once it is already a fixture,
		*		but do it before creating any fixture if you don't it right after fixtureDef
		* ALSO: since same fDef for body of PLord, feet, and head, don't have to worry about setting this filter anywhere else here
		*****************************************************************************************************************************/
		// Category for PhantomLord body, aka, what is this fixture's category
		fDef.filter.categoryBits = PhantomLord.PLORD_BIT;
		// What can Phantom Lord collide with
		fDef.filter.maskBits = PhantomLord.OBJECT_BIT | PhantomLord.ENEMY_BIT |
				PhantomLord.ENEMYHEAD_BIT | PhantomLord.ITEM_BIT |  
				PhantomLord.EVENT_BIT | PhantomLord.NOTHING_BIT;
		// Set this fixture to our body for Phantom Lord and setUserData for WorldContactListera
		fDef.shape = shape;
		box2DBody.createFixture(fDef).setUserData(this);
		
		// Fix issue where Phantom Lord will bounce in between blocks (gives upward velocity originally) ground doesn't have this issue
		// Create sensor for Plord's feet (since using safDef, this is considered to be part of PLORD_BITategory filter)
		// Also allows it so the main box2DBody doesn't have to be hitting with ground, but instead how many spaces from the feet (which
		// will touch the ground now),where the 'feet' we put -7 pixels from the main body (as shown below in both Vector2's 2nd parameter)
		// This will make the main body in a good position relative to collision detection as well
		// Since didn't change fDef, feet are considered part of "PLORD_BIT"
		EdgeShape feet = new EdgeShape();
		// Coordinates relative to PLord body, edgeShape acts as feet to glide across tiles, make feet -4 to 4, for 8 pixles length, seemss to 
		feet.set(new Vector2(-4 / PhantomLord.PIXELS_PER_METER, -7 / PhantomLord.PIXELS_PER_METER),
				new Vector2(4 / PhantomLord.PIXELS_PER_METER, -7 / PhantomLord.PIXELS_PER_METER));
		fDef.shape = feet;
		// need to put .setUserData, otherwise feet body won't send the PhantomLordChar class type to world contact listener
		box2DBody.createFixture(fDef).setUserData(this);
		
		// Since didn't change fDef, feet are considered part of "PLORD_BIT"
		// Create sensor on PLord's head (EdgeShape is line between two different points)
		EdgeShape head = new EdgeShape();
		// Coordinates relative to PLord body, edgeShape acts as head (make -2 and 2, so length of line is fairly short, so we don't hit more than 
		// 		one object at a time., put y position at 10, just right at top of PLord's head
		head.set(new Vector2(-2 / PhantomLord.PIXELS_PER_METER, 10 / PhantomLord.PIXELS_PER_METER),
				new Vector2(2 / PhantomLord.PIXELS_PER_METER, 10 / PhantomLord.PIXELS_PER_METER));
		fDef.filter.categoryBits = PhantomLord.PLORDHEAD_BIT;
		fDef.shape = head;
		// When you create fixture definition that is sensor, it no longer collides with anything in the world, used to query if anything in box2DWorld, so assign this
		//fDef.isSensor = true;			// Not going to use it as sensor, as PLord needs good head body, since his circle body isn't long enough
		// Uniquely define head fixture as "head"
		box2DBody.createFixture(fDef).setUserData(this);
	}
	
	public void reDefinePhantomLord()
	{
		// Determine current position of box2DBody, so can assign new one to it
		Vector2 currentPosition = box2DBody.getPosition();
		
		// Destroy old body
		world.destroyBody(box2DBody);
		
		// Redefine body for PhamtomLord and assign body to world
		BodyDef bDef = new BodyDef();
		
		// Set the position of the redefined body, from the position of the last body
		bDef.position.set(currentPosition);					// Position of the box2DBody, which is important, as the sprite will go to this location
		bDef.type = BodyDef.BodyType.DynamicBody;														// So that the sprite/animation stays in line with the actual physics collider
		box2DBody = world.createBody(bDef);																// Also, when we move PLord, the box2DBody gets moved to correspond to that, so the Spite 
																										// needs to be rendered wherever the box2DBody is
		// Define fixture for PhantomLord (make radius appropriate size that matches the PLord sprite)													
		// 7 is good radius, BUT make sure PLord has head and foot bodies as well, since its a circle and
		// his body shape is rectangular
		FixtureDef fDef = new FixtureDef();
		CircleShape shape = new CircleShape();
		shape.setRadius(6 / PhantomLord.PIXELS_PER_METER);
		
		/***************************************************************************************************************************
		* Set the category and what PLord can collide with for fixture filters (see PhantomLord.java for more info)
		* Can assign this in fixture definition (as I am doing) or could also do it once it is already a fixture,
		*		but do it before creating any fixture if you don't it right after fixtureDef
		* ALSO: since same fDef for body of PLord, feet, and head, don't have to worry about setting this filter anywhere else here
		*****************************************************************************************************************************/
		// Category for PhantomLord body, aka, what is this fixture's category
		fDef.filter.categoryBits = PhantomLord.PLORD_BIT;
		// What can Phantom Lord collide with
		fDef.filter.maskBits =  PhantomLord.OBJECT_BIT | PhantomLord.ENEMY_BIT |
				PhantomLord.ENEMYHEAD_BIT | PhantomLord.ITEM_BIT |  
				PhantomLord.EVENT_BIT | PhantomLord.NOTHING_BIT;
		// Set this fixture to our body for Phantom Lord and setUserData for WorldContactListera
		fDef.shape = shape;
		box2DBody.createFixture(fDef).setUserData(this);
		
		// Fix issue where Phantom Lord will bounce in between blocks (gives upward velocity originally) ground doesn't have this issue
		// Create sensor for Plord's feet (since using safDef, this is considered to be part of PLORD_BITategory filter)
		// Also allows it so the main box2DBody doesn't have to be hitting with ground, but instead how many spaces from the feet (which
		// will touch the ground now),where the 'feet' we put -7 pixels from the main body (as shown below in both Vector2's 2nd parameter)
		// This will make the main body in a good position relative to collision detection as well
		// Since didn't change fDef, feet are considered part of "PLORD_BIT"
		EdgeShape feet = new EdgeShape();
		// Coordinates relative to PLord body, edgeShape acts as feet to glide across tiles, make feet -4 to 4, for 8 pixles length, seemss to 
		feet.set(new Vector2(-4 / PhantomLord.PIXELS_PER_METER, -7 / PhantomLord.PIXELS_PER_METER),
				new Vector2(4 / PhantomLord.PIXELS_PER_METER, -7 / PhantomLord.PIXELS_PER_METER));
		fDef.shape = feet;
		// need to put .setUserData, otherwise feet body won't send the PhantomLordChar class type to world contact listener
		box2DBody.createFixture(fDef).setUserData(this);
		
		// Since didn't change fDef, feet are considered part of "PLORD_BIT"
		// Create sensor on PLord's head (EdgeShape is line between two different points)
		EdgeShape head = new EdgeShape();
		// Coordinates relative to PLord body, edgeShape acts as head (make -2 and 2, so length of line is fairly short, so we don't hit more than 
		// 		one object at a time., put y position at 10, just right at top of PLord's head
		head.set(new Vector2(-2 / PhantomLord.PIXELS_PER_METER, 10 / PhantomLord.PIXELS_PER_METER),
				new Vector2(2 / PhantomLord.PIXELS_PER_METER, 10 / PhantomLord.PIXELS_PER_METER));
		fDef.filter.categoryBits = PhantomLord.PLORDHEAD_BIT;
		fDef.shape = head;
		// When you create fixture definition that is sensor, it no longer collides with anything in the world, used to query if anything in box2DWorld, so assign this
		//fDef.isSensor = true;			// Not going to use it as sensor, as PLord needs good head body, since his circle body isn't long enough
		// Uniquely define head fixture as "head"
		box2DBody.createFixture(fDef).setUserData(this);
	}
}
