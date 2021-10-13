package edu.uidaho.phantomlord.Screens;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.Screen;
import com.badlogic.gdx.audio.Music;
import com.badlogic.gdx.audio.Sound;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.Animation;
import com.badlogic.gdx.graphics.g2d.BitmapFont;
import com.badlogic.gdx.graphics.g2d.Sprite;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.g2d.TextureAtlas;
import com.badlogic.gdx.graphics.g2d.TextureRegion;
import com.badlogic.gdx.scenes.scene2d.Actor;
import com.badlogic.gdx.scenes.scene2d.Stage;
import com.badlogic.gdx.scenes.scene2d.ui.Table;
import com.badlogic.gdx.scenes.scene2d.ui.TextButton;
import com.badlogic.gdx.scenes.scene2d.ui.TextButton.TextButtonStyle;
import com.badlogic.gdx.scenes.scene2d.utils.ChangeListener;
import com.badlogic.gdx.scenes.scene2d.utils.ChangeListener.ChangeEvent;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.viewport.FitViewport;
import com.badlogic.gdx.utils.viewport.Viewport;

import edu.uidaho.phantomlord.PhantomLord;
import edu.uidaho.phantomlord.Tools.TextureLoader;

/*
 * Code Inspired from
 * https://github.com/libgdx/libgdx/wiki/Extending-the-simple-game
 * "The Main Main" sectio
 */
public class InfoMenu2 implements Screen
{
	// Background for menu
	private Texture background;
	
	// Reference to game
	private PhantomLord game;
	
	private Viewport viewport;
	private Stage stage;
	
	// Keep track of time we are in any given state
	private float stateTimer;	
	private float updateTimer;	
	
	// Camera
	protected OrthographicCamera camera;
	
	private final int titleWidth = 500;
	private final int titleHeight = 150;
	private final int buttonFontSize = 25;
	private final float buttonWidth = 100;
	private final float buttonHeight = 35;
	
	// Load images from texture atlases using texture packer
	private TextureAtlas atlas;
	private TextureAtlas atlasPLordEnem;
	
	// sprites
	protected SpriteBatch spriteBatch;
	
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
	private Animation<TextureRegion> floatAnimation;
	private Animation<TextureRegion> hideInHatStaticAnimation;
	private Animation<TextureRegion> hideInHatMovingAnimation;
	private Animation<TextureRegion> bounceAnimation;
	private Animation<TextureRegion> flyAnimation;
	private Animation<TextureRegion> caneSpinning;
	private Animation<TextureRegion> floatCandleAnimation;
	private Animation<TextureRegion> pumpkinHitAnimation;
	
	private Sprite gameTitle;
	private Sprite ghostStance;
	private Sprite pumpkinStance;
	private Sprite batStance;
	private Sprite caneSpin;
	private Sprite ghostHideInHat;
	private Sprite pumpkinHit;
	private Sprite pLordFastRun;
	private Sprite pLordThrow;
	private Sprite candle;
	private Sprite candyCorn;
	private Sprite hatCandy;
	private Sprite ghostMask;
	private Table buttonContainer, buttonContainer2, buttonContainer3;
	private TextButton exitInfoMenuButton, backButton, nextButton;
	
	// Music variable for setting up music to play in game
	private Music music;
	
	// Constructor
	public InfoMenu2(PhantomLord game)
	{
		PhantomLord.currentInfoMenuScreen = 2;
		
		// assign PhantomLord game instance to the class
		this.game = game;
		
		// Create an array of texture regions to pass the constructor for the animation
		Array<TextureRegion> frames = new Array<TextureRegion>();
		
		atlasPLordEnem = new TextureAtlas(Gdx.files.internal("assets/actors/PhantomLord_and_Enemies.pack").file().getAbsolutePath());
		
		viewport = new FitViewport(PhantomLord.V_WIDTH, PhantomLord.V_HEIGHT, new OrthographicCamera());
		
		// Set stage (avoid having to create new batch by using old one)
		stage = new Stage(viewport, ((PhantomLord) game).batch);
		
		// assign PhantomLord game instance to the class
		this.game = game;
		
		atlas = new TextureAtlas(Gdx.files.internal("assets/textures.atlas").file().getAbsolutePath());
		
		// Method to make camera (how speeecial)
		createCamera();
		
		viewport = new FitViewport(PhantomLord.V_WIDTH, PhantomLord.V_HEIGHT, new OrthographicCamera());
		
		// Set stage (avoid having to create new batch by using old one)
		stage = new Stage(viewport, ((PhantomLord) game).batch);
		
		// Assign backgorund texture for info menu
		background = new Texture(Gdx.files.internal("assets/menu/PhantomLord_How_To_Play_Screen2.png").file().getAbsolutePath());
	
		// Set up Music (Remember using static, IT MIGHT GIVE ERRORS, when testing if it does, fix it, otherwise don't worry)
		// Load in 'Intense(Compressed).mp3', the music for level 1, loop it and play it
		music = PhantomLord.manager.get("assets/audio/music/Spirit Waltz(Menu) (Compressed).mp3", Music.class);
		music.setLooping(true); 				// Loop the song
		music.play(); 	
		
		/* Set up the sprites for Animations of PLord on Screen */
		pumpkinStance = new Sprite(new TextureRegion(atlasPLordEnem.findRegion("Pumpkin"), 1*35, 0 * 37, 35, 35));
		pumpkinStance.setBounds(50, 140, 20, 20);
		ghostStance = new Sprite(new TextureRegion(atlasPLordEnem.findRegion("GhostWithHat"), 0*35, 0 * 37, 35, 35));
		ghostStance.setBounds(50, 120, 20, 20);
		batStance = new Sprite(new TextureRegion(atlasPLordEnem.findRegion("Bat"), 0*35, 0 * 37, 35, 35));
		batStance.setBounds(50, 100, 20, 20);
		ghostHideInHat = new Sprite(new TextureRegion(atlasPLordEnem.findRegion("GhostWithHat"), 5*35, 0 * 37, 35, 35));
		ghostHideInHat.setBounds(207, 140, 20, 20);
		caneSpin = new Sprite(new TextureRegion(atlasPLordEnem.findRegion("CanePower"), 0*35, 0 * 37, 35, 35));
		caneSpin.setBounds(207, 120, 20, 20);
		pumpkinHit = new Sprite(new TextureRegion(atlasPLordEnem.findRegion("Pumpkin"), 0*35, 0 * 37, 35, 35));
		pumpkinHit.setBounds(207, 105, 20, 20);
		candle = new Sprite(new TextureRegion(atlasPLordEnem.findRegion("Candle"), 0*35, 0 * 37, 35, 35));
		candle.setBounds(208.5f, 82, 17, 17);
		candyCorn = new Sprite(new TextureRegion(atlasPLordEnem.findRegion("CandyCorn"), 0*35, 0 * 37, 35, 35));
		candyCorn.setBounds(50, 80, 20, 20);
		hatCandy = new Sprite(new TextureRegion(atlasPLordEnem.findRegion("HatCandy"), 0*35, 0 * 37, 35, 35));
		hatCandy.setBounds(50, 85, 20, 20);
		ghostMask = new Sprite(new TextureRegion(atlasPLordEnem.findRegion("GhostMask"), 0*35, 0 * 37, 35, 35));
		ghostMask.setBounds(50, 80, 20, 20);
		
		stateTimer = 0;
		updateTimer = 0;
		
		// Method to help make main menu
		//createGameTitle();
		createButtons();
		createButtonsListeners();
		createButtonsContainer();
		
		// Set up the spritebatch
		spriteBatch = new SpriteBatch();

		stage.addActor(buttonContainer);
		// Don't make "back" button if on first screen
		if(PhantomLord.currentInfoMenuScreen != 1)
			stage.addActor(buttonContainer2);
		// Don;t make "next" button" if on last screen
		if(PhantomLord.currentInfoMenuScreen != 2)
			stage.addActor(buttonContainer3);
		
		
		/*********************************************************
		 *  Initialize Death Animation for LPhantom Lord Death
		 *********************************************************/
		// Death frame animation at index 6 for "PhantomLordLow" texture region
		for (int i = 6; i < 7; i++)
		{
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordLow"), i * 35, 0, 35, 35));
		}
		pLordDead = new Animation<TextureRegion>(0.1f, frames);		
		
		frames.clear();
		
		/*********************************************************
		 *  Initialize Running Animation for LowPhantom Lord
		 *********************************************************/
		//  Add running animation textures to the array of texture regions to create running animation later, running animation at indexes 7-12, at row index 0 (relative to just "PhantomLordLow" coordinates)
		for (int i = 7; i < 13; i++)
		{
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordLow"), i * 35, 0, 35, 35));	// Go through each iteration, width and height for each animation is the same, 
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
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordLow"), i * 35, 0, 35, 35));	// Go through each iteration, width and height for each animation is the same, 
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
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordLow"), i*35, 0, 35, 35));
		}
		lowPLordStance = new Animation<TextureRegion>(0.4f, frames);					// Need higher time between each frame (seconds) runs too fast if lower
		
		frames.clear();
		
		/*********************************************************
		 *  Initialize Jumping Animation for LowPhantom Lord
		 *********************************************************/
		// Jump is at animation index 13 for PhantomLordLow region
		for (int i = 13; i < 14; i++)
		{
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordLow"), i*35, 0, 35, 35));
		}
		lowPLordJump = new Animation<TextureRegion>(0.1f, frames);						// 0.1f good for time between each frame for jumping animation
		
		frames.clear();
		
		/*********************************************************
		 *  Initialize Running Animation for AveragePhantom Lord
		 *********************************************************/
		//  Add running animation textures to the array of texture regions to create running animation later, running animation at indexes 7-12, at row index 0 (relative to just "PhantomLordAverage" coordinates)
		for (int i = 7; i < 13; i++)
		{
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordAverage"), i * 35, 0, 35, 35));	// Go through each iteration, width and height for each animation is the same, 
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
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordAverage"), i * 35, 0, 35, 35));	// Go through each iteration, width and height for each animation is the same, 
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
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordAverage"), i*35, 0, 35, 35));
		}
		averagePLordStance = new Animation<TextureRegion>(0.4f, frames);					// Need higher time between each frame (seconds) runs too fast if lower
		
		frames.clear();
		
		/*********************************************************
		 *  Initialize Jumping Animation for AveragePhantom Lord
		 *********************************************************/
		// Jump is at animation index 13 for PhantomLordAverage region
		for (int i = 13; i < 15; i++)
		{
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordAverage"), i*35, 0, 35, 35));
		}
		averagePLordJump = new Animation<TextureRegion>(.7f, frames);						// 0.1f good for time between each frame for jumping animation
		
		frames.clear();
		
		/*********************************************************
		 *  Initialize Running Animation for PowerUpPhantom Lord
		 *********************************************************/
		//  Add running animation textures to the array of texture regions to create running animation later, running animation at indexes 7-12, at row index 0 (relative to just "PhantomLordLow" coordinates)
		for (int i = 7; i < 13; i++)
		{
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordPower"), i * 35, 0, 35, 35));	// Go through each iteration, width and height for each animation is the same, 
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
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordPower"), i * 35, 0, 35, 35));	// Go through each iteration, width and height for each animation is the same, 
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
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordPower"), i*35, 0, 35, 35));
		}
		powerPLordStance = new Animation<TextureRegion>(0.4f, frames);					// Need higher time between each frame (seconds) runs too fast if lower
		
		frames.clear();
		
		/*********************************************************
		 *  Initialize Jumping Animation for PowerUPhantom Lord
		 *********************************************************/
		// Jump is at animation index 13 for PhantomLordLow region
		for (int i = 13; i < 14; i++)
		{
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordPower"), i*35, 0, 35, 35));
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
				frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordLow"), i*35, 0, 35, 35));
				frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordAverage"), i*35, 0, 35, 35));
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
				frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordAverage"), i*35, 0, 35, 35));
				frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordPower"), i*35, 0, 35, 35));
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
				frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordAverage"), i*35, 0, 35, 35));
				
				frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordAverage"), i*35, 0, 0, 0));
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
				frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordLow"), i*35, 0, 35, 35));
				
				frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordLow"), i*35, 0, 0, 0));
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
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordPower"), i*35, 0, 35, 35));
		}
		throwCane = new Animation<TextureRegion>(.15f, frames);					// Need higher time between each frame (seconds) runs too fast if lower
		
		frames.clear();
		
		/********************************************************
		 *  Initialize Move Animation for HatGhost
		 ********************************************************/
		// Bounce is at animation indexes 0-2 for "HatGhost" at row 1 (row index 0) this index# and row# is given by "GhostWithHat" texture region, which means we only focus on coordinates of
		// "GhostWithHat" region, thus the top left of just that spriteSheet will be (0,0), image that we are looking at the "GhostWithHat.png" from "AllSpriteSheets (For libGDX Texture Packer)" folder
		// And getting the coordinate from there, which we can use a for loop to quickly navigate though and get sprite frame, since each sprite is 35x35 
		for (int i = 0; i < 3; i++)
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("GhostWithHat"), i*35, 0 * 37, 35, 35));
		floatAnimation = new Animation<TextureRegion>(0.2f, frames);					// Might need higher or lower time (1st parameter, depending on animation)
		frames.clear();
		
		/********************************************************
		 *  Initialize HatHideStatic Animation for HatGhost
		 ********************************************************/
		// HatHide animations at indexes 5-6 for GhostWithHat texture region
		for (int i = 5; i < 7; i++)
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("GhostWithHat"), i*35, 0 * 37, 35, 35));
		hideInHatStaticAnimation = new Animation<TextureRegion>(0.4f, frames);					// Might need higher or lower time (1st parameter, depending on animation)
		frames.clear();
		
		/********************************************************
		 *  Initialize HatHideMoving Animation for HatGhost
		 ********************************************************/
		// HatHide animations at indexes e3-4 for GhostWithHat texture region
		for (int i = 3; i < 5; i++)
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("GhostWithHat"), i*35, 0 * 37, 35, 35));
		hideInHatMovingAnimation = new Animation<TextureRegion>(0.4f, frames);					// Might need higher or lower time (1st parameter, depending on animation)
		frames.clear();
		
		/*
		/********************************************************
		 *  Initialize Bounce Animation for Pumpkin
		 ********************************************************/
		// Bounce is at animation indexes 1-2 for "Pumpkin" at row 1 (row index 0) this index# and row# is given by "Pumpkin" texture region, which means we only focus on coordinates of
		// "Pumpkin" region, thus the top left of just that spriteSheet will be (0,0), image that we are looking at the "Pumpkin.png" from "AllSpriteSheets (For libGDX Texture Packer)" folder
		// And getting the coordinate from there, which we can use a for loop to quickly navigate though and get sprite frame, since each sprite is 35x35 
		for (int i = 1; i < 3; i++)
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("Pumpkin"), i*35, 0 * 37, 35, 35));
		bounceAnimation = new Animation<TextureRegion>(0.4f, frames);					// Might need higher or lower time (1st parameter, depending on animation)
		stateTimer = 0;
		frames.clear();
		
		/*
		/********************************************************
		 *  Initialize Hit Animation for Pumpkin
		 ********************************************************/
		// Bounce is at animation indexes 1-2 for "Pumpkin" at row 1 (row index 0) this index# and row# is given by "Pumpkin" texture region, which means we only focus on coordinates of
		// "Pumpkin" region, thus the top left of just that spriteSheet will be (0,0), image that we are looking at the "Pumpkin.png" from "AllSpriteSheets (For libGDX Texture Packer)" folder
		// And getting the coordinate from there, which we can use a for loop to quickly navigate though and get sprite frame, since each sprite is 35x35 
		for (int i = 0; i < 2; i++)
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("Pumpkin"), i*35, 0 * 37, 35, 35));
		pumpkinHitAnimation = new Animation<TextureRegion>(0.4f, frames);					// Might need higher or lower time (1st parameter, depending on animation)
		stateTimer = 0;
		frames.clear();
		
		/*
		/********************************************************
		 *  Initialize Fly Animation for Bat
		 ********************************************************/
		// Bounce is at animation indexes 1-2 for "Bat" at row 1 (row index 0) this index# and row# is given by "Bat" texture region, which means we only focus on coordinates of
		// "Bat" region, thus the top left of just that spriteSheet will be (0,0), image that we are looking at the "Bat.png" from "AllSpriteSheets (For libGDX Texture Packer)" folder
		// And getting the coordinate from there, which we can use a for loop to quickly navigate though and get sprite frame, since each sprite is 35x35 
		for (int i = 1; i < 3; i++)
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("Bat"), i*35, 0 * 37, 35, 35));
		flyAnimation = new Animation<TextureRegion>(0.4f, frames);					// Might need higher or lower time (1st parameter, depending on animation)
		stateTimer = 0;
		frames.clear();
		
		/*********************************************************
		 *  Initialize Death Animation for LPhantom Lord Death
		 *********************************************************/
		// Death frame animation at index 6 for "PhantomLordLow" texture region
		for (int i = 0; i < 8; i++)
		{
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("CanePower"), i * 35, 0, 35, 35));
		}
		caneSpinning = new Animation<TextureRegion>(0.03f, frames);		
		
		frames.clear();
		
		/*
		/********************************************************
		 *  Initialize Bounce Animation for Pumpkin
		 ********************************************************/
		// Bounce is at animation indexes 1-2 for "Pumpkin" at row 1 (row index 0) this index# and row# is given by "Pumpkin" texture region, which means we only focus on coordinates of
		// "Pumpkin" region, thus the top left of just that spriteSheet will be (0,0), image that we are looking at the "Pumpkin.png" from "AllSpriteSheets (For libGDX Texture Packer)" folder
		// And getting the coordinate from there, which we can use a for loop to quickly navigate though and get sprite frame, since each sprite is 35x35 
		for (int i = 0; i < 4; i++)
			frames.add(new TextureRegion(atlasPLordEnem.findRegion("Candle"), i*35, 0 * 37, 35, 35));
		floatCandleAnimation = new Animation<TextureRegion>(.15f, frames);					// Might need higher or lower time (1st parameter, depending on animation)
		frames.clear();
	
		// Don't need to set up music, it should still be playing from main menu
	}
	
	/*
	 * Method to create the camera
	 */
	private void createCamera() 
	{
		// Still in the 2d realm, use this camera
		camera = new OrthographicCamera();
		// Set camera to desired location (since our screen doesn't need to move,
		// just set to virtual width and height)
		camera.setToOrtho(false, PhantomLord.V_WIDTH, PhantomLord.V_HEIGHT);
		// Make sure to update the camera since the attributes were changed
		camera.update();
	}
	
	@Override public void show() 
	{ 
		Gdx.input.setInputProcessor(stage); 
	}
	
	@Override
	public void render(float delta)
	{
		stateTimer += delta;
		updateTimer += delta;
		
		if(updateTimer > 0 && updateTimer < .5f)
		{
			candyCorn.setPosition(50, 87);
			ghostMask.setPosition(50, 67);
			hatCandy.setPosition(50, 47);
		}
		else if(updateTimer > .5f && updateTimer < 1)
		{
			candyCorn.setPosition(50, 85);
			ghostMask.setPosition(50, 65);
			hatCandy.setPosition(50, 45);
		}
		else if(updateTimer > 1)
			updateTimer = 0;
		
		// Set Our animation texture and position
		ghostStance.setRegion(floatAnimation.getKeyFrame(stateTimer, true));
		pumpkinStance.setRegion(bounceAnimation.getKeyFrame(stateTimer, true));
		batStance.setRegion(flyAnimation.getKeyFrame(stateTimer, true));
		ghostHideInHat.setRegion(hideInHatStaticAnimation.getKeyFrame(stateTimer, true));
		caneSpin.setRegion(caneSpinning.getKeyFrame(stateTimer, true));
		pumpkinHit.setRegion(pumpkinHitAnimation.getKeyFrame(stateTimer, true));
		candle.setRegion(floatCandleAnimation.getKeyFrame(stateTimer, true));
		
		// Add clear color to clear out the screen on every render and have background color
		Gdx.gl.glClearColor(0, 0, 0, 1);			// Set background color
		Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);	// Clear the screen
		
		// draw the stage
		stage.draw();
		
		// Begin Drawing textures/sprites to screen
		game.batch.begin();
		
		// Position on x and y axis, (0,0) at bottom right hand corner
		game.batch.draw(background, 0, 0, PhantomLord.V_WIDTH, PhantomLord.V_HEIGHT);
		ghostStance.draw(game.batch);
		pumpkinStance.draw(game.batch);
		batStance.draw(game.batch);
		ghostHideInHat.draw(game.batch);
		caneSpin.draw(game.batch);
		pumpkinHit.draw(game.batch);
		candle.draw(game.batch);
		candyCorn.draw(game.batch);
		hatCandy.draw(game.batch);
		ghostMask.draw(game.batch);
		// End drawing 
		game.batch.end();
		
		spriteBatch.begin();
		//gameTitle.draw(spriteBatch);
		spriteBatch.end();
		stage.draw();
	}

	/*
	 * Create containors for the buttons
	 */
	private void createButtonsContainer()
	{
		// make table for the buttons
		buttonContainer = new Table();
		// Good size for the title
		buttonContainer.setPosition(PhantomLord.V_WIDTH - 350, PhantomLord.V_HEIGHT - 20);
		buttonContainer.add(exitInfoMenuButton).size(buttonWidth, buttonHeight);
		
		if(PhantomLord.currentInfoMenuScreen != 1)
		{
			// make table for the buttons
			buttonContainer2 = new Table();
			buttonContainer2.setPosition(PhantomLord.V_WIDTH - 350, PhantomLord.V_HEIGHT - 189);
			buttonContainer2.add(backButton).size(buttonWidth, buttonHeight);
		}
		
		if(PhantomLord.currentInfoMenuScreen != 2)
		{
			// make table for the buttons
			buttonContainer3 = new Table();
			buttonContainer3.setPosition(PhantomLord.V_WIDTH - 50, PhantomLord.V_HEIGHT - 189);
			buttonContainer3.add(nextButton).size(buttonWidth, buttonHeight);
		}
	}
	
	/*
	 * Method to create the menu buttons
	 */
	private void createButtons() 
	{
		// Set up the style
		TextButtonStyle style = new TextButtonStyle();
		style.font =  new BitmapFont();
		// Set texture for pressing and not pressing the button
		style.up = TextureLoader.getDrawable("buttonStandard");
		style.down = TextureLoader.getDrawable("buttonPressed");
		
		// Put the text in the buttom
		exitInfoMenuButton = new TextButton("Main Menu", style);
		nextButton = new TextButton("Next", style);
		backButton = new TextButton("Back", style);
	}
	
	private void createButtonsListeners() 
	{
		exitInfoMenuButton.addListener(new ChangeListener() 
		{
			@Override public void changed (ChangeEvent event, Actor actor) 
			{
				// Play "Select Menu Item" sound
				PhantomLord.manager.get("assets/audio/sounds/MenuSelect.mp3", Sound.class).play();
				
				dispose();
				
				// Set the screen to the new game screen
				game.setScreen(new PhantomLordMainMenu((PhantomLord) game));
	        }
	    });
		
		
		if(PhantomLord.currentInfoMenuScreen != 1)
		{
			backButton.addListener(new ChangeListener()
			{
				@Override public void changed (ChangeEvent event, Actor actor) 
				{
					// Play "Select Menu Item" sound
					PhantomLord.manager.get("assets/audio/sounds/MenuSelect.mp3", Sound.class).play();
					
					dispose();
					
					// Set the screen to the info menu screen
					game.setScreen(new InfoMenu((PhantomLord) game));
		        }
		    });
		}
		
		if(PhantomLord.currentInfoMenuScreen != 2)
		{
			nextButton.addListener(new ChangeListener()
			{
				@Override public void changed (ChangeEvent event, Actor actor) 
				{
					// Play "Select Menu Item" sound
					PhantomLord.manager.get("assets/audio/sounds/MenuSelect.mp3", Sound.class).play();
					
					dispose();
					
					// Set the screen to the info menu screen
					game.setScreen(new InfoMenu2((PhantomLord) game));
		        }
		    });
		}
	}
	
	@Override
	public void resize(int width, int height) {
		stage.getViewport().update(width, height, true);
	}

	@Override
	public void pause() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void resume() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void hide() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void dispose() 
	{
		// dispose the stage
		stage.dispose();
		background.dispose();
	}
}