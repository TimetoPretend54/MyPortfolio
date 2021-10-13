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
public class InfoMenu implements Screen
{
	// Background for menu
	private Texture background;
	
	// Reference to game
	private PhantomLord game;
	
	private Viewport viewport;
	private Stage stage;
	
	// Keep track of time we are in any given state
	private float stateTimer;	
	
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
	
	private Sprite gameTitle;
	private Sprite pLordAvgStance;
	private Sprite pLordLowStance;
	private Sprite pLordHighStance;
	private Sprite pLordRunning;
	private Sprite pLordRunningFlip;
	private Sprite pLordJump;
	private Sprite pLordFastRun;
	private Sprite pLordThrow;
	private Sprite pLord7;
	private Sprite pLord8;
	private Sprite pLord9;
	private Sprite pLord10;
	private Sprite pLord11;
	private Table buttonContainer, buttonContainer2, buttonContainer3;
	private TextButton exitInfoMenuButton, backButton, nextButton;
	
	// Music variable for setting up music to play in game
	private Music music;
	
	// Constructor
	public InfoMenu(PhantomLord game)
	{
		PhantomLord.currentInfoMenuScreen = 1;
		
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
		background = new Texture(Gdx.files.internal("assets/menu/PhantomLord_How_To_Play_Screen1.png").file().getAbsolutePath());
	
		// Set up Music (Remember using static, IT MIGHT GIVE ERRORS, when testing if it does, fix it, otherwise don't worry)
		// Load in 'Intense(Compressed).mp3', the music for level 1, loop it and play it
		music = PhantomLord.manager.get("assets/audio/music/Spirit Waltz(Menu) (Compressed).mp3", Music.class);
		music.setLooping(true); 				// Loop the song
		music.play(); 	
		
		/* Set up the sprites for Animations of PLord on Screen */
		pLordAvgStance = new Sprite(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordAverage"), 14*35, 0 * 37, 35, 35));
		pLordAvgStance.setBounds(50, 100, 23, 28);
		pLordLowStance = new Sprite(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordLow"), 14*35, 0 * 37, 35, 35));
		pLordLowStance.setBounds(50, 130, 23, 28);
		pLordHighStance = new Sprite(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordPower"), 14*35, 0 * 37, 35, 35));
		pLordHighStance.setBounds(50, 70, 23, 28);
		pLordRunningFlip = new Sprite(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordAverage"), 7*35, 0 * 37, 35, 35));
		pLordRunningFlip.flip(true, false);
		pLordRunningFlip.setBounds(205, 130, 23, 28);
		pLordRunning = new Sprite(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordAverage"), 7*35, 0 * 37, 35, 35));
		pLordRunning.setBounds(205, 100, 23, 28);
		pLordJump = new Sprite(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordAverage"), 13*35, 0 * 37, 35, 35));
		pLordJump.setBounds(205, 70, 23, 28);
		pLordFastRun = new Sprite(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordAverage"), 7*35, 0 * 37, 35, 35));
		pLordFastRun.setBounds(205, 40, 23, 28);
		pLordThrow = new Sprite(new TextureRegion(atlasPLordEnem.findRegion("PhantomLordPower"), 0*35, 0 * 37, 35, 35));
		pLordThrow.setBounds(50, 40, 23, 28);
		
		stateTimer = 0;
		
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
		
		// Set Our animation texture and position
		pLordAvgStance.setRegion(averagePLordStance.getKeyFrame(stateTimer, true));
		pLordLowStance.setRegion(lowPLordStance.getKeyFrame(stateTimer, true));
		pLordHighStance.setRegion(powerPLordStance.getKeyFrame(stateTimer, true));
		pLordRunning.setRegion(averagePLordRun.getKeyFrame(stateTimer, true));
		pLordRunningFlip.setRegion(averagePLordRun.getKeyFrame(stateTimer, true));
		pLordFastRun.setRegion(averagePLordFastRun.getKeyFrame(stateTimer, true));
		pLordRunningFlip.flip(true, false);
		pLordJump.setRegion(averagePLordJump.getKeyFrame(stateTimer, true));
		pLordThrow.setRegion(throwCane.getKeyFrame(stateTimer, true));
		
		// Add clear color to clear out the screen on every render and have background color
		Gdx.gl.glClearColor(0, 0, 0, 1);			// Set background color
		Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);	// Clear the screen
		
		// draw the stage
		stage.draw();
		
		// Begin Drawing textures/sprites to screen
		game.batch.begin();
		
		// Position on x and y axis, (0,0) at bottom right hand corner
		game.batch.draw(background, 0, 0, PhantomLord.V_WIDTH, PhantomLord.V_HEIGHT);
		pLordAvgStance.draw(game.batch);
		pLordLowStance.draw(game.batch);
		pLordHighStance.draw(game.batch);
		pLordRunning.draw(game.batch);
		pLordRunningFlip.draw(game.batch);
		pLordJump.draw(game.batch);
		pLordFastRun.draw(game.batch);
		pLordThrow.draw(game.batch);
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
