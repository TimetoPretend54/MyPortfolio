package edu.uidaho.phantomlord.Screens;

import com.badlogic.gdx.Game;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.Screen;
import com.badlogic.gdx.audio.Music;
import com.badlogic.gdx.audio.Sound;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.BitmapFont;
import com.badlogic.gdx.graphics.g2d.Sprite;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.g2d.TextureAtlas;
import com.badlogic.gdx.scenes.scene2d.Actor;
import com.badlogic.gdx.scenes.scene2d.Stage;
import com.badlogic.gdx.scenes.scene2d.ui.Label;
import com.badlogic.gdx.scenes.scene2d.ui.Table;
import com.badlogic.gdx.scenes.scene2d.ui.TextButton;
import com.badlogic.gdx.scenes.scene2d.ui.TextButton.TextButtonStyle;
import com.badlogic.gdx.scenes.scene2d.utils.ChangeListener;
import com.badlogic.gdx.scenes.scene2d.utils.ChangeListener.ChangeEvent;
import com.badlogic.gdx.utils.viewport.FitViewport;
import com.badlogic.gdx.utils.viewport.Viewport;

import edu.uidaho.phantomlord.PhantomLord;
import edu.uidaho.phantomlord.Tools.Font;
import edu.uidaho.phantomlord.Tools.TextureLoader;

/*
 * Code Inspired from
 * https://github.com/libgdx/libgdx/wiki/Extending-the-simple-game
 * "The Main Main" section
 */
public class PhantomLordMainMenu implements Screen
{
	// Background for menu
	private Texture background;
	
	// Reference to game
	private PhantomLord game;
	
	private Viewport viewport;
	private Stage stage;
	
	// Camera
	protected OrthographicCamera camera;
	
	private final int titleWidth = 500;
	private final int titleHeight = 150;
	private final int buttonFontSize = 25;
	private final float buttonWidth = 100;
	private final float buttonHeight = 35;
	
	// Load images from texture atlases using texture packer
	private TextureAtlas atlas;
	
	// sprites
	protected SpriteBatch spriteBatch;
	
	private Sprite gameTitle;
	private Table buttonContainer;
	private TextButton startGameButton, exitGameButton, infoButton;
	
	// Music variable for setting up music to play in game
	private Music music;
	
	// Constructor
	public PhantomLordMainMenu(PhantomLord game)
	{
		// assign PhantomLord game instance to the class
		this.game = game;
		
		atlas = new TextureAtlas(Gdx.files.internal("assets/textures.atlas").file().getAbsolutePath());
		
		// Method to make camera (how speeecial)
		createCamera();
		
		viewport = new FitViewport(PhantomLord.V_WIDTH, PhantomLord.V_HEIGHT, new OrthographicCamera());
		
		// Set stage (avoid having to create new batch by using old one)
		stage = new Stage(viewport, ((PhantomLord) game).batch);
		
		// Assign backgorund texture for main menu
		background = new Texture(Gdx.files.internal("assets/menu/PhantomLordMainMenu.png").file().getAbsolutePath());
	
		// Set up Music (Remember using static, IT MIGHT GIVE ERRORS, when testing if it does, fix it, otherwise don't worry)
		// Load in 'Intense(Compressed).mp3', the music for level 1, loop it and play it
		music = PhantomLord.manager.get("assets/audio/music/Spirit Waltz(Menu) (Compressed).mp3", Music.class);
		music.setLooping(true); 				// Loop the song
		music.play(); 	
		
		// Method to help make main menu
		//createGameTitle();
		createButtons();
		createButtonsListeners();
		createButtonsContainer();
		
		// Set up the spritebatch
		spriteBatch = new SpriteBatch();

		stage.addActor(buttonContainer);
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
		// Add clear color to clear out the screen on every render and have background color
		Gdx.gl.glClearColor(0, 0, 0, 1);			// Set background color
		Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);	// Clear the screen
		
		// draw the stage
		stage.draw();
		
		// Begin Drawing textures/sprites to screen
		game.batch.begin();
		
		// Position on x and y axis, (0,0) at bottom right hand corner
		game.batch.draw(background, 0, 0, PhantomLord.V_WIDTH, PhantomLord.V_HEIGHT);
		
		// End drawing 
		game.batch.end();
		
		spriteBatch.begin();
		//gameTitle.draw(spriteBatch);
		spriteBatch.end();
		stage.draw();
	}
	
	// Load in the game title
	private void createGameTitle() 
	{
		//gameTitle = new Sprite(new Texture(Gdx.files.internal("assets/PhantomLord.png")));
		//gameTitle.setPosition(PhantomLord.V_WIDTH/2 - titleWidth/2.3f, PhantomLord.V_HEIGHT/2 - titleHeight/2 + 200);
	}
	
	/*
	 * Create containors for the buttons
	 */
	private void createButtonsContainer()
	{
		// make table for the buttons
		buttonContainer = new Table();
		// Good size for the title
		buttonContainer.setPosition(PhantomLord.V_WIDTH/2, PhantomLord.V_HEIGHT/2 - 30);
		
		// Set size of button and pad them and make sure 2nd button is below the other by ".row()"
		buttonContainer.add(startGameButton).size(buttonWidth, buttonHeight);
		buttonContainer.row().padTop(0);
		buttonContainer.add(infoButton).size(buttonWidth, buttonHeight);
		buttonContainer.row().padTop(0);
		buttonContainer.add(exitGameButton).size(buttonWidth, buttonHeight);
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
		startGameButton = new TextButton("Start", style);
		infoButton = new TextButton("Info", style);
		exitGameButton = new TextButton("Exit", style);
	}
	
	private void createButtonsListeners() 
	{
		startGameButton.addListener(new ChangeListener() 
		{
			@Override public void changed (ChangeEvent event, Actor actor) 
			{
				// Play "Select Menu Item" sound
				PhantomLord.manager.get("assets/audio/sounds/MenuSelect.mp3", Sound.class).play();
				
				PhantomLord.manager.get("assets/audio/music/Spirit Waltz(Menu) (Compressed).mp3", Music.class).stop();
				
				dispose();
				
				// Set the screen to the new game screen
				game.setScreen(new LevelTransitionScreen((PhantomLord) game));
	        }
	    });
		
		exitGameButton.addListener(new ChangeListener()
		{
			@Override public void changed (ChangeEvent event, Actor actor) 
			{
				// Play "Select Menu Item" sound
				PhantomLord.manager.get("assets/audio/sounds/MenuSelect.mp3", Sound.class).play();
				
				// Exit the game
	            Gdx.app.exit();
	        }
	    });
		
		infoButton.addListener(new ChangeListener()
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

