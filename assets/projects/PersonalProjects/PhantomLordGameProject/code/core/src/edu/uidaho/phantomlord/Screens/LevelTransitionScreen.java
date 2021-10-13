package edu.uidaho.phantomlord.Screens;

import com.badlogic.gdx.Game;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.Screen;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.BitmapFont;
import com.badlogic.gdx.graphics.g2d.Sprite;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.g2d.TextureAtlas;
import com.badlogic.gdx.graphics.g2d.TextureRegion;
import com.badlogic.gdx.scenes.scene2d.Stage;
import com.badlogic.gdx.scenes.scene2d.ui.Label;
import com.badlogic.gdx.scenes.scene2d.ui.Table;
import com.badlogic.gdx.utils.viewport.FitViewport;
import com.badlogic.gdx.utils.viewport.Viewport;

import edu.uidaho.phantomlord.PhantomLord;

/*
 * Code Inspired from
 * https://github.com/libgdx/libgdx/wiki/Extending-the-simple-game
 * "The Main Main" sectio
 */
public class LevelTransitionScreen implements Screen
{
	private Viewport viewport;
	private Stage stage;
	
	// Use Integer class type when you need an int to be treated like an 'object'
	private Integer worldTimerPLord;
	private boolean timeUpPLord;					// Is true when timer reaches 0
	private float timeCountPLord;
	private static Integer scorePLord;				// Can access in other classes without passing 
	
	// Label class type allows two types of info, the actual text to be displayed, and a 
	// 'labelStyle' (which involves info about a BitmapFont and Color used for font graphics)
	private Label countdownPLordLabel;
	private static Label scorePLordLabel;			// Can access in other classes without passing 
	private Label timePLordLabel;
	private Label levelPLordLabel;
	private Label worldPLordLabel;
	private Label phantomLordPLordLabel;
	private Label lifeCount;
	
	// Load images from texture atlases using texture packer
	private TextureAtlas atlas;
	
	private Sprite livesImg;
	
	public float gameTimer = 0f;
	
	/// Bring in 'main' game ibject
	private Game game;
		
	// Constructor
	public LevelTransitionScreen(Game game)
	{
		this.game = game;
		
		atlas = new TextureAtlas(Gdx.files.internal("assets/actors/PhantomLord_and_Enemies.pack").file().getAbsolutePath());
		
		livesImg = new Sprite(new TextureRegion(atlas.findRegion("HatCandy")));
		
		// 400 x 204 pixels for screen
		livesImg.setPosition(157, 90);
		
		// Set viewport for GameOver Screen
		viewport = new FitViewport(PhantomLord.V_WIDTH, PhantomLord.V_HEIGHT, new OrthographicCamera());
	
		// Set stage (avoid having to create new batch by using old one)
		stage = new Stage(viewport, ((PhantomLord) game).batch);
		
		// Create the font for GameoOver Text
		Label.LabelStyle font = new Label.LabelStyle(new BitmapFont(), Color.WHITE);
		
		// Create actors for stage
		Table table = new Table();
		// Align the table in center of the screen
		table.top();
		// THe table with take up the entire stage
		table.setFillParent(true);
		
		// string labels ( %03 means # of numbers in count down, which will be three)
		countdownPLordLabel = new Label("300", new Label.LabelStyle(new BitmapFont(), Color.WHITE));
		scorePLordLabel = new Label(String.format("%06d", PhantomLord.score), new Label.LabelStyle(new BitmapFont(), Color.WHITE));
		timePLordLabel = new Label("TIME", new Label.LabelStyle(new BitmapFont(), Color.WHITE));
		levelPLordLabel = new Label(String.format(PhantomLord.WorldNumber + "-" + PhantomLord.LevelNumber), new Label.LabelStyle(new BitmapFont(), Color.WHITE));
		worldPLordLabel = new Label(String.format("WORLD"), new Label.LabelStyle(new BitmapFont(), Color.WHITE));
		phantomLordPLordLabel = new Label("Phantom Lord", new Label.LabelStyle(new BitmapFont(), Color.WHITE));
		
		lifeCount = new Label(String.format(" x %02d", PhantomLord.lives), font);
		
		//
		// Add our labels to the table
		//
		
		// 	*** Top Row ***
		// Multiple things in single row, then ExpandX will make it so they all share an equal
		// portion of the screen, and pad the top by 10 pixels
		table.add(phantomLordPLordLabel).expandX().padTop(10);
		table.add(worldPLordLabel).expandX().padTop(10);
		table.add(timePLordLabel).expandX().padTop(10);
		table.row();
		
		//  *** Next Row ***
		table.add(scorePLordLabel).expandX();
		table.add(levelPLordLabel).expandX();
		table.add(countdownPLordLabel).expandX();
		
		// Create actors for stage
		Table table1 = new Table();
		// Align the table in center of the screen
		table1.center();
		// THe table with take up the entire stage
		table1.setFillParent(true);
		
		// Label for lives (will have live counter for final version)
		//Label playAgainLabel = new Label(" x Infinite (Press [ENTER] to begin)", font);
		
		// Expand whole length of the row
		//table.add(gameOverLabel).expandX();
		
		// below game over label create new row for playAgain label
		table1.row();
		// Expand whole length of the row, pad it so 10 pixels below Game Over label
		table1.add(lifeCount).expandX().padTop(10f);
		
		// Add the table to the stage
		stage.addActor(table);
		
		// Add table to stage
		stage.addActor(table1);
	}
	
	@Override
	public void show() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void render(float delta)
	{ 
		gameTimer += delta;
		
		// Go to level after 2 seconds
		if(gameTimer > 2)
		{
			// Set screen to the level of PhantomLord, right now called "PLordPlayScreen"
			game.setScreen(new PLordPlayScreen((PhantomLord) game));
			
			// Anytime we change screen, dispose of screen we are moving from
			dispose();
		}
		
		// Add clear color to clear out the screen on every render and have background color
		Gdx.gl.glClearColor(0, 0, 0, 1);			// Set background color
		Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);	// Clear the screen
		
		((PhantomLord) game).batch.begin();				// Begin our batch
		livesImg.draw(((PhantomLord) game).batch);
		((PhantomLord) game).batch.end();
		// Draw the stage
		stage.draw();
	}

	@Override
	public void resize(int width, int height) {
		// TODO Auto-generated method stub
		
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
	}

}
