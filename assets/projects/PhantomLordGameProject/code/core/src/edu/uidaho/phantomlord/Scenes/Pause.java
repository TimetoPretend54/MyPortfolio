package edu.uidaho.phantomlord.Scenes;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.graphics.g2d.BitmapFont;
import com.badlogic.gdx.graphics.g2d.Sprite;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.g2d.TextureAtlas;
import com.badlogic.gdx.graphics.g2d.TextureRegion;
import com.badlogic.gdx.scenes.scene2d.Stage;
import com.badlogic.gdx.scenes.scene2d.ui.Label;
import com.badlogic.gdx.scenes.scene2d.ui.Table;
import com.badlogic.gdx.utils.Disposable;
import com.badlogic.gdx.utils.viewport.FitViewport;
import com.badlogic.gdx.utils.viewport.Viewport;

import edu.uidaho.phantomlord.PhantomLord;

public class Pause implements Disposable
{
	// When game world changes, want HUD to stay the same, new camera and new viewport for
	// HUD, so it stays locked there and only renders that part of the screen and world can
	// move independently on its own
	public Stage stage;
	private Viewport viewport;
	
	// Use Integer class type when you need an int to be treated like an 'object'
	private Integer worldTimerPLord;
	private boolean timeUpPLord;					// Is true when timer reaches 0
	private float timeCountPLord;
	private static Integer scorePLord;				// Can access in other classes without passing 
	
	// Label class type allows two types of info, the actual text to be displayed, and a 
	// 'labelStyle' (which involves info about a BitmapFont and Color used for font graphics)
	private Label countdownPLordLabel;
	private static Label scorePLordLabel;			// Can access in other classes without passing 
	private Label pauseLabel;
	
	// Load images from texture atlases using texture packer
	private TextureAtlas atlas;
	
	private Sprite livesImg;
	private Sprite candleImg;
	
	private SpriteBatch spritebatch;
	
	public Pause(SpriteBatch spritebatch)
	{
		this.spritebatch = spritebatch;
				
		// Set up HUD elements
		worldTimerPLord = 300;						// Timer for level will always be 300 seconds
		timeCountPLord = 0;
		
		atlas = new TextureAtlas(Gdx.files.internal("assets/actors/PhantomLord_and_Enemies.pack").file().getAbsolutePath());
		
		livesImg = new Sprite(new TextureRegion(atlas.findRegion("HatCandy")));
		// Resize Image (1st & 2nd parameters is position, 3rd and 4th are width and height - 400 x 204 pixels for screen)
		livesImg.setBounds(144, 180, 25, 25);
		
		candleImg = new Sprite(new TextureRegion(atlas.findRegion("Candle"), 1*35, 0 * 37, 35, 35));
		// Resize Image (1st & 2nd parameters is position, 3rd and 4th are width and height - 400 x 204 pixels for screen)
		candleImg.setBounds(150, 160, 20, 20);
		
		// Set viewport for HUD
		viewport = new FitViewport(PhantomLord.V_WIDTH, PhantomLord.V_HEIGHT, new OrthographicCamera());
		
		// Assign HUD to stage (empty box)
		stage = new Stage(viewport, spritebatch);
		
		//
		// Need to create table inside the stage, so then we can lay out the table in a way
		// to organize our labels in a certain position inside our stage
		//
		
		// Used to layout widgets, instead of manually sizing and positioning widgets
		// It is a group that sizes and positions children using table constraints
		Table table = new Table();
		
		// table.top will be at top of stage (normally table will align in center of stage, which
		// is center of virtual width and height, but we are specifying where to put it)
		table.center();
		
		// table is now size of our stage
		table.setFillParent(true);
		
		
		// string labels ( %03 means # of numbers in count down, which will be three)
		pauseLabel = new Label("PAUSE", new Label.LabelStyle(new BitmapFont(), Color.WHITE));
	
		//
		// Add our labels to the table
		//
		
		// 	*** Top Row ***
		// Multiple things in single row, then ExpandX will make it so they all share an equal
		// portion of the screen, and pad the top by 10 pixels
		table.add(pauseLabel).expandX().padTop(10);
		
		// Add table to stage
		stage.addActor(table);
	}
	
	public void update(float dt)
	{
		// Count down the timer each update
		timeCountPLord += dt;
	}
	
	@Override
	public void dispose() 
	{
		stage.dispose();
	}
}
