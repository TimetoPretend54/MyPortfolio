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

public class PLordHud implements Disposable
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
	private Label timePLordLabel;
	private Label levelPLordLabel;
	private Label worldPLordLabel;
	private Label phantomLordPLordLabel;
	private static Label lifeCount;
	private static Label candleCount;
	
	// Load images from texture atlases using texture packer
	private TextureAtlas atlas;
	
	private Sprite livesImg;
	private Sprite candleImg;
	
	private SpriteBatch spritebatch;
	
	public PLordHud(SpriteBatch spritebatch)
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
		table.top();
		
		// table is now size of our stage
		table.setFillParent(true);
		
		
		// string labels ( %03 means # of numbers in count down, which will be three)
		countdownPLordLabel = new Label(String.format("%03d", worldTimerPLord), new Label.LabelStyle(new BitmapFont(), Color.WHITE));
		scorePLordLabel = new Label(String.format("%06d", PhantomLord.score), new Label.LabelStyle(new BitmapFont(), Color.WHITE));
		timePLordLabel = new Label("TIME", new Label.LabelStyle(new BitmapFont(), Color.WHITE));
		levelPLordLabel = new Label(String.format(PhantomLord.WorldNumber + "-" + PhantomLord.LevelNumber), new Label.LabelStyle(new BitmapFont(), Color.WHITE));
		worldPLordLabel = new Label(String.format("WORLD"), new Label.LabelStyle(new BitmapFont(), Color.WHITE));
		phantomLordPLordLabel = new Label("Phantom Lord", new Label.LabelStyle(new BitmapFont(), Color.WHITE));
		lifeCount = new Label(String.format(" x %02d", PhantomLord.lives), new Label.LabelStyle(new BitmapFont(), Color.WHITE));
		candleCount = new Label(String.format(" x %03d", PhantomLord.candles), new Label.LabelStyle(new BitmapFont(), Color.WHITE));
	
		//
		// Add our labels to the table
		//
		
		// 	*** Top Row ***
		// Multiple things in single row, then ExpandX will make it so they all share an equal
		// portion of the screen, and pad the top by 10 pixels
		table.add(phantomLordPLordLabel).expandX().padTop(10);
		table.add(lifeCount).expandX().padTop(10);
		table.add(worldPLordLabel).expandX().padTop(10);
		table.add(timePLordLabel).expandX().padTop(10);
		table.row();
		
		//  *** Next Row ***
		table.add(scorePLordLabel).expandX();
		table.add(candleCount).expandX();
		table.add(levelPLordLabel).expandX();
		table.add(countdownPLordLabel).expandX();
		
		// Add table to stage
		stage.addActor(table);
	}
	
	public void update(float dt)
	{
		// Count down the timer each update
		timeCountPLord += dt;
		if(timeCountPLord >= 1)
		{
			if (worldTimerPLord > 0)
				worldTimerPLord--;
			else
				timeUpPLord = true;
			countdownPLordLabel.setText(String.format("%03d", worldTimerPLord));
			timeCountPLord = 0;
		}
	}
	
	public void draw(float delta)
	{ 
		spritebatch.begin();				// Begin our batch
		livesImg.draw(spritebatch);
		candleImg.draw(spritebatch);
		spritebatch.end();
		
	}
	
	// Add static so I can reference the hud.addScore without having to pass it
	public static void addScore(int value)
	{
		PhantomLord.score += value;
		scorePLordLabel.setText(String.format("%06d", PhantomLord.score));
	}
	
	public static void addLives()
	{
		lifeCount.setText(String.format(" x %02d", PhantomLord.lives));
	}
	
	public static void addCandles()
	{
		candleCount.setText(String.format(" x %03d", PhantomLord.candles));
	}
	
	public boolean isTimeUp()
	{
		return timeUpPLord;
	}
	
	@Override
	public void dispose() 
	{
		stage.dispose();
	}
}
