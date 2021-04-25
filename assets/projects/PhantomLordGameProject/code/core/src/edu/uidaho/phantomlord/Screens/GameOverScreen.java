package edu.uidaho.phantomlord.Screens;

import com.badlogic.gdx.Game;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Input;
import com.badlogic.gdx.Screen;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.graphics.g2d.BitmapFont;
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
public class GameOverScreen implements Screen
{
	private Viewport viewport;
	private Stage stage;
	
	/// Bring in 'main' game ibject
	private Game game;
	
	public float gameTimer = 0f;
	
	// COnstructor
	public GameOverScreen(Game game)
	{
		this.game = game;
		
		// Set viewport for GameOver Screen
		viewport = new FitViewport(PhantomLord.V_WIDTH, PhantomLord.V_HEIGHT, new OrthographicCamera());
	
		// Set stage (avoid having to create new batch by using old one)
		stage = new Stage(viewport, ((PhantomLord) game).batch);
		
		// Create the font for GameoOver Text
		Label.LabelStyle font = new Label.LabelStyle(new BitmapFont(), Color.WHITE);
		
		// Create actors for stage
		Table table = new Table();
		// Align the table in center of the screen
		table.center();
		// THe table with take up the entire stage
		table.setFillParent(true);
		
		Label gameOverLabel = new Label("Game OVER", font);
		// Label for play again
		//Label playAgainLabel = new Label("Press [ENTER] to Play 'Phantom Lord' Again", font);
		// label for Exiting back to main menu
		//Label exitToMainMenuLabel = new Label("Press Escape to exit to Main Menu", font);
		
		// Expand whole length of the row
		table.add(gameOverLabel).expandX();
		// below game over label create new row for playAgain label
		//table.row();
		// Expand whole length of the row, pad it so 10 pixels below Game Over label
		//table.add(playAgainLabel).expandX().padTop(40f);
		// add row
		//table.row();
		// add row
		//table.add(exitToMainMenuLabel).expandX().padTop(10f);
		
		// Add the table to the stage
		stage .addActor(table);
	}
	
	@Override
	public void show() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void render(float delta)
	{
		gameTimer += delta;
		
		// Add clear color to clear out the screen on every render and have background color
		Gdx.gl.glClearColor(0, 0, 0, 1);			// Set background color
		Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);	// Clear the screen
		
		PhantomLord.pLordState = 1;					// Back to low health
		
		// Draw the stage
		stage.draw();
		
		if(gameTimer > 2)
		{
			// Set Screen to the Main Menu Screen
			game.setScreen(new PhantomLordMainMenu((PhantomLord) game));
			
			// Anytime we change screen, dispose of screen we are moving from
			dispose();
		}
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
