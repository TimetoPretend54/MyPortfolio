package edu.uidaho.phantomlord.Tools;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.Pixmap.Format;
import com.badlogic.gdx.graphics.g2d.TextureAtlas;
import com.badlogic.gdx.graphics.g2d.TextureRegion;
import com.badlogic.gdx.scenes.scene2d.ui.Skin;
import com.badlogic.gdx.scenes.scene2d.utils.Drawable;
import com.badlogic.gdx.scenes.scene2d.utils.TextureRegionDrawable;

/*
 * Slightly modified from http://libdgxtutorials.blogspot.com/2013/09/libgdx-tutorial-8-using-texture-packer.html
 * &
 * https://stackoverflow.com/questions/16550544/libgdx-how-to-add-sprite-from-a-texture-atlas
 */

public class TextureLoader 
{
	/*
	 * For texture atlus and skin (skin is for UI widgets)
	 */
	private static Skin skin;
	private static TextureAtlas atlas;
			
	/*
	 * Get the skin if it exists
	 */
	public static Skin getSkin() 
	{
		if(skin == null)
			new TextureLoader();
		return skin;
	}
	
	/*
	 * Get the drawable texture, if it exists
	 */
	public static Drawable getDrawable(String name) 
	{
		if(skin == null) 
			new TextureLoader();
		return skin.getDrawable(name);
	}
	
	/*
	 * Create a new drawable, based on its width, height and color
	 */
	public static Drawable getDrawable(int width, int height, Color col) 
	{
		Pixmap pixmap = new Pixmap(width, height, Format.RGBA8888);
		pixmap.setColor(col);
		pixmap.fill();
		Drawable drw = new TextureRegionDrawable(new TextureRegion(new Texture(pixmap)));
		pixmap.dispose();
				
		return drw;
	}
	
	/*
	 * 
	 */
	private TextureLoader() 
	{
		try 
		{
			// Loads a texture atlus in
			atlas = new TextureAtlas(Gdx.files.internal("assets/textures.atlas"));		
		}
		catch(Exception ex) 
		{
			System.err.println("Failed to load textures - " + ex.getMessage());
			
			// make/remake our texture atlus
			ErrorCreatingAtlas();
		}
		//The finally block always executes when the try block exits. This ensures that the finally 
		// block is executed even if an unexpected exception occurs
		finally 
		{
			// Create a skin
			createSkin();
		}
	}
	
	private void ErrorCreatingAtlas() 
	{
		// Set up gray square in texture atlus (for one of checker boards)
		Pixmap pixmap = new Pixmap(100, 100, Format.RGBA8888);
		pixmap.setColor(Color.LIGHT_GRAY);
		pixmap.fill();
		TextureRegion region = new TextureRegion(new Texture(pixmap));
		pixmap.dispose();
		
		// Load in the regions
		atlas = new TextureAtlas();	
		atlas.addRegion("buttonStandard", region);
		atlas.addRegion("buttonPressed", region);
		atlas.addRegion("boardPurple", region);
		atlas.addRegion("boardDark", region);
		atlas.addRegion("pawnRed", region);
		atlas.addRegion("pawnBlue", region);
		atlas.addRegion("pawnRedKing", region);
		atlas.addRegion("pawnBlueKing", region);
		atlas.addRegion("checkboxOn", region);
		atlas.addRegion("boardWhite", region);
		atlas.addRegion("NewGameBackground", region);
	}
	
	/*
	 * Create a skin
	 */
	private void createSkin() 
	{
		// Make the new skin and add it to the atlus
		skin = new Skin();
		skin.addRegions(atlas);
		/*
		 * sel explanatory
		 */
		skin.getRegion("pawnRed").getTexture().setFilter(Texture.TextureFilter.Linear, Texture.TextureFilter.Linear);
		skin.getRegion("pawnBlue").getTexture().setFilter(Texture.TextureFilter.Linear, Texture.TextureFilter.Linear);
		skin.getRegion("pawnRedKing").getTexture().setFilter(Texture.TextureFilter.Linear, Texture.TextureFilter.Linear);
		skin.getRegion("pawnBlueKing").getTexture().setFilter(Texture.TextureFilter.Linear, Texture.TextureFilter.Linear);
	}
}