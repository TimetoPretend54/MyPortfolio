package edu.uidaho.phantomlord.Tools;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g2d.BitmapFont;
import com.badlogic.gdx.graphics.g2d.freetype.FreeTypeFontGenerator;
import com.badlogic.gdx.graphics.g2d.freetype.FreeTypeFontGenerator.FreeTypeFontParameter;

/*
 * Slightly modified from
 * https://stackoverflow.com/questions/12466385/how-can-i-draw-text-using-libgdx-java
 */

public class Font 
{
	/*
	 * The font I want to use
	 */
	private static final String FONT_NAME = "verdana.ttf";
	
	/*
	 * They do stuff...not very good explanations online, but...they work
	 */
	private static FreeTypeFontGenerator generator;
	private static FreeTypeFontParameter parameter;
		
	// Get the font type
	public static BitmapFont get(int size) 
	{
		generator = new FreeTypeFontGenerator(Gdx.files.internal("assets/"+FONT_NAME));
		parameter = new FreeTypeFontParameter();
		parameter.size = size;
		
		// Generate the font
		BitmapFont font = generator.generateFont(parameter);
		
		// Remove the generator
		generator.dispose();
		
		// Paremeter is of no use
		parameter = null;
		
		// Return the created font
		return font;
	}
		
	/*
	 * Construtor
	 */
	private Font() { }
}