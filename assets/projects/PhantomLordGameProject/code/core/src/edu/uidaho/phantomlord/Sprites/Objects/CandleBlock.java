package edu.uidaho.phantomlord.Sprites.Objects;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.audio.Sound;
import com.badlogic.gdx.graphics.g2d.Animation;
import com.badlogic.gdx.graphics.g2d.TextureRegion;
import com.badlogic.gdx.maps.MapObject;
import com.badlogic.gdx.maps.tiled.TiledMap;
import com.badlogic.gdx.maps.tiled.TiledMapTileSet;
import com.badlogic.gdx.math.Rectangle;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.BodyDef;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.badlogic.gdx.physics.box2d.PolygonShape;
import com.badlogic.gdx.physics.box2d.World;

import edu.uidaho.phantomlord.PhantomLord;
import edu.uidaho.phantomlord.Scenes.PLordHud;
import edu.uidaho.phantomlord.Screens.PLordPlayScreen;
import edu.uidaho.phantomlord.Sprites.PhantomLordChar;
import edu.uidaho.phantomlord.Sprites.Items.HatCandy;
import edu.uidaho.phantomlord.Sprites.Items.CandyCorn;
import edu.uidaho.phantomlord.Sprites.Items.GhostMask;
import edu.uidaho.phantomlord.Sprites.Items.ItemDefinition;

public class CandleBlock extends TileObject
{
	private static TiledMapTileSet tileSet;
	private final int BLANK_CANDLE = 328+1;				// ID# was 328 in Tiled for Blank_Candle in "PhantomLordTileSet" 
														// (that we had to add cause of libGDX only reading first tileSet, 
														// 			so use only one tileSet for a .tmx file)
														// but liGDX starts tile ID's at 1, not 0 like TIled, 
														// so have to add + 1 to it
	
	/* 
	* Idea from
	* http://www.gamefromscratch.com/post/2014/09/25/LibGDX-LibGDX-Tutorial-13-Physics-with-Box2D-Part-4-Controlling-collisions-using-filters.aspx
	*/ 
	public CandleBlock(PLordPlayScreen screen, MapObject object, Boolean sensor)
	{
		// Super calls parent constructor, thus when we create a Candle class instance
		// it will pass it parameters to the InteractiveTileObject to create an 
		// instance of the object specifically for this class
		super(screen, object, sensor);
		
		
		// - Open tileSet (can only use "PhantomLordTileSet", libGDX only allows first/one tileSet to have inserted/rendered tiles, 
		// 		which means you should only use one tileSet per .tmx file, use tools to merge multiple tileSets before creating
		// 		a .tmx file, DONT use multiple tileSets when making a .tmx (Tiled file), 
		//
		// - We want to replace the Candle block with the 
		// 		image after it has been hit (the candle_blank tile in "PhantomLordTileSet"), which is in the 
		// 		tileSet "PhantomLordTileSet" (ONLY ONE TILESET CAN BE USED FOR A .tmx FILE!), don't need to 
		//		specify path, as Tiled package kept track of it
		tileSet = map.getTileSets().getTileSet("PhantomLordTileSet");
		
		//													  NOTE:
		//	In Tiled, find that image/tile of the Candle block after it has been hit, ONLY ONE TILESET FOR A Tiled.tmx file!
		//		and click on that tile for that block image, it should give an ID# to the left of the screen (the BLANK_CANDLE ID is 328 in Tiled)
		// NOTE: Tiled starts counting at the 0 index, and LibGDX starts counting at the 1 index for tiles, 
		//		so the ID# will now be 328+1 = 329
		
		/*********************************************************************************************************************** 
		* 													 WARNING: 
		* Limitation of libGDX, only FIRST tileSet in Tiled will render properly, so use ONLY ONE TILESET FOR A Tiled.tmx file!
		* that is the only tileSet I can use...must merge all tileSets together before making a .tmx file, or if needing 
		* to add a new tileSet merge the new tileSet with existing SINGLE tilseSet we have for the .tmx file (Tiled file)
		* 
		* - Find the ID# (Left of TiledD) to determine which tile to replace in "PhantomLordTileSet" (ONLY TILESET WE CAN HAVE 
		* 		FOR THIS TMX FILE), 
		* 
		* - use MediBand Paint and TecturePackerGUI for merging tileSets together
		************************************************************************************************************************/
		
		
		// Set user data to object itself
		fixture.setUserData(this);
		
		// Set category filter for Candle Fixture
		setCategoryFilter(PhantomLord.OBJECT_BIT);
	}
	/*
	 * Idea for Plord Colliison from
	 * http://www.hobbygamedev.com/adv/2d-platformer-advanced-collision-detection/
	 */
	@Override
	public void onPLordHeadHit(PhantomLordChar pLord) 
	{
		/*
		 * Some of code/idea from 
		 * https://stackoverflow.com/questions/27790191/how-to-get-the-position-of-a-tiledmap-cell-in-libgdx
		 * &
		 * https://gamedev.stackexchange.com/questions/69560/libgdx-get-the-value-of-a-key
		 */
		
		// Need to check when to play audio for if the candleBlock was hit last time, or other audio if we have already hit it and play the "dud" sound
		// This will also correlate to determining if we get points, as if we have already hit the Candle, we should NOT get any additional points
		if(getCell().getTile().getId() == BLANK_CANDLE)
		{
			// Add sound file for when hitting the 'blank Candle Block' from AssetManager and then play it (in same line, since we don't need to loop it)
			PhantomLord.manager.get("assets/audio/sounds/StoneBump.mp3", Sound.class).play();
		}
		else
		{
			// Check if tile has a "custom property" for a candycorn item (this was set in Tiled by selecting a "candleBlock" object in the candleBlock 
			// object layer, and then at the bottom left of the properties window, clicking "+" to add the custom property "candycorn" to that specific
			// candleblock, thus some candleBlock will have custom properties that specifify an item, instead of having to create a whole new object 
			// layer just for items
			// Check to see if ther is "candycorn" custom property/key
			if(object.getProperties().containsKey("candycorn"))
			{
				// Check is PLord is in low health mode or high/average mode, if in average/high mode, instead of spawing candycorn, spawn GhostMask
				if(pLord.isPLordAverage() || pLord.isPLordPower())
				{
					// Spawn Ghost Mask, in avg health state or powerup state
					// Since candy corn should exist a this tile, since it had a set custom property for it, spawn the candycorn or GhostMask
					
					
					// Use PlayScreen "screen" object to spawn CandyCorn right above (the +16 part) the Candle Block that gets hits by PLord
					screen.spawnItem(new ItemDefinition(new Vector2(box2DBody.getPosition().x, 
							box2DBody.getPosition().y + 13 / PhantomLord.PIXELS_PER_METER),
							GhostMask.class));
				}
				else
				{
					// Spawn Candy COrn, in low health state
					// Since candy corn should exist a this tile, since it had a set custom property for it, spawn the candycorn or GhostMask
					
					
					// Use PlayScreen "screen" object to spawn CandyCorn right above (the +16 part) the Candle Block that gets hits by PLord
					screen.spawnItem(new ItemDefinition(new Vector2(box2DBody.getPosition().x, 
							box2DBody.getPosition().y + 16 / PhantomLord.PIXELS_PER_METER),
							CandyCorn.class));
				}
					
				// Play Power Up Spawn sound
				PhantomLord.manager.get("assets/audio/sounds/PowerUpSpawn.mp3", Sound.class).play();
			}
			else if(object.getProperties().containsKey("hatcandy"))
			{
				// Use PlayScreen "screen" object to spawn CandyCorn right above (the +16 part) the Candle Block that gets hits by PLord
				screen.spawnItem(new ItemDefinition(new Vector2(box2DBody.getPosition().x, 
						box2DBody.getPosition().y + 16 / PhantomLord.PIXELS_PER_METER),
						HatCandy.class));
				
				// Play Power Up Spawn sound
				PhantomLord.manager.get("assets/audio/sounds/PowerUpSpawn.mp3", Sound.class).play();
			}
			// Otherwise defaultly Add sound file for when hitting the Candle Block from AssetManager and then play it (in same line, since we don't need to loop it)
			else
			{
				// Add score to HUD, as haven't hit the 'Candle block' before this
				PLordHud.addScore(100);
				
				PhantomLord.manager.get("assets/audio/sounds/CandleCollect.mp3", Sound.class).play();
				
				// candle count
				PhantomLord.candles++;
				PLordHud.addCandles();
				
				if(PhantomLord.candles == 50 || PhantomLord.candles == 100)
				{
					PhantomLord.lives++;
					PLordHud.addLives();
					PhantomLord.manager.get("assets/audio/sounds/PowerUp.mp3", Sound.class).play();
				}
			}
		}
		
		// When head hit the Candle Block object, want to replace the tile with the Blank_Candle tile
		// We are using variable "tileSet" the contains the tileSet of "PhantomLordTileSet" 
		// (assigned above in Candle constructor) that holds the Blank_Candle tile
		// "BLANK_CANDLE" is the ID# for the tile, we do this afterward, because otherwise we change the state before
		// knowing if the candle block was hit or not beforehand, thus it would always be considered "Blank"
		getCell().setTile(tileSet.getTile(BLANK_CANDLE));
		
	}

}
