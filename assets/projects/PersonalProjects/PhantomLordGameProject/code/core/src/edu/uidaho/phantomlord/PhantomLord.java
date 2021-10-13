/* PhantomLord.java
 * Description: Java libGDX scidescroller game
 * Programmer: Adrian Beehner
 * Date: 9/29/17

Problem: Side-Scroller

Write a simple game in libGDX that is a "side scroller" with the following characteristics:
	* Major point of assignment: work with graphics assets
	* You may determine your own premise
	* Suggested themes. Your own, or one of:
			* dodge insincere politicians and their minions
			* weave and bob your way through hurricanes
			* rescue victims from a collapsing building
	* The scroller must scroll. It must involve at least one "level" that is considerably larger than a single screen shows at any one time.
	* It can terminate via death, reaching an exit point, or traversing from one end to another.
	* It must not be code from some non-original internet solution; write your own game.*/

package edu.uidaho.phantomlord;

import com.badlogic.gdx.ApplicationAdapter;
import com.badlogic.gdx.Game;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.assets.AssetManager;
import com.badlogic.gdx.audio.Music;
import com.badlogic.gdx.audio.Sound;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;

import edu.uidaho.phantomlord.Screens.PLordPlayScreen;
import edu.uidaho.phantomlord.Screens.PhantomLordMainMenu;


public class PhantomLord extends Game
{
	// Final means we cannot change the variable, and statics means we need no instance to use it
	// Our Virtual Widths and Heights
	public static final int V_WIDTH = 400;
	public static final int V_HEIGHT = 208;
	public static final float PIXELS_PER_METER = 100;		// For scaling purposes for box2d
	
	// Box2D Filters: fixture in box2D has a filter, which has two parts
	//		1) Category: What is this fixture, is it PhantomLord, Candle, CrumbleStone, etc..
	//		2) Mask: What can that fixture collide with? Can PhantomLord collide with Candle,
	//				CrumbleStone, ground, etc....
	// Create default values for 'filters'
	//
	// Idea from http://www.aurelienribon.com/post/2011-07-box2d-tutorial-collision-filtering
	//
	public static final short NOTHING_BIT = 0;					
	public static final short PLORD_BIT = 2;				// Power of 2 for all filters because SHORT Type and easy to OR powers of two
	public static final short ENEMY_BIT = 4;
	public static final short OBJECT_BIT = 8;				// Every fixture that doesn't have "unique functionality", which right now is ground, crumblestone, candleblock, vegetation, topOfArchitecture, Stone
	public static final short DESTROYED_BIT = 16;			// Bit to make item in environment disappear
	public static final short ENEMYHEAD_BIT = 32;
	public static final short ITEM_BIT = 64;
	public static final short PLORDHEAD_BIT = 128;
	public static final short EVENT_BIT = 256;
	public static final short WEAPON_BIT = 512;				// For cane throw
	
	public static int WorldNumber = 1;
	public static int LevelNumber = 1;
	public static int WorldTimer = 300;
	public static int lives = 3;							// default life count
	public static int candles = 0;
	public static int score = 0;
	public static int pLordState = 1;						// 1 = low health, 2 = average, 3 = powerup
	public static int currentInfoMenuScreen = 0;
	
	// all different screens will have access to this spriteBatch
	public SpriteBatch batch;
	
	// Variabel to hold beginning numbers for world and leve;
	
	/*
	 * WARNING: Using AssetManager in a static way can cause issues,
	 * Instead I may might/need to pass around AssetManager to the classes that need it.
	 * I am going to use static context, at least for the time being, unless it provides errors
	 */
	// Declare Assetmanager 'manager' variable
	public static AssetManager manager;
	
	@Override
	public void create () 
	{
		// Create new spriteBatch
		batch = new SpriteBatch();
		
		/*
		* Code slightmodified from
		* https://github.com/libgdx/libgdx/wiki/Managing-your-assets
		*/
		
		// Create new AssetManager
		manager = new AssetManager();
		
		//
		// Queue up assets to be loaded by the manager (1st the Music files (noted by the 2nd parameter that states they are a Music.class)
		// DONT NEED TO DO ABSOLUTE PATH, as if we generally aren't using Android and need to specify a filename, we have to give full
		// path: "Gdx.files.internal("assets/stuff...").file().getAbsolutePath()" for AssestManager.load(),
		// Just specify the "assets/stuff" for the filename requirement (weird that libGDX can't keep these things straight...)
		//
		// Load up Music Files
		manager.load("assets/audio/music/Intense(Compressed).mp3", Music.class);
		//manager.load("assets/audio/music/Lonely Witch(Menu) (Compressed).mp3", Music.class);
		//manager.load("assets/audio/music/Menu-Upbeat.mp3", Music.class);
		manager.load("assets/audio/music/Spirit Waltz(Menu) (Compressed).mp3", Music.class);
		manager.load("assets/audio/music/Menu-Upbeat.mp3", Music.class);
		manager.load("assets/audio/music/Love Upon Differences.mp3", Music.class);
		manager.load("assets/audio/music/Winter waltz.mp3", Music.class);
		// Load up Sound FIles
		manager.load("assets/audio/sounds/CandleCollect.mp3", Sound.class);
		manager.load("assets/audio/sounds/CaneThrow.mp3", Sound.class);
		manager.load("assets/audio/sounds/CompleteLevel.mp3", Sound.class);
		manager.load("assets/audio/sounds/GameOver.mp3", Sound.class);
		manager.load("assets/audio/sounds/Jump.mp3", Sound.class);
		manager.load("assets/audio/sounds/MenuSelect.mp3", Sound.class);
		manager.load("assets/audio/sounds/PLordHitPowerDown.mp3", Sound.class);
		manager.load("assets/audio/sounds/PowerUp.mp3", Sound.class);
		manager.load("assets/audio/sounds/PowerUpSpawn.mp3", Sound.class);
		manager.load("assets/audio/sounds/SmashCStone.mp3", Sound.class);
		manager.load("assets/audio/sounds/StompOrHitEnemy.mp3", Sound.class);
		manager.load("assets/audio/sounds/StoneBump.mp3", Sound.class);
		manager.finishLoading();				// Boxes everything and says to finish loading assets for now (Synchronous method for AssetManager)
												// If we wanted to do asynchronous, we would NOT put manager.finishLoading()
		
		// Pass the game itself to PhantomLordMainMenu class, so it can set screens in the future
		// but it is our starting screen, so it MUST be set here!
		setScreen(new PhantomLordMainMenu(this));
	}

	@Override
	public void render () 
	{
		// Delegate render method to PlayScreen (or whatever screen is active at the time)
		super.render();
		
		// From https://github.com/libgdx/libgdx/wiki/Managing-your-assets
//		// Asynchronous part of Asset Manager Asset (To load assets inside the render method, I am not planning on using this though for now)
//		// calls the manager to continue to load your assets, 
//		// update returns boolean value stating T/F whether your assets are updated/loaded
//		if(manager.update())				// If Wanted to tell if all assets are loaded
//		{
//			// Code for any kind of stuff that requires those assets above
//		}
		
		// I am going to do "Synchronous" loading instead
	}
	
	@Override
	public void dispose() {
		super.dispose();
		manager.dispose();
		batch.dispose();
	}
}
