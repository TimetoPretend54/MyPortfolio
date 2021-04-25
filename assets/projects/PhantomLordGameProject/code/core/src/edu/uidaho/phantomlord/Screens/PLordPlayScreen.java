package edu.uidaho.phantomlord.Screens;

import java.util.LinkedList;
import java.util.concurrent.LinkedBlockingQueue;

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
import com.badlogic.gdx.graphics.g2d.TextureAtlas;
import com.badlogic.gdx.graphics.g2d.TextureRegion;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.maps.MapObject;
import com.badlogic.gdx.maps.objects.RectangleMapObject;
import com.badlogic.gdx.maps.tiled.TiledMap;
import com.badlogic.gdx.maps.tiled.TmxMapLoader;
import com.badlogic.gdx.maps.tiled.renderers.OrthogonalTiledMapRenderer;
import com.badlogic.gdx.maps.tiled.renderers.OrthogonalTiledMapRendererBleedFix;
import com.badlogic.gdx.math.Rectangle;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.BodyDef;
import com.badlogic.gdx.physics.box2d.Box2DDebugRenderer;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.badlogic.gdx.physics.box2d.PolygonShape;
import com.badlogic.gdx.physics.box2d.World;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.viewport.FitViewport;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import com.badlogic.gdx.utils.viewport.StretchViewport;
import com.badlogic.gdx.utils.viewport.Viewport;

import edu.uidaho.phantomlord.PhantomLord;
import edu.uidaho.phantomlord.Scenes.PLordHud;
import edu.uidaho.phantomlord.Scenes.Pause;
import edu.uidaho.phantomlord.Sprites.PhantomLordChar;
import edu.uidaho.phantomlord.Sprites.Enemies.Enemy;
import edu.uidaho.phantomlord.Sprites.Enemies.Pumpkin;
import edu.uidaho.phantomlord.Sprites.Items.CandyCorn;
import edu.uidaho.phantomlord.Sprites.Items.CandyCorn;
import edu.uidaho.phantomlord.Sprites.Items.GhostMask;
import edu.uidaho.phantomlord.Sprites.Items.HatCandy;
import edu.uidaho.phantomlord.Sprites.Items.Item;
import edu.uidaho.phantomlord.Sprites.Items.ItemDefinition;
import edu.uidaho.phantomlord.Sprites.PhantomLordChar.State;
import edu.uidaho.phantomlord.Sprites.PowerUps.Cane;
import edu.uidaho.phantomlord.Sprites.PowerUps.SpawningCane;
import edu.uidaho.phantomlord.Tools.Box2DWorldCreator;
import edu.uidaho.phantomlord.Tools.PLordWorldContactListener;

/*
 * Code inspired from
 * https://github.com/libgdx/libgdx/wiki/Extending-the-simple-game
 * "The Game Screen" section
 * NOTE: Since this instantiates many objects, refer to the class of
 * that object for more info
 */
public class PLordPlayScreen implements Screen
{
	public enum State
	{
	    PAUSE,
	    RUN,
	    RESUME
	}
	
	private State state = State.RUN;
	
	// Reference to the game, used to set Screens
	private PhantomLord game;
	
	// Load images from texture atlases using texture packer
	private TextureAtlas atlas;
	
	// Playscreen variables
	private OrthographicCamera gameCam;
	private Viewport gamePort;
	private PLordHud hud;
	private Pause pauseHud;

	// load our tile map into the game
	private TmxMapLoader maploader;
	// Reference to map itself
	private TiledMap map;
	// THis is what renders the map to the screen (using the one I modified, which fixes the texture bleeding)
	private OrthogonalTiledMapRendererBleedFix renderer;
	
	
	//Box2d Variables
	private World world;
	private Box2DDebugRenderer box2DdbugR;		// graphical representation of our fixtures and bodies in box2d world
	private Box2DWorldCreator worldCreator;
	
	// Sprites
	private PhantomLordChar player;
	
	// Music variable for setting up music to play in game
	private Music music;
	
	private boolean stateJump;
	private BitmapFont font = new BitmapFont();
	
	private ShapeRenderer shapeRenderer;
	
	// Item Variables
	private Array<Item> items;
	// Using this, because we are not setting priority on items and not needing PriorityQueue
	private LinkedList<ItemDefinition> itemsToSpawn;
	
    private Array<Cane> canes;
    private LinkedBlockingQueue<SpawningCane> caneSpawnQueue;
	
	// Brings in game class
	public PLordPlayScreen(PhantomLord game)
	{
		// Reads directly from "assets" folder in desktop/assets (since using desktop system for running)
		// Since NOT using Android need to use Gdx.file.internal and get absolute path, and also move
		// The assets folder from 'core' to the desired system (in this case the desktop folder)
		// Load in texture sprite pack from textureAtlas
		// Not using libGDX asset manager because not using many textures, not intensive for this many graphics
		// If have a alot of resources, use libGDX assetManager
		atlas = new TextureAtlas(Gdx.files.internal("assets/actors/PhantomLord_and_Enemies.pack").file().getAbsolutePath());
		
		this.game = game;
		
		// Resize Font
		font.getData().setScale(.4f);
		
		shapeRenderer = new ShapeRenderer();
		
		// Create new camera orthographic is 2D camera
		gameCam = new OrthographicCamera();
		
		
		/********************
		* Screen Options:	*
		*********************/
		
			// Set our viewport variable to stretch viewport type
			// virtual width and height are 800x400, respectively, everything is
			// assumed to be this way, no matter what the actual screen is, thus it 
			// stretches or squashes the screen
			//gamePort = new StretchViewport(800, 480, gameCam);
			
			// Set our viewport variable to stretch viewport type, doesn't take
			// width and height just shows more of our game world, which can be problematic
			//gamePort = new ScreenViewport(gameCam);
			
			// Set our viewport variable to stretch viewport type, doesn't take
			// Add bars to maintain aspect ratio for the width and height given (800x480)
			// It makes a black bar so you can Only see the screen that is allowed by the virtual
			// width and height
			// We are using this! so we can maintain aspect ratio
		gamePort = new FitViewport(PhantomLord.V_WIDTH / PhantomLord.PIXELS_PER_METER, PhantomLord.V_HEIGHT / PhantomLord.PIXELS_PER_METER, gameCam);
		
		// Create instance of our HUD
		hud = new PLordHud(game.batch);
		pauseHud = new Pause(game.batch);
		
		
		/****************************************
		 * Load in our "tmx" map and render it 	*
		 ****************************************/
		
		// object to hold TmxMapLoader, which will load in or map
		maploader = new TmxMapLoader();
		
		// Reads directly from "assets" folder in desktop/assets (since using desktop system for running)
		// Since NOT using Android need to use Gdx.file.internal and get absolute path, and also move
		// The assets folder from 'core' to the desired system (in this case the desktop folder)
		switch(PhantomLord.LevelNumber)
		{
			// Level 1
			case 1:
				map = maploader.load(Gdx.files.internal("assets/maps/PhantomLordMap1-1.tmx").file().getAbsolutePath());
				break;
			// Level 2
			case 2:
				map = maploader.load(Gdx.files.internal("assets/maps/PhantomLordMap1-2.tmx").file().getAbsolutePath());
				break;
			// Level 3
			case 3:
				map = maploader.load(Gdx.files.internal("assets/maps/PhantomLordMap1-3.tmx").file().getAbsolutePath());
				break;
			// Otherwise Level 4
			default:
				map = maploader.load(Gdx.files.internal("assets/maps/PhantomLordMap1-4.tmx").file().getAbsolutePath());
				break;
		}
		
		// Set up renderer to render the loaded "map" ((using the one I modified, which fixes the texture bleeding)
		renderer = new OrthogonalTiledMapRendererBleedFix(map, 1 / PhantomLord.PIXELS_PER_METER);
		
		// When we create camera, it defaults to coordinate 0,0 thus only in one quadrant (25%
		// of screen), need to center it (0 for z axis in follow declaration)
		gameCam.position.set(gamePort.getWorldWidth() / 2, gamePort.getWorldHeight() / 2, 0);
		
		// For gravity (Vector2), true for "sleep" object that are at rest (box2d doesn't 
		// calculate, inside its physics simulation, bodies that are at rest)
		world = new World(new Vector2(0,-10), true);
		
		// Set up debug renderer (To render it and dispose of it, uncomment line 336, and 461 as well)
		//box2DdbugR = new Box2DDebugRenderer();
		
		// Create bodies and fixtures in every corresponding 'object' in tiled map layers
		worldCreator = new Box2DWorldCreator(this);
		
		// Initialize PhantomLordChar class object
		player = new PhantomLordChar(this);
		
		// Set up the Contact Listener
		world.setContactListener(new PLordWorldContactListener());
		
		// Set up Music (Remember using static, IT MIGHT GIVE ERRORS, when testing if it does, fix it, otherwise don't worry)
		// Load in 'Intense(Compressed).mp3', the music for level 1, loop it and play it
		switch(PhantomLord.LevelNumber)
		{
			// Level 1 
			case 1:
				music = PhantomLord.manager.get("assets/audio/music/Menu-Upbeat.mp3", Music.class);
				break;
			// Level 2
			case 2:
				music = PhantomLord.manager.get("assets/audio/music/Love Upon Differences.mp3", Music.class);
				break;
			// Level 3
			case 3:
				music = PhantomLord.manager.get("assets/audio/music/Intense(Compressed).mp3", Music.class);
				break;
			// Otherwise Level 4
			default:
				music = PhantomLord.manager.get("assets/audio/music/Winter waltz.mp3", Music.class);
				break;
		}
		
		music.setLooping(true); 				// Loop the song
		music.play(); 							// Begin Playing the song
		
		// Initialize "Item" class variables
		items = new Array<Item>();
		itemsToSpawn = new LinkedList<ItemDefinition>();
		
		stateJump = false;
		
		// Ifor spawning cane
		canes = new Array<Cane>();
		caneSpawnQueue = new LinkedBlockingQueue<SpawningCane>();
	}
	
	/*
	 * Idea from
	 * https://gamedev.stackexchange.com/questions/103710/static-spawning-locations-for-players-creatures-items-in-tiled-map-editor
	 */
	public void spawnItem(ItemDefinition itemDef)
	{
		// Add item definition to the itemsToSpawn LinkedBlockingQueue<ItemDefinition
		itemsToSpawn.add(itemDef);
	}
	
	/*
	 * Idea from
	 * https://gamedev.stackexchange.com/questions/103710/static-spawning-locations-for-players-creatures-items-in-tiled-map-editor
	 */
	public void handleSpawningItems()
	{
		// If there are items to spawn
		if(itemsToSpawn.size() > 0)
		{
			ItemDefinition itemDef = itemsToSpawn.poll();		// Retrieves and removes head of queue (like a 'pop')
		
			// if the itemDef is the CandyCorn class, create new CandyCorn item and add it the the array of 'items'
			if(itemDef.type == CandyCorn.class)
			{
				items.add(new CandyCorn(this, itemDef.position.x, itemDef.position.y));
			}
			
			if(itemDef.type == GhostMask.class)
			{
				items.add(new GhostMask(this, itemDef.position.x, itemDef.position.y));
			}
			
			if(itemDef.type == HatCandy.class)
			{
				items.add(new HatCandy(this, itemDef.position.x, itemDef.position.y));
			}
		}
	}
	
    public void addSpawnCane(float x, float y, boolean movingRight) {
    	caneSpawnQueue.add(new SpawningCane(x, y, movingRight));
    }

    public void handleSpawningCane() {
        if (caneSpawnQueue.size() > 0) {
            SpawningCane spawningCane = caneSpawnQueue.poll();
            canes.add(new Cane(this, spawningCane.x, spawningCane.y, spawningCane.movingRight));
        }
    }

	
	public TextureAtlas getAtlas()
	{
		return atlas;
	}
	
    public OrthographicCamera getCamera() {
        return gameCam;
    }
	
	@Override
	public void show() 
	{
		// TODO Auto-generated method stub
		
	}
	
	public void handleInput(float dt)
	{
		// Two ways to move object in box2d, 
		// 	1) force = gradual increase/decrease in speed
		//	2) impulse = immediate change
		
		/*********************
		 * applyLinearImpulse
		 *********************/
		// 1st parameter asks for x and y for Impulse, we want just Y for jump
		// 2nd parameter is where in the body where we want to apply Impulse/Force, if apply off-center,
		//		there will be a torque to the body, change angluarlity, DONT WANT THAT, apply to CENTER of body
		// 3rd parameter is will this force/impulse wake the object up? we set it to true so it will (box2d
		//		doesn't check asleep objects only awake ones, saves times
		
		// If PLord is dead or has reached the goal, don't want user input, otherwise, allow user input
		if(player.currentState != PhantomLordChar.State.DEAD && player.currentState != PhantomLordChar.State.GOALREACHED)
		{
		    float maxSpeed = player.normalSpeedMax;
	        float force = player.normalForce;
	        
			// omly allow user to jump if PLord is not already jumping or falling
			if ( player.currentState != PhantomLordChar.State.FALLING && state != State.PAUSE)
			{
				if (Gdx.input.isKeyJustPressed(Input.Keys.SPACE) && player.currentState != PhantomLordChar.State.JUMPING )
				{
					if((player.currentState == PhantomLordChar.State.GOTHIT || player.currentState == PhantomLordChar.State.POWERUP|| player.currentState == PhantomLordChar.State.AVGUP) && player.box2DBody.getLinearVelocity().y == 0)
					{
			            player.keyPressedTime = 0;
			            
			            // Vecto2 only assign y part to 2.8, leave x part to 0, as we only want to apply the force to y axis, as we are jumping
						player.box2DBody.applyLinearImpulse(new Vector2(0, 2.8f), player.box2DBody.getWorldCenter(), true);
						PhantomLord.manager.get("assets/audio/sounds/Jump.mp3", Sound.class).play();
					}
					else if(player.currentState != PhantomLordChar.State.GOTHIT && player.currentState != PhantomLordChar.State.POWERUP && player.currentState != PhantomLordChar.State.AVGUP)
					{
			            player.keyPressedTime = 0;
			            
			            // Vecto2 only assign y part to 2.8, leave x part to 0, as we only want to apply the force to y axis, as we are jumping
						player.box2DBody.applyLinearImpulse(new Vector2(0, 2.8f), player.box2DBody.getWorldCenter(), true);
						PhantomLord.manager.get("assets/audio/sounds/Jump.mp3", Sound.class).play();
					}
				}
				
				if (Gdx.input.isKeyPressed(Input.Keys.SPACE) && ((player.currentState == PhantomLordChar.State.JUMPING) || (player.box2DBody.getLinearVelocity().y > 0 || player.box2DBody.getLinearVelocity().y < 0)))
				{
					if((player.currentState == PhantomLordChar.State.GOTHIT || player.currentState == PhantomLordChar.State.POWERUP || player.currentState == PhantomLordChar.State.AVGUP))
					{
						if (player.keyPressedTime > 0.1f && player.keyPressedTime < 0.55f) 
			            {
			                player.box2DBody.applyLinearImpulse(new Vector2(0.0f, 1.5f), player.box2DBody.getWorldCenter(), true);
			                player.keyPressedTime = 99.0f;
			            }
					}
					else if(player.currentState != PhantomLordChar.State.GOTHIT && player.currentState != PhantomLordChar.State.POWERUP && player.currentState != PhantomLordChar.State.AVGUP)
					{
						if (player.keyPressedTime > 0.1f && player.keyPressedTime < 0.55f) 
			            {
			                player.box2DBody.applyLinearImpulse(new Vector2(0.0f, 1.5f), player.box2DBody.getWorldCenter(), true);
			                player.keyPressedTime = 99.0f;
			            }
					}
				}
			}
			// Not justpressed, because player need to hold down key to continue moving unlike jumping
			// Also need to check (&&) that PhantomLord isn't going faster than specific speed (of x axis)
			// using getLinearVelocity, which returns the linear velocity of body in pixels per second (2 seems good)
		       // Accelerate
			if(state != State.PAUSE)
			{
		        if (Gdx.input.isKeyPressed(Input.Keys.ALT_RIGHT) && ( player.currentState != PhantomLordChar.State.FALLING &&  player.currentState != PhantomLordChar.State.JUMPING)) {
		            maxSpeed = player.fastSpeedMax;
		            force = player.fastForce;
		        }
				if (Gdx.input.isKeyPressed(Input.Keys.D) && player.box2DBody.getLinearVelocity().x < maxSpeed)
					player.box2DBody.applyLinearImpulse(new Vector2(force, 0), player.box2DBody.getWorldCenter(), true);
			
				// Same as right key, except negative and need to make greater than or equal to
				if (Gdx.input.isKeyPressed(Input.Keys.A) && player.box2DBody.getLinearVelocity().x > -maxSpeed)
					player.box2DBody.applyLinearImpulse(new Vector2(-force, 0), player.box2DBody.getWorldCenter(), true);
		       
				// Throw Cane
		        if (Gdx.input.isKeyJustPressed(Input.Keys.ALT_RIGHT) && player.isPLordPower() && player.caneTimer > player.caneInterval && !player.runThrowingAnimation && !player.pLordGotHit) {
		            player.caneTimer = 0;
		            player.runThrowingAnimation = true;
		        }
			}
			else if(state == State.PAUSE)
	        {
	        	if(Gdx.input.isKeyPressed(Input.Keys.ALT_RIGHT) && Gdx.input.isKeyPressed(Input.Keys.ENTER) && Gdx.input.isKeyPressed(Input.Keys.SHIFT_RIGHT) && !player.pLordAtGoal)
	        	{
					switch(PhantomLord.LevelNumber)
					{
						// Level 1
						case 1:
							// Stop the background music
							PhantomLord.manager.get("assets/audio/music/Menu-Upbeat.mp3", Music.class).stop();
							break;
						// Level 2
						case 2:
							PhantomLord.manager.get("assets/audio/music/Love Upon Differences.mp3", Music.class).stop();
							break;
						// Level 3
						case 3:
							PhantomLord.manager.get("assets/audio/music/Intense(Compressed).mp3", Music.class).stop();
							break;
						// Otherwise level 4
						default:
							// Stop the background music
							PhantomLord.manager.get("assets/audio/music/Winter waltz.mp3", Music.class).stop();
							break;
					}
					
					// Reset Level #
					PhantomLord.LevelNumber = 1;
					
					// reset score
					PhantomLord.score = 0;
					
					// Reset lives
					PhantomLord.lives = 3;
					
					// Reset candle count
					PhantomLord.candles = 0;
					
					// Reset PLord's State
					PhantomLord.pLordState = 1;
					
					// change to MainMenu Screen
					game.setScreen(new PhantomLordMainMenu(game));
					
					//dispose of all of our resources
					dispose();
	        	}
	        }
	        
	        // Pause
	        if(Gdx.input.isKeyJustPressed(Input.Keys.ESCAPE) && state == State.RUN)
			{
				state = State.PAUSE;
				
				// Play "Select Menu Item" sound
				PhantomLord.manager.get("assets/audio/sounds/MenuSelect.mp3", Sound.class).play();
				
				switch(PhantomLord.LevelNumber)
				{
					// Level 1
					case 1:
						// Stop the background music
						PhantomLord.manager.get("assets/audio/music/Menu-Upbeat.mp3", Music.class).pause();
						break;
					// Level 2
					case 2:
						PhantomLord.manager.get("assets/audio/music/Love Upon Differences.mp3", Music.class).pause();
						break;
					// Level 3
					case 3:
						PhantomLord.manager.get("assets/audio/music/Intense(Compressed).mp3", Music.class).pause();
						break;
					// Otherwise level 4
					default:
						// Stop the background music
						PhantomLord.manager.get("assets/audio/music/Winter waltz.mp3", Music.class).pause();
						break;
				}
			}
	        
	        // UnPause
	        else if(Gdx.input.isKeyJustPressed(Input.Keys.ESCAPE) && state == State.PAUSE)
			{
				state = State.RESUME;
				
				// Play "Select Menu Item" sound
				PhantomLord.manager.get("assets/audio/sounds/MenuSelect.mp3", Sound.class).play();
				
				switch(PhantomLord.LevelNumber)
				{
					// Level 1
					case 1:
						// Stop the background music
						PhantomLord.manager.get("assets/audio/music/Menu-Upbeat.mp3", Music.class).play();
						break;
					// Level 2
					case 2:
						PhantomLord.manager.get("assets/audio/music/Love Upon Differences.mp3", Music.class).play();
						break;
					// Level 3
					case 3:
						PhantomLord.manager.get("assets/audio/music/Intense(Compressed).mp3", Music.class).play();
						break;
					// Otherwise level 4
					default:
						// Stop the background music
						PhantomLord.manager.get("assets/audio/music/Winter waltz.mp3", Music.class).play();
						break;
				}
			}
		}
		else 
			return;
	}
	
	public void update(float dt)
	{
		// Handle user Input
		handleInput(dt);
		
		// Handle multiple spawns
		handleSpawningItems();			// Handle Spawning items
		handleSpawningCane();			// Handle spawning cane
		
		/********************************************
		 * RUN SIMULATION (world.step)
		 * 
		 * IMPORTANT IDEAS ABOUT HOW TO RUN PHYSICS 
		 * SIMULATION FAST ENOIGHT TO KEEP UP WITH 
		 * FAST COLLISIONS
		 *******************************************/
		// timestep, velocityiterations, positioniterations, higher the #,
		// the longer it takes to do the calculations, but the more precise
		// Takes 1 step in physics simulations (60 times per second), so it will 
		// simulate 60 times a second (the higher this is over 60, the slower it will go)
		// Increase positonIterations for better collision detection positions!
		// Increase velocityIteratons for better velocity detection ()
		
		// Bigger issue is just that the simulation is only getting called each frame once, 
		// so instead I want to call it 3 times each frame (for loop), that way my collision are
		// better and I don't get the issue of PLord stomping the enemy but also getting hit
		// at the same time (due to simulation not being able to keep up with the collisions)
		// Basically now we will do about 180 (dt / 3, which dt is the # of seconds since last frame,
		// which since game is 60 fps, should be around 1/60, thus 1/60 / 3 = 1/180), which would slow
		//the simulation down, if we said to set 1/180 each frame, but instead we are saying in the 
		//for loop to call 1/180 3 times in a frame, thus keeping the orignal speed of the game 
		// thus 1/180 steps a second rather than the default 1/60f, which is 60 steps a second, can do
		// more steps a second easily with this for loop if need be, WILL IMPACT PERFORANCE
		/*
		 *  Code for the for loop slightly modified from
		 *  https://stackoverflow.com/questions/5362291/fast-moving-bodies-in-box2d-sometimes-pass-through-each-other
		 */
		for (int i = 0; i < 3; i++)
		{
			world.step(dt / 3, 8, 8);
		}
		
		// Update the sprite for Phantom Lord Character's position and animation frame
		player.update(dt);
		
		// Update all Pumpkins
		for(Enemy enemy : worldCreator.getEnemies())
		{
			enemy.update(dt, this);								// Update the sprites for all pumpkins in the world
			
			if (enemy.getX() < player.getX() + 224 / PhantomLord.PIXELS_PER_METER)	
			{												// (If pumpkin is at player) + (basically if it is shown 
				if(enemy.box2DBody != null)
					enemy.box2DBody.setActive(true);			// ALMOST on the screen is what we want, so add the # required to 
			}												// it by the screen then set it to active, so it begins moving)
		}													// 12 blocks in a screen, 16 pixels, 12X16 = 192, spawn enemy 2 
															// blocks after that which = 32 more pixels 192 + 32 = 224 pixels additional 
															// after player			
		
		// Update Items, go through the array of items and update each one
		for(Item item : items)
		{
			item.update(dt);
		}
		
		// Get the items that are NOT spawned thought boxes
		for(Item item2 : worldCreator.getItems())
		{
			item2.update(dt);
		}
		
		// Update canes
		for(Cane cane : canes)
		{
			cane.update(dt);
		}
		
		// Pass delta time the HUD
		hud.update(dt);
		
		// Every time PhantomLord moves, want to track him with gameCam
		// Only want psoiton.x, DONT want camera moving up and down with him
		// DONT MOVE CAMERA WITH PLORD IF HE DIES
		if(player.currentState != PhantomLordChar.State.DEAD)
			gameCam.position.x = player.box2DBody.getPosition().x;
		
		//always update camera any time it moves
		gameCam.update();
		
		// need to let map renderer know what to render
		renderer.setView(gameCam);
		
		cleanUpDestroyedObjects();
	}
	
    private void cleanUpDestroyedObjects() {
        for (int i = 0; i < items.size; i++) {
            if (items.get(i).isDestroyed()) {
              //  items.removeIndex(i);
                
            }
        }

        for (int i = 0; i < canes.size; i++) {
            if (canes.get(i).isDestroyed()) {
                canes.removeValue(canes.get(i), true);
               // canes.removeIndex(i);
            }
        }
    }

	@Override
	public void render(float delta) 
	{
		switch(state)
		{
			case RUN:
			{
				// Render getting called over and over, need to first call 'update' then
				update(delta);
				
				// Clear the screen
				Gdx.gl.glClearColor(0,0,0,1);					// Clear color and alpha (Having all 1st three parameters (RGB, 4th is Alpha) 0 makes black background)
				Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);		// Actually clear the screen
				
				// Render the map (only after clearing the screen)
				renderer.render();
				
				// Render our Box2dDDebugLines
				//box2DdbugR.render(world, gameCam.combined);
					
				// Set batch to the gameCam, which is only what the game camera see, before rendering
				// aka the the camera that follows the player
				game.batch.setProjectionMatrix(gameCam.combined);
				
				/**************************
				 * Draw textures to screen
				 **************************/
				game.batch.begin();				// Begin our batch
				
				player.draw(game.batch);		// Give sprite game batch to draw itself on from player object, which holds our PhantomLordChar Class, thus drawing our 'Phantom Lord Character'
				
				// Draw all our pumpkin sprites in the game World
				for(Enemy enemy : worldCreator.getEnemies())
					enemy.draw(game.batch);
				
				// Draw all items in the "items" array (which holds all the current items)
				for(Item item : items)
					item.draw(game.batch);
				
				// Get the items that are NOT spawned thought boxes
				for(Item item2 : worldCreator.getItems())
				{
					item2.draw(game.batch);
				}
				
				// Draw Cane
				for(Cane cane : canes)
					cane.draw(game.batch);
					
				/************************
				 *  End Drawing
				 ***********************/
				game.batch.end();
				
				// Tell game batch to now recognize where camera is in our game world and only 
				// render what the camera can see, and also what the camera HUD sees
				game.batch.setProjectionMatrix(hud.stage.getCamera().combined);
				
				// Draw the stage (HUD)
				hud.stage.draw();
				
				// For drawing images
				hud.draw(delta);
				
				// If PLord Died
				if(pLordDeath())
				{
					PhantomLord.lives--;							// remove one life
					
					// GAME OVER
					if(PhantomLord.lives == 0)
					{
						// Reset Level #
						PhantomLord.LevelNumber = 1;
						
						// reset score
						PhantomLord.score = 0;
						
						// Reset lives
						PhantomLord.lives = 3;
						
						// Reset candle count
						PhantomLord.candles = 0;
						
						// change to gameOverscren
						game.setScreen(new GameOverScreen(game));
						
						//dispose of all of our resources
						dispose();
					}
					else
					{
						// change to level transition screen
						game.setScreen(new LevelTransitionScreen(game));
						
						//dispose of all of our resources
						dispose();
					}
					
				}
				
				// If we finished the level
				if(levelFinished())
				{
					// We beat the game!
					if(PhantomLord.LevelNumber == 4)
					{
						// Reset Level #
						PhantomLord.LevelNumber = 1;
						
						// reset score
						PhantomLord.score = 0;
						
						// Reset lives
						PhantomLord.lives = 3;
						
						// Reset candle count
						PhantomLord.candles = 0;
						
						// Reset PLord's State
						PhantomLord.pLordState = 1;
						
						game.setScreen(new GameCompleteScreen(game));
					}
					// More Levels to Go!
					else
					{
						PhantomLord.LevelNumber++;
						
						// Set screen to transition screen, as we are about to enter a level from previous level, but need a transition screen
						game.setScreen(new LevelTransitionScreen((PhantomLord) game));
						
						// dispose of all resources
						dispose();
					}
				}
				
				break;
			}
			case PAUSE:
			{
				// Clear the screen
				Gdx.gl.glClearColor(0,0,0,1);					// Clear color and alpha (Having all 1st three parameters (RGB, 4th is Alpha) 0 makes black background)
				Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);		// Actually clear the screen
				
				// Render the map (only after clearing the screen)
				renderer.render();
	
					
				// Set batch to the gameCam, which is only what the game camera see, before rendering
				// aka the the camera that follows the player
				game.batch.setProjectionMatrix(gameCam.combined);
				
				// Tell game batch to now recognize where camera is in our game world and only 
				// render what the camera can see, and also what the camera HUD sees
				game.batch.setProjectionMatrix(hud.stage.getCamera().combined);
				
				// Check to see if player presses escape to unpause
			    handleInput(delta);
			    
			    // Dim the screen by drawing a transparent black retangle over whole screen
				Gdx.gl.glEnable(GL20.GL_BLEND);
			    Gdx.gl.glBlendFunc(GL20.GL_SRC_ALPHA, GL20.GL_ONE_MINUS_SRC_ALPHA);
			    shapeRenderer.begin(ShapeRenderer.ShapeType.Filled);
			    shapeRenderer.setColor(new Color(0, 0, 0, 0.5f));
			    shapeRenderer.rect(0, 0, PhantomLord.V_WIDTH * PhantomLord.PIXELS_PER_METER, PhantomLord.V_HEIGHT * PhantomLord.PIXELS_PER_METER);
			    shapeRenderer.end();
			    Gdx.gl.glDisable(GL20.GL_BLEND);
			    
				game.batch.begin();				// Begin our batch
				font.draw(game.batch, "Press R.ALT+ENTER+R.SHIFT to Return to Main Menu", 10, 10);
				game.batch.end();
			    
				// Draw the stage (HUD) (put it after dimming the screen, so it still appears bright)
				pauseHud.stage.draw();
			    
				break;
			}
			case RESUME:
				state = State.RUN;
				break;
			}
	}
	
	public boolean pLordDeath()
	{
		// If player is dead, and has been dead for at least 3 seconds
		if(player.currentState == PhantomLordChar.State.DEAD && player.getStateTimer() > 3)
		{
			PhantomLord.pLordState = 1;
			return true;
		}
		else
			return false;
	}
	
	public boolean levelFinished()
	{
		// If PLord make it to the end part of level and disappeared, then change screen to level complete screen after 1 second has passed
		if(player.currentState == PhantomLordChar.State.LEVELFINISHED && player.getStateTimer() > 1)
			return true;
		else
			return false;
	}
	
	public Box2DWorldCreator getWorldCreator()
	{
		return worldCreator;
	}
	
	public PLordHud getHud()
	{
		return hud;
	}

	@Override
	public void resize(int width, int height) 
	{
		// When changing the size of our screen, make sure viewport gets adjusted, so know
		// what the actual screen size is
		gamePort.update(width, height);
	}

	public TiledMap getMap()
	{
		return map;
	}
	
	public World getWorld()
	{
		return world;
	}
	
	@Override
	public void pause() 
	{
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
		// Dispose of resources
		map.dispose();
		renderer.dispose();
		world.dispose();
		//box2DdbugR.dispose();
		hud.dispose();
	}
}
