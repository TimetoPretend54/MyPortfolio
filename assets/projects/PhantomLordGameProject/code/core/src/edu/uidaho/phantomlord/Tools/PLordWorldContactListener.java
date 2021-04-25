package edu.uidaho.phantomlord.Tools;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.audio.Sound;
import com.badlogic.gdx.maps.MapObject;
import com.badlogic.gdx.physics.box2d.Contact;
import com.badlogic.gdx.physics.box2d.ContactImpulse;
import com.badlogic.gdx.physics.box2d.ContactListener;
import com.badlogic.gdx.physics.box2d.Fixture;
import com.badlogic.gdx.physics.box2d.Manifold;

import edu.uidaho.phantomlord.PhantomLord;
import edu.uidaho.phantomlord.Sprites.PhantomLordChar;
import edu.uidaho.phantomlord.Sprites.PowerUps.Cane;
import edu.uidaho.phantomlord.Sprites.Enemies.Enemy;
import edu.uidaho.phantomlord.Events.Goal;
import edu.uidaho.phantomlord.Scenes.PLordHud;
import edu.uidaho.phantomlord.Sprites.Items.Item;
import edu.uidaho.phantomlord.Sprites.Objects.TileObject;

//
// Idea for collision filtering from 
// http://www.aurelienribon.com/post/2011-07-box2d-tutorial-collision-filtering
//

public class PLordWorldContactListener implements ContactListener
{

	// Contact Listener is what gets called when two fixtures in box2D collide with each other
	
	@Override
	public void beginContact(Contact contact) 
	{
		// Gets called when fixtures begin to make a connection (begin to collide)
		
		// Contact contact has two fixtures fixture a and fixture b (fixtures in process of beginning contact)
		// Need to figure out which is which
		Fixture fixA = contact.getFixtureA();
		Fixture fixB = contact.getFixtureB();
		
		// Or-ing the category bits of both fixtures when they collide with each other
		// So if PLord Collides with a ground (GROUND_BIT = 1) and Himself (PLORD_BIT = 2)
		// It will be 00011 (1st one is PLord bit, and 2nd one is ground bit), which is value '3' as our
		// collision definition
		int collisionDefinition = fixA.getFilterData().categoryBits | fixB.getFilterData().categoryBits;
		
		/*********************************************************
		 *  MAKE SURE TO BREAK EACH CASE, had some annoying bugs
		 *  because forgot to for some of the cases
		 ********************************************************/
		
		switch (collisionDefinition)
		{
			// Check of PLord head hits these items:
			case PhantomLord.PLORDHEAD_BIT | PhantomLord.OBJECT_BIT:
                if(fixA.getFilterData().categoryBits == PhantomLord.PLORDHEAD_BIT)
                    ((TileObject) fixB.getUserData()).onPLordHeadHit((PhantomLordChar) fixA.getUserData());
                else
                    ((TileObject) fixA.getUserData()).onPLordHeadHit((PhantomLordChar) fixB.getUserData());
                break;
                
			// If PLord Collides with enemy (NOT the head, but the enemy body itself)
			case PhantomLord.PLORD_BIT | PhantomLord.ENEMY_BIT:
                if(fixA.getFilterData().categoryBits == PhantomLord.PLORD_BIT)
                    ((PhantomLordChar) fixA.getUserData()).pLordHit((Enemy)fixB.getUserData());
                else
                	((PhantomLordChar) fixB.getUserData()).pLordHit((Enemy)fixA.getUserData());
                break;
                
			case PhantomLord.EVENT_BIT | PhantomLord.PLORD_BIT:
	            if(fixA.getFilterData().categoryBits == PhantomLord.EVENT_BIT)
	                ((Goal) fixA.getUserData()).goalReached((PhantomLordChar)fixB.getUserData());
	            else
	            	((Goal) fixB.getUserData()).goalReached((PhantomLordChar)fixA.getUserData());
	            break;
                
			// if PLord_bit collides with enemy's head bit
			case PhantomLord.ENEMYHEAD_BIT | PhantomLord.PLORD_BIT:
				// Need to figure out which fixture is the enemy fixture
				if(fixA.getFilterData().categoryBits == PhantomLord.ENEMYHEAD_BIT)
					((Enemy)fixA.getUserData()).hitOnHead((PhantomLordChar) fixB.getUserData());		// Set user data to the object itself so can cast inside Enemy object
				else
					((Enemy)fixB.getUserData()).hitOnHead((PhantomLordChar) fixA.getUserData());		// Run hitOnHead with current fixture from Enemy class
				break;
				
			case PhantomLord.ENEMYHEAD_BIT | PhantomLord.WEAPON_BIT:
				// Play StompOrHitEnemy sound
				PhantomLord.manager.get("assets/audio/sounds/StompOrHitEnemy.mp3", Sound.class).play();
				PLordHud.addScore(200);
				// Need to figure out which fixture is the enemy fixture
				if(fixA.getFilterData().categoryBits == PhantomLord.ENEMYHEAD_BIT)
				{
					((Enemy)fixA.getUserData()).killEnemy();	// Set user data to the object itself so can cast inside Enemy object
					((Cane)fixB.getUserData()).toBeDestroyed = true;
				}
				else
				{
					((Enemy)fixB.getUserData()).killEnemy();		// Run hitOnHead with current fixture from Enemy class
					((Cane)fixA.getUserData()).toBeDestroyed = true;
				}
				break;
				
			case PhantomLord.ENEMY_BIT | PhantomLord.WEAPON_BIT:
				// Play StompOrHitEnemy sound
				PhantomLord.manager.get("assets/audio/sounds/StompOrHitEnemy.mp3", Sound.class).play();
				PLordHud.addScore(200);
				// Need to figure out which fixture is the enemy fixture
				if(fixA.getFilterData().categoryBits == PhantomLord.ENEMY_BIT)
				{
					((Enemy)fixA.getUserData()).killEnemy();	// Set user data to the object itself so can cast inside Enemy object
					((Cane)fixB.getUserData()).toBeDestroyed = true;
				}
				else
				{
					((Enemy)fixB.getUserData()).killEnemy();		// Run hitOnHead with current fixture from Enemy class
					((Cane)fixA.getUserData()).toBeDestroyed = true;
				}
				break;
				
			// If Enemy_bit collides with the Object_bit, which consists of stone objects and topOfarchitecture,
			// Reverse velocity, so the enemy turns around (should be good for now, for later development may 
			// to consider adding CrumbleStone and Candle, so enemy can hit a wall and turn around on anything)
				// Need to still figure out which fixture is the enemy bit first though
			case PhantomLord.ENEMY_BIT | PhantomLord.OBJECT_BIT:
				// Need to figure out which fixture is the enemy fixture
				if(fixA.getFilterData().categoryBits == PhantomLord.ENEMY_BIT)
					((Enemy)fixA.getUserData()).reverseVelocity(true, false);		// Reverse x-axis(true), don't care about y axis (false)
				else
					((Enemy)fixB.getUserData()).reverseVelocity(true, false);
				break;
				
			// Two enemies hit each other
			case PhantomLord.ENEMY_BIT | PhantomLord.ENEMY_BIT:
				// Determine if enemies should reverse velocity or one should get hurt by the other one
				((Enemy)fixA.getUserData()).onEnemyHit((Enemy) fixB.getUserData());	
				((Enemy)fixB.getUserData()).onEnemyHit((Enemy) fixA.getUserData());
				break;
				
			// If item bit collides with object bit, reverse velocity
			case PhantomLord.ITEM_BIT | PhantomLord.OBJECT_BIT:
				// Need to figure out which fixture is the item fixture;
				if(fixA.getFilterData().categoryBits == PhantomLord.ITEM_BIT)
					((Item)fixA.getUserData()).reverseVelocity(true, false);		// Reverse x-axis(true), don't care about y axis (false)
				else
					((Item)fixB.getUserData()).reverseVelocity(true, false);
				break;
		
			// If item bit collides with PLORD BIT
			case PhantomLord.ITEM_BIT | PhantomLord.PLORD_BIT:
				// Need to figure out which fixture is the item fixture
				if(fixA.getFilterData().categoryBits == PhantomLord.ITEM_BIT)
					((Item)fixA.getUserData()).use((PhantomLordChar) fixB.getUserData());		// Run 'use' from Item class using current fixture info
				else
					((Item)fixB.getUserData()).use((PhantomLordChar) fixA.getUserData());		// "Same"
				break;
				
			// If item bit collides with PLORDHEAD_BIT
			case PhantomLord.ITEM_BIT | PhantomLord.PLORDHEAD_BIT:
				// Need to figure out which fixture is the item fixture
				if(fixA.getFilterData().categoryBits == PhantomLord.ITEM_BIT)
					((Item)fixA.getUserData()).use((PhantomLordChar) fixB.getUserData());		// Run 'use' from Item class using current fixture info
				else
					((Item)fixB.getUserData()).use((PhantomLordChar) fixA.getUserData());		// "Same"
				break;
		}
	}

	@Override
	public void endContact(Contact contact)
	{
		// Gets called when fixtures disconnect their connection (end their collision)
		
		// Contact contact has two fixtures fixture a and fixture b in ending collision
		// Need to figure out which is which
	}

	// preSolve and poSolve wont really be of concern in the game
	
	@Override
	public void preSolve(Contact contact, Manifold oldManifold) 
	{
		// Gives opportunity to, once something has collided, can change the characteristics of 
		// that collision for any given reason
		
	}

	@Override
	public void postSolve(Contact contact, ContactImpulse impulse) 
	{
		// Gives results of what happened due to that collision (what angles fixtures went off
		// at and so on).
		
	}

}
