package edu.uidaho.phantomlord.Sprites.Items;

import com.badlogic.gdx.math.Vector2;

public class ItemDefinition 
{
	public Vector2 position;
	
	// Class type we don't know yet (the 'itemtype' Class)
	public Class<?> type;
	
	// Set up constructor
	public ItemDefinition(Vector2 position, Class<?> type)
	{
		this.position = position;
		this.type = type;
	}
}
