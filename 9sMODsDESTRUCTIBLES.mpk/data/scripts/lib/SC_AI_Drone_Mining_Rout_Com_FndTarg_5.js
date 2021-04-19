using(console);
using(player);
using(npc);
using(ship);
using(timer);
using(items);

include(SC_Utilities.js);
include(SC_AI_Drone_Mining_InventoryChecks_5.js);
include(SC_AI_Drone_Mining_Rout_For_SFWPP_5.js);

var listOfAsteroids = [];
var lastListOfAsteroids = [];
var test;

var SC_AI_Drone_Mining_Rout_Com_FndTarg_5 =
{
    npcFindTarget: function (cData)
    {		
		//console.PrintError(cData.lLSwtc);
		if(cData.lLSwtc == 0)
		{
			var isNpcInventoryFull = SC_AI_Drone_Mining_InventoryChecks_5.AICheckStateOfInventory(cData.objt.nid);

			if (isNpcInventoryFull == 0)
			{
				listOfAsteroids = npc.GetSystemAsteroidsByDistanceToNPC(cData.objt.nid, 20);
				if (listOfAsteroids != null)
				{	
					if (listOfAsteroids.length > 0)
					{	
						if (listOfAsteroids[0] != null)
						{	
							lastListOfAsteroids = listOfAsteroids;

							var coords = game.GetObjectCoordinates(cData.objt.sid, lastListOfAsteroids[0]);

							var distToAst = SC_Utilities.GetDistance(coords,cData.nData.nCoord);
							var dirToAstX = (coords.x - cData.nData.nCoord.x) / distToAst;
							var dirToAstY = (coords.y - cData.nData.nCoord.y) / distToAst;

							if(distToAst <= 25)
							{
								var waypointX = coords.x - (dirToAstX * 4);
								var waypointY = coords.y - (dirToAstY * 4);
								cData.cAstP = {x:waypointX,y:waypointY};
								cData.cAstPD = distToAst;
								//cData.cFWP = {x:waypointX,y:waypointY};
								//cData.cFWPD =distToAst;
								//cData.dog = null;
								//cData.log = null;
								//cData.dog = [];
								//cData.log = [];

								cData.mSwtc = 1;
								cData.lLSwtc = 1;
							}
							else
							{
								storage.SetGlobal("currentCommand_switch_" + cData.objt.nid, 1);
								storage.SetGlobal("currentCommand" + cData.objt.nid, { id: cData.objt.nid, command: 0, formation: cData.objt.formation, lastCommand: -1, wepPropType: cData.objt.wepPropType, wepDistType: cData.objt.wepDistType, droneIndex: cData.objt.droneIndex, addFriend: 0, baseVicinity: -1 });					
							}							
						}
						else
						{
							//console.PrintError("no Asteroids");
							storage.SetGlobal("currentCommand_switch_" + cData.objt.nid, 1);
							storage.SetGlobal("currentCommand" + cData.objt.nid, { id: cData.objt.nid, command: 0, formation: cData.objt.formation, lastCommand: -1, wepPropType: cData.objt.wepPropType, wepDistType: cData.objt.wepDistType, droneIndex: cData.objt.droneIndex, addFriend: 0, baseVicinity: -1 });
						}
					}
					else
					{
						//console.PrintError("no Asteroids");
                 		storage.SetGlobal("currentCommand_switch_" + cData.objt.nid, 1);
						storage.SetGlobal("currentCommand" + cData.objt.nid, { id: cData.objt.nid, command: 0, formation: cData.objt.formation, lastCommand: -1, wepPropType: cData.objt.wepPropType, wepDistType: cData.objt.wepDistType, droneIndex: cData.objt.droneIndex, addFriend: 0, baseVicinity: -1 });
					}
				}
				else
				{
					//console.PrintError("no Asteroids");
               		storage.SetGlobal("currentCommand_switch_" + cData.objt.nid, 1);
					storage.SetGlobal("currentCommand" + cData.objt.nid, { id: cData.objt.nid, command: 0, formation: cData.objt.formation, lastCommand: -1, wepPropType: cData.objt.wepPropType, wepDistType: cData.objt.wepDistType, droneIndex: cData.objt.droneIndex, addFriend: 0, baseVicinity: -1 });
				}  
			}
			else
			{
				var isPlayerInventoryFull = SC_AI_Drone_Mining_InventoryChecks_5.AIIsPlayerCargoFull(cData.objt.pid);
            					
				if(isPlayerInventoryFull.swtc == 0)
				{
					//go back to player to deposit
					cData.lLSwtc = 5;
					cData.mSwtc = 1;
				}
				else
				{
					//playerInventory is full. go back in formation. 
					storage.SetGlobal("currentCommand_switch_" + cData.objt.nid, 1);
					storage.SetGlobal("currentCommand" + cData.objt.nid, { id: cData.objt.nid, command: 0, formation: cData.objt.formation, lastCommand: -1, wepPropType: cData.objt.wepPropType, wepDistType: cData.objt.wepDistType, droneIndex: cData.objt.droneIndex, addFriend: 0, baseVicinity: -1 });
				}
			}
		}
		else if(cData.lLSwtc == 1)
		{		
			var distToAst = SC_Utilities.GetDistance(cData.cAstP,cData.nData.nCoord);
			cData.cAstPD = distToAst;
			cData.cFWP = cData.cAstP;
			cData.cFWPD =distToAst;

			if(distToAst > 6 && distToAst <= 25)
			{
				npc.StickToPoint(cData.objt.nid, cData.cAstP.x, cData.cAstP.y, 0);
			}
			else if(distToAst > 25)
			{
				storage.SetGlobal("currentCommand_switch_" + cData.objt.nid, 1);
				storage.SetGlobal("currentCommand" + cData.objt.nid, { id: cData.objt.nid, command: 0, formation: cData.objt.formation, lastCommand: -1, wepPropType: cData.objt.wepPropType, wepDistType: cData.objt.wepDistType, droneIndex: cData.objt.droneIndex, addFriend: 0, baseVicinity: -1 });				
			}
			else
			{
				npc.Unstick(cData.objt.nid);
				npc.InstantStop(cData.objt.nid);
				cData.lLSwtc = 2;
			}
		}
		else if(cData.lLSwtc == 2)
		{
			var distToAst = SC_Utilities.GetDistance(cData.cAstP,cData.nData.nCoord);
			cData.cAstPD = distToAst;
			cData.cFWP = cData.cAstP;
			cData.cFWPD = distToAst;

			if(distToAst <= 6)
			{
				//activate pathfind.
				if(cData.mSwtc != 9)
				{	
					cData = SC_AI_Drone_Mining_Rout_For_SFWPP_5.npcPathfindToWaypoint(cData);
				}
				else
				{
					//console.PrintError("waypoint Reached");
					cData.mSwtc = 1;
					cData.lLSwtc = 3;
				}
			}
		}
		else if(cData.lLSwtc == 3)
		{	
			npc.InstantStop(cData.objt.nid);

            if (lastListOfAsteroids != null)
            {
                if (lastListOfAsteroids.length > 0)
                {
                    if (lastListOfAsteroids[0] != null)
                    {	
						if (game.IsAsteroidExists(lastListOfAsteroids[0]) == true)                        
						{
							npc.DeviceActivateOnObject(cData.objt.nid, "device_mining_civilian_NPC", lastListOfAsteroids[0]);			
							storage.SetGlobal("timer_Switch" + cData.objt.nid, 0);
							//test = timer.SetTimer(6, "OnTimerWorked",{id:cData.objt.nid},1);		
							timer.ClearTimer("timer_" + cData.objt.nid);
							timer.AddOrUpdate("timer_" + cData.objt.nid, 6, "OnTimerWorked", {id:cData.objt.nid}, 1)
							cData.lLSwtc = 4;
						}
						else
						{
							cData.lLSwtc = 0;
						}
					}
					else
					{
						cData.lLSwtc = 0;
					}
				}
				else
				{
					cData.lLSwtc = 0;
				}
			}
			else
			{
				cData.lLSwtc = 0;
			}
			//console.PrintError("startMining");		
		}
		else if(cData.lLSwtc == 4)
		{	
			var switcher = storage.GetGlobal("timer_Switch" + cData.objt.nid);

			if(switcher == 1)
			{
				var isNpcInventoryFull = SC_AI_Drone_Mining_InventoryChecks_5.AICheckStateOfInventory(cData.objt.nid);

				if (isNpcInventoryFull == 0)
				{
					cData.lLSwtc = 3;
				}
				else
				{
					var isPlayerInventoryFull = SC_AI_Drone_Mining_InventoryChecks_5.AIIsPlayerCargoFull(cData.objt.pid);
            					
					if(isPlayerInventoryFull.swtc == 0)
					{
						//go back to player to deposit
						//cData.cAstP = cData.pData.pCoord;						
						cData.lLSwtc = 5;
					}
					else
					{
						//playerInventory is full. go back in formation. 
						storage.SetGlobal("currentCommand_switch_" + cData.objt.nid, 1);
						storage.SetGlobal("currentCommand" + cData.objt.nid, { id: cData.objt.nid, command: 0, formation: cData.objt.formation, lastCommand: -1, wepPropType: cData.objt.wepPropType, wepDistType: cData.objt.wepDistType, droneIndex: cData.objt.droneIndex, addFriend: 0, baseVicinity: -1 });
					}
				}
			}			
		}
		else if(cData.lLSwtc == 5)
		{
			var distToPlayer = SC_Utilities.GetDistance(cData.pData.pCoord,cData.nData.nCoord);
			var distToAst = SC_Utilities.GetDistance(cData.cAstP,cData.nData.nCoord);
			cData.cAstPD = distToAst;

			var dirToPlayerX = (cData.pData.pCoord.x - cData.nData.nCoord.x)/distToPlayer;
			var dirToPlayerY = (cData.pData.pCoord.y - cData.nData.nCoord.y)/distToPlayer;

			if(distToAst <= 4)
			{
			
				var OffSetX = cData.nData.nCoord.x + (dirToPlayerX * 3.5);
				var OffSetY = cData.nData.nCoord.y + (dirToPlayerY * 3.5);
				var OffSet = {x:OffSetX,y:OffSetY};
				var distToPoint= SC_Utilities.GetDistance(OffSet,cData.nData.nCoord);

				cData.cFWP = OffSet;
				cData.cFWPD = distToPoint;

				if(cData.mSwtc != 9)
				{						
					cData = SC_AI_Drone_Mining_Rout_For_SFWPP_5.npcPathfindToWaypoint(cData);
				}
				else 
				{
					if(distToPlayer <= 4)
					{
						cData.mSwtc = 1;				
						cData.lLSwtc = 7;
					}
					else
					{
						npc.StickToPoint(cData.objt.nid, cData.pData.pCoord.x + (dirToPlayerX * 2), cData.pData.pCoord.y+ (dirToPlayerY * 2), 0);
					}					
				}
			}
			else
			{	
				if(distToPlayer <= 4)
				{
					cData.mSwtc = 1;				
					cData.lLSwtc = 7;
				}
				else
				{
					npc.StickToPoint(cData.objt.nid, cData.pData.pCoord.x + (dirToPlayerX * 2), cData.pData.pCoord.y+ (dirToPlayerY * 2), 0);
				}
			}
		}
		else if(cData.lLSwtc == 7)
		{
			npc.InstantStop(cData.objt.nid);
			//console.PrintError("deposit in Player Cargo");
			AIDepositMineralsInsidePlayersCargo(cData);
			cData.lLSwtc = 0;
		}
	
		return cData;
    }
};

function OnTimerWorked(data)
{   
	storage.SetGlobal("timer_Switch" + data.id, 1);
}

function AIDepositMineralsInsidePlayersCargo(cData)
{
	var NPCContainerID = items.GetGameObjectContainerId(cData.objt.nid);
	var itemsInCargoOfNPC = items.GetItemsAndCargo(NPCContainerID);
	var PlayerContainerID = items.GetGameObjectContainerId(cData.objt.pid);

	var totalQ = [];
	for (var t = 0; t < itemsInCargoOfNPC.length; t++) 
	{
        var q = itemsInCargoOfNPC[t].quantity;
        //items.RemoveCargo(NPCContainerID, itemsInCargoOfNPC[t].xml_id, q);
        //items.AddItem(PlayerContainerID, itemsInCargoOfNPC[t].xml_id, q);
		totalQ.push(q);
    }

	var isPlayerInventoryFull = SC_AI_Drone_Mining_InventoryChecks_5.AIIsPlayerCargoFull(cData.objt.pid);

	for (var t = 0; t < itemsInCargoOfNPC.length; t++) 
	{
		var q = itemsInCargoOfNPC[t].quantity;
		if(q < isPlayerInventoryFull.space)
		{
			items.RemoveCargo(NPCContainerID, itemsInCargoOfNPC[t].xml_id, q);
			items.AddItem(PlayerContainerID, itemsInCargoOfNPC[t].xml_id, q);
			isPlayerInventoryFull.space -= q;
		}
		else
		{
			items.RemoveCargo(NPCContainerID, itemsInCargoOfNPC[t].xml_id, isPlayerInventoryFull.space);
			items.AddItem(PlayerContainerID, itemsInCargoOfNPC[t].xml_id, isPlayerInventoryFull.space);
		}
    }
}