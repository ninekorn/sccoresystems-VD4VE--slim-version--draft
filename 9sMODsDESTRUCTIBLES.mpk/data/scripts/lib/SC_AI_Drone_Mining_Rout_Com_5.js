using(console);

include(SC_AI_Drone_Mining_Rout_For_SFWP_5.js);
include(SC_AI_Drone_Mining_Rout_Com_FndTarg_5.js);
include(SC_AI_Drone_Mining_InventoryChecks_5.js);
var tempLceData = null;
var dataToReturn = null;

var SC_AI_Drone_Mining_Rout_Com_5 =
{
    AICombatRoutine: function (cData, ceData)
    {  
		if(cData.objt.command == 0)
		{
			cData = SC_AI_Drone_Mining_Rout_For_SFWP_5.npcStayInFormation(cData);
		}
		else if(cData.objt.command == 11)
		{
			tempLceData = SC_AI_Drone_Mining_Rout_Com_FndTarg_5.npcFindTarget(cData);
			cData = tempLceData;
		}
		else if(cData.objt.command == 2)
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
					cData = SC_AI_Drone_Mining_Rout_For_SFWPP_1.npcPathfindToWaypoint(cData);
				}
				else 
				{
					if(distToPlayer <= 4)
					{
						npc.InstantStop(cData.objt.nid);
						AIDepositMineralsInsidePlayersCargo(cData);
						storage.SetGlobal("currentCommand_switch_" + cData.objt.nid, 1);
						storage.SetGlobal("currentCommand" + cData.objt.nid, { id: cData.objt.nid, command: 0, formation: cData.objt.formation, lastCommand: -1, wepPropType: cData.objt.wepPropType, wepDistType: cData.objt.wepDistType, droneIndex: cData.objt.droneIndex, addFriend: 0, baseVicinity: -1 });
					
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
					npc.InstantStop(cData.objt.nid);
					AIDepositMineralsInsidePlayersCargo(cData);
					storage.SetGlobal("currentCommand_switch_" + cData.objt.nid, 1);
					storage.SetGlobal("currentCommand" + cData.objt.nid, { id: cData.objt.nid, command: 0, formation: cData.objt.formation, lastCommand: -1, wepPropType: cData.objt.wepPropType, wepDistType: cData.objt.wepDistType, droneIndex: cData.objt.droneIndex, addFriend: 0, baseVicinity: -1 });					
				}
				else
				{
					npc.StickToPoint(cData.objt.nid, cData.pData.pCoord.x + (dirToPlayerX * 2), cData.pData.pCoord.y+ (dirToPlayerY * 2), 0);
				}
			}
		}

        dataToReturn = { forData: cData, comData: ceData };
        return dataToReturn;
    }
};

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