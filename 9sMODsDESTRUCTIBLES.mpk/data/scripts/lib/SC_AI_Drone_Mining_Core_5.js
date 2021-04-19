using(console);
using(npc);

include(SC_AI_Drone_Mining_Rout_For_5.js);
include(SC_Utilities.js);
include(SC_AI_Drone_Manager_Utilities.js);
include(SC_AI_Drone_Mining_InventoryChecks_5.js);

var currentIndex = -1;
var initOnce = 1;
var CToD = null;

var arrayOfDroneFormation = [];
var arrayOfFriendlies = [];
var lockdist = 50;

var droneSwitch;

var arrayOfLastDroneCommand = [];
var currentCToD;
var currentObjective;

var SC_AI_Drone_Mining_Core_5 =
{
	AICoreInit: function (SHIP_ID, droneIndex, playerName, player_id)
	{
		var jumpGateToTravel = ship.EnteringJumpgate(player_id);

		if (jumpGateToTravel != 0) 
		{
			npc.TravelThroughJumpgate(SHIP_ID, jumpGateToTravel);
		}

		var docked = ship.IsDocked(player_id);
		var isNPCDocked = npc.IsOnBase(SHIP_ID);

		if (docked && !isNPCDocked)
		{					
			if (npc.IsStickToPoint(SHIP_ID) || npc.IsStickToObject(SHIP_ID)) {
				npc.Unstick(SHIP_ID);
			}

			if (npc.IsLocked(SHIP_ID) || npc.IsFriendlyLocked(SHIP_ID)) {
				npc.Unlock(SHIP_ID);
			}
			//npc.StopFollowingRoute(player_id);

			var current_base = ship.GetCurrentBase(player_id);
			npc.FollowRouteToBase(SHIP_ID, current_base);

			if (npc.IsAtBaseEntry(SHIP_ID, current_base))
			{
				npc.StopFollowingRoute(SHIP_ID);
				npc.EnterBase(SHIP_ID, current_base);
			}
		}					
		else if (isNPCDocked && !docked)
		{
			npc.StopFollowingRoute(SHIP_ID);
			npc.LeaveBase(SHIP_ID);
		}				
		else if(!isNPCDocked && !docked)
		{		
			npc.StopFollowingRoute(SHIP_ID);
			droneSwitch = storage.GetGlobal("currentCommand_switch_" + SHIP_ID);
			
			if(droneSwitch == 1||arrayOfLastDroneCommand.length < droneIndex|| arrayOfLastDroneCommand[0] == null)
			{
				CToD = storage.GetGlobal("currentCommand" + SHIP_ID);
				if(arrayOfLastDroneCommand.length < droneIndex|| arrayOfLastDroneCommand[0] == null)
				{
					arrayOfLastDroneCommand.push(CToD);
				}
				else
				{	
					arrayOfLastDroneCommand[droneIndex] = CToD;
				}
			}
			else if(arrayOfLastDroneCommand[droneIndex] == null )
			{
				CToD = storage.GetGlobal("currentCommand" + SHIP_ID);
				arrayOfLastDroneCommand[droneIndex] = CToD;
			}
			currentCToD = arrayOfLastDroneCommand[droneIndex];
		
			if (currentCToD != null)
			{
				if (currentCToD.id != null) 
				{
					if (currentCToD.id == SHIP_ID) 
					{
						if (npc.CountObjectives(currentCToD.id) > 0) // isSelfActive == true => compute Objective
						{
							currentObjective = npc.GetCurrentObjective(currentCToD.id);

							AICoreComputeObjective(currentObjective, currentCToD.addFriend);

							if (currentCToD.lastCommand != currentCToD.command) 
							{
								//console.PrintError("new command");
								if (npc.IsStickToPoint(currentCToD.id) || npc.IsStickToObject(currentCToD.id)) {
									npc.Unstick(currentCToD.id);
								}

								if (npc.IsLocked(currentCToD.id) || npc.IsFriendlyLocked(currentCToD.id)) {
									npc.Unlock(currentCToD.id);
								}

								npc.StopFollowingRoute(currentCToD.id);
								npc.NextObjective(currentCToD.id);
								npc.CleanObjectives(currentCToD.id);
								//npc.InstantStop(CToD.id);
								npc.Stop(currentCToD.id);

								//CToD = storage.GetGlobal("currentCommand" + CToD.id);
								currentCToD.lastCommand = currentCToD.command;
								storage.SetGlobal("currentCommand" + currentCToD.id, currentCToD);
							}
						}
						else
						{	
                            if (currentCToD.command == 0 || currentCToD.command == 1 || currentCToD.command == 2 || currentCToD.command == 11) 
							{									
								var sys_id = npc.GetCurrentSystemID(currentCToD.id);
							
								npc.AddObjective(currentCToD.id, "get_in_formation",
									{
										nid: currentCToD.id,
										pid: player_id,
										sid: sys_id,
										bid: currentCToD.baseVicinity,
										pName: playerName,
										command: currentCToD.command,
										formation: currentCToD.formation,
										speedSwitch: 0,
										maxPlayerForwardSpeed: null,
										maxNPCForwardSpeed: null,
										maxNPCStrafeSpeed: null,
										wepPropType: currentCToD.wepPropType,
										wepDistType: currentCToD.wepDistType,
										droneIndex: droneIndex,
										aDock: currentCToD.aDock,
										mDock: currentCToD.sDock,
										aDef: currentCToD.aDef,
										mDef: currentCToD.sDef,
										aRep: currentCToD.aRep,
										mRep: currentCToD.sRep,
										minHP: currentCToD.minHP,
										sDefLoc: currentCToD.sDefLoc
									});
							}
							else if (currentCToD.command == 9)
							{
								var sys_id = npc.GetCurrentSystemID(currentCToD.id);

								npc.AddObjective(currentCToD.id, "return_to_player",
									{
										nid: currentCToD.id,
										pid: player_id,
										sid: sys_id,
										bid: currentCToD.baseVicinity,
										pName: playerName,
										command: currentCToD.command,
										formation: currentCToD.formation,
										speedSwitch: 0,
										maxPlayerForwardSpeed: null,
										maxNPCForwardSpeed: null,
										maxNPCStrafeSpeed: null,
										wepPropType: currentCToD.wepPropType,
										wepDistType: currentCToD.wepDistType,
										droneIndex: droneIndex,
										aDock: currentCToD.aDock,
										mDock: currentCToD.sDock,
										aDef: currentCToD.aDef,
										mDef: currentCToD.sDef,
										aRep: currentCToD.aRep,
										mRep: currentCToD.sRep,
										minHP: currentCToD.minHP,
										sDefLoc: currentCToD.sDefLoc
									});
							}
							/*else if (currentCToD.command == 2)
							{
								console.PrintError("current command 1");
								var sys_id = npc.GetCurrentSystemID(currentCToD.id);

								npc.AddObjective(currentCToD.id, "station_vicinity",
									{
										nid: currentCToD.id,
										pid: player_id,
										sid: sys_id,
										bid: currentCToD.baseVicinity,
										pName: playerName,
										command: currentCToD.command,
										formation: currentCToD.formation,
										speedSwitch: 0,
										maxPlayerForwardSpeed: null,
										maxNPCForwardSpeed: null,
										maxNPCStrafeSpeed: null,
										wepPropType: currentCToD.wepPropType,
										wepDistType: currentCToD.wepDistType,
										droneIndex: droneIndex,
										aDock: currentCToD.aDock,
										mDock: currentCToD.sDock,
										aDef: currentCToD.aDef,
										mDef: currentCToD.sDef,
										aRep: currentCToD.aRep,
										mRep: currentCToD.sRep,
										minHP: currentCToD.minHP,
										sDefLoc: currentCToD.sDefLoc
									});
							}*/
						}
					}
					else {
					console.PrintError("null DATA0");
						return;
					}
				}
				else {
				console.PrintError("null DATA1");
					return;
				}
			}
			else 
			{
				console.PrintError("null DATA2");
				return;
			}
		}		
	}
};

function AICoreComputeObjective(currentObjective, addFriendSwitch)
{
	if (currentObjective == null)
	{
		return;
	}

	switch (currentObjective.name) 
	{
		case "get_in_formation":
			{
				SC_AI_Drone_Mining_Rout_For_5.AIRoutineInit(currentObjective, addFriendSwitch);
				break;
			}
		case "return_to_player":
			{          
				var distDroneToPlayer = SC_Utilities.GetDistance(currentObjective.nid, currentObjective.pid);

				if(distDroneToPlayer > 3)
				{
					npc.FollowRouteToObject(currentObjective.nid, currentObjective.pid);
				}
				else
				{   
					var droneIndex = npc.GetTag(currentObjective.nid, "droneIndex");
					var	currentObjective = npc.GetCurrentObjective(currentObjective.nid);
					SC_AI_Drone_Manager_Utilities.setRemovedFromGlobalDroneArrayMining(currentObjective.droneIndex, currentObjective.wepDistType, currentObjective.wepPropType,currentObjective.formation);				 
						
					var PlayerContainerID = items.GetGameObjectContainerId(currentObjective.pid);
							
					var droneXML = npc.GetTag(currentObjective.nid, "xml");					
					//items.AddItem(PlayerContainerID, droneXML, 1);
					var obj = ship.AddItem(currentObjective.pid, droneXML, 1);
					generator.RemoveShip(currentObjective.nid);


					/*var current_Player_Space = SC_AI_Drone_Mining_InventoryChecks_1.AIIsPlayerCargoFull(currentObjective.pid);

					if(current_Player_Space > 0)
					{
						var droneIndex = npc.GetTag(currentObjective.nid, "droneIndex");
						var	currentObjective = npc.GetCurrentObjective(currentObjective.nid);
						SC_AI_Drone_Manager_Utilities.setRemovedFromGlobalDroneArrayMining(currentObjective.droneIndex, currentObjective.wepDistType, currentObjective.wepPropType,currentObjective.formation);				 
						
						var PlayerContainerID = items.GetGameObjectContainerId(currentObjective.pid);
							
						var droneXML = npc.GetTag(currentObjective.nid, "xml");					
						//items.AddItem(PlayerContainerID, droneXML, 1);
						var obj = ship.AddItem(currentObjective.pid, droneXML, 1);
					}
					else
					{
						storage.SetGlobal("currentCommand_switch_" + currentObjective.nid, 1);
						storage.SetGlobal("currentCommand" + currentObjective.nid, { id: currentObjective.nid, command: 0, formation: currentObjective.formation, lastCommand: -1, wepPropType: currentObjective.wepPropType, wepDistType: currentObjective.wepDistType, droneIndex: currentObjective.droneIndex, addFriend: 0, baseVicinity: -1 });					
						game.SendNotification(currentObjective.pName, "OnBoard Computer", "Cannot Recover drone because your inventory is full!");
					}*/
					//generator.RemoveShip(currentObjective.nid);

					//items.AddItem(PlayerContainerID, itemsInCargoOfNPC[t].xml_id, q);
				}

				break;
			}

		case "station_vicinity":
			{          
				break;
			}

		case "player_has_docked":
			{
				break;
			}

		case "player_has_jumped":
			{
				break;
			}
	}
}