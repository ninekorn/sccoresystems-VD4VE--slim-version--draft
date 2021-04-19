using(relations);
using(npc);
using(console);
using(ship);
using(generator);
using(game);
using(items);
using(player);
using(storage);

include(SC_AI_Drone_Manager_Utilities.js);

var state;
var input;
var quest_drone_id = 0;
var quest_system_id = 0;
var quest_base_id = 0;
var amIInDialogue = true;

var stater;
var inputer;
var whatever = true;
var testingSomething = true;

var SwitchForMining = false;
var droneReleaseCounter = 0;
var listOfDrones = [];
var quantity = 0;

var listOfCombatDrones = [];
var listOfMiningDrones = [];
var listOfHealerDrones = [];
var currentChoiceIndex;

var droneAIMining = "SC_Drone_Mining_AI_";

var droneNameCombat = "mining";

var droneXMLCombat = "";//"xml_drone_combat";

var selectedDroneType = "";
var selectedDroneAI = "";
var selectedDroneXML = "";

var frontR = "front right";
var frontL = "front left";
var backR = "back right";
var backL = "back left";
var middleB = "middle back";

var arrayOfStringPos = [];
var rr = 0;

var test00 = 0;
var coliss;
var testArray = [];
var totalReleasedObjects = 0;

var droneHealer = "healer";
var droneCombat = "combat";
var droneMining = "mining";
var chosenDrone = "";

var max = 6;
var min = 1;
var behavior = 12;

var currentReleasedDrones = -1;

var quantityOfDrones = 0;
var quantityOfDronesToRelease = 0;
var chosenPosition = -1;
var arrayOfDrones;
var switchForMultipleMiningDrones = 0;
var totalDronesQuant = 0;
var tempDroneCombatArray = [];

var lastInput = -1;
var droneCounterer = 0;
var arrayOfPoserer;

function OnCheckRequirements()
{
    var npc_id = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npc_id, "class") == "droneManagerMining")
    {
        return true;
    }
    return false;
}

function OnDialogue(args)
{
    var npc_id = topic.GetCurrentNpcShipId();
    var player_id = PLAYER_SHIP;

    var name = game.GetShipOwner(player_id);

    var coords = ship.GetCoordinates(player_id);
    var sys_id = ship.GetSystemID(player_id);
    input = topic.GetInput();
    state = topic.GetState();

    var isPlayerShipDocked = ship.IsDocked(player_id);

    //var currentDroneCounter = storage.GetGlobal("data" + player_id);

    //console.PrintError(input + " input");
    //console.PrintError(state + " state");

    if (state == 0)
    {
        if (!isPlayerShipDocked)
        {
            if (input == NO_INPUT || input == -1 )
            {       
                totalDronesQuant = 0;
                droneAIMining = "SC_Drone_Mining_AI_";
                //selectedDroneAI = droneAIRepair;
                selectedDroneType = "mining";
                selectedDroneXML = "";
                tempDroneCombatArray = [];
				droneCounter = 0;
				arrayOfPoserer = [];
                //var currentDroneCounter = storage.GetGlobal("data" + player_id);
                //var arrayOfDroneData = SC_AI_Drone_Manager_Utilities.getIDSOfDrones(player_id);

                var currentDroneCounter = SC_AI_Drone_Manager_Utilities.resetGlobalArray(player_id);
                //var array = { pos: arrayOfPoser[i].pos, id: arrayOfPoser[i].id, formation: arrayOfPoser[i].formation };

                if (currentDroneCounter.droneCounter > 0 && currentDroneCounter.droneCounter < 5)
                {
					topic.AddChoice(1, "Release Drone");		
					topic.AddChoice(2, "Recover Drone");
				    topic.AddChoice(300, "Drone Go Mining");
                    topic.AddChoice(302, "Drone Stop Mining");
                    topic.AddChoice(303, "Drone Deposit Minerals in Player Cargo");                   			
                    topic.AddChoice(11, "Go back to the OnBoard Computer");
					
                    //topic.AddChoice(801, "Recall Drone");
                }
				else if(currentDroneCounter.droneCounter == 0)
				{
					topic.AddChoice(1, "Release Drone");		               			
					topic.AddChoice(11, "Go back to the OnBoard Computer");
				}
                else if(currentDroneCounter.droneCounter == 5)
                {
					topic.AddChoice(2, "Recover Drone");
				    topic.AddChoice(300, "Drone Go Mining");
                    topic.AddChoice(302, "Drone Stop Mining");
                    topic.AddChoice(303, "Drone Deposit Minerals in Player Cargo");
                    //topic.AddPhrase("You cannot release other Drones! The On-Board Computer doesn't have enough capacity to manage more Drones than what are currently released.");
					topic.AddChoice(11, "Go back to the OnBoard Computer");
                }
       
                //selectedDroneXML = droneXMLCombat;
            }

            else if (input == 9)
            {
                topic.DialogueBreak();
            }

            else if (input == 11)
            {
                player.StartDialogue(name, "OnBoardComputer");
            }

            else if (input == 1)
            {
                var playerContainerID = items.GetGameObjectContainerId(player_id);
                var itemsAndCargo = items.GetItemsAndCargo(playerContainerID);

                for (var i = 0; i < itemsAndCargo.length; i++)
                {
                    if (itemsAndCargo[i].xml_id.substring(0, 12) == "drone_mining")
                    {
                        //mystring = mystring.replace(/\/r/g, '');
                        //var chosenDrone = itemsAndCargo[i].xml_id.replace('_', '');

                        var tempXMLID = itemsAndCargo[i].xml_id;

                        tempXMLID = tempXMLID.replace(/_/g, " ");

                        topic.AddChoice(800 + totalDronesQuant, " Select " + tempXMLID); //+ itemsAndCargo[i].xml_id //totalDronesQuant
                        tempDroneCombatArray.push(itemsAndCargo[i].xml_id);
                        totalDronesQuant++;       
                    }
                }
              
                topic.AddChoice(lastInput, "Go back!");
                lastInput = -1;
            }


			else if (input == 2)
            {		

                var currentDroneData = storage.GetGlobal("data" + player_id);
				arrayOfPoserer = currentDroneData.arrayOfPos;
				
				for (var i = 0; i < arrayOfPoserer.length; i++)
				{
					if (arrayOfPoserer[i].id != -1)
					{
						var shipExists = generator.ShipExists(arrayOfPoserer[i].id);

						if (shipExists && ship.GetCurrentValue(arrayOfPoserer[i].id, "structure") > 0)
						{
							if (player_id == npc.GetTag(arrayOfPoserer[i].id, "ownerPlayerShipId") &&  npc.GetTag(arrayOfPoserer[i].id, "typeOfDrone") == "mining")
							{								
								var nameOfDrone = game.GetShipOwner(arrayOfPoserer[i].id);

								var forma = npc.GetTag(arrayOfPoserer[i].id, "formation");
							
								topic.AddChoice(700 + i, "Recover " + nameOfDrone + " drone at formation " + forma);
								
								//storage.SetGlobal("currentCommand_switch_" + id1, 1);
								//storage.SetGlobal("currentCommand" + id1, { id: id1, command: 0, formation: chosenPosition, lastCommand: 0, wepPropType: weaponPropulsionType, wepDistType: weaponDistanceType, droneIndex: index, addFriend: 0, baseVicinity: -1 }); //, behavior: behavior

								arrayOfPoserer[i].id = arrayOfPoserer[i].id;
								arrayOfPoserer[i].pos = 1;								
							}
							/*else
							{
								arrayOfPoserer[i].pos = 0;
								arrayOfPoserer[i].id = -1;
							}*/                
						}
						else
						{
							arrayOfPoserer[i].pos = 0;
							arrayOfPoserer[i].id = -1;
						}
					}
					else {
						arrayOfPoserer[i].pos = 0;
						arrayOfPoserer[i].id = -1;
					}
					droneCounterer++;
				}

            }

			
            else if (input >= 700 && input <= 700 + droneCounterer)
            {
				var currentSelectedDroneIndex = input - 700;
				var currentDroneData = storage.GetGlobal("data" + player_id);
				var arrayOfPoser = currentDroneData.arrayOfPos;

				var currentSelectedDrone = arrayOfPoser[currentSelectedDroneIndex].id;
				arrayOfPoser[currentSelectedDroneIndex].id = -1;
				currentDroneData.arrayOfPos = arrayOfPoser;
				storage.SetGlobal("data" + player_id, currentDroneData);

				var wepProp = npc.GetTag(currentSelectedDrone, "wepPropType");
				var wepDist = npc.GetTag(currentSelectedDrone, "wepDistType");
				var droneIndex =  npc.GetTag(currentSelectedDrone, "droneIndex");
				var forma = npc.GetTag(currentSelectedDrone, "formation");

				storage.SetGlobal("currentCommand_switch_" + currentSelectedDrone, 1);
				storage.SetGlobal("currentCommand" + currentSelectedDrone, { id: currentSelectedDrone, command: 9, formation: forma, lastCommand: -1, wepPropType: wepProp, wepDistType: wepDist, droneIndex: droneIndex, addFriend: 0, baseVicinity: -1 }); //, behavior: behavior
			}
















			//------------MINING DRONES--------------------
			//------------MINING DRONES--------------------
			//------------MINING DRONES--------------------
            else if (input == 300)
            {
                var currentDroneCounter = SC_AI_Drone_Manager_Utilities.resetGlobalArray(player_id);

                arrayOfDrones = SC_AI_Drone_Manager_Utilities.getIDSOfDrones(player_id, selectedDroneType);

                if (currentDroneCounter.droneCounter > 0 || arrayOfDrones.length > 0)
                {
                    var quantityOfDronesToSendMining = arrayOfDrones.length;

                    if (quantityOfDronesToSendMining > 0)
                    {
                        if (quantityOfDronesToSendMining <= 1)
                        {
                            for (var i = 0; i < arrayOfDrones.length; i++)
                            {
                                var test = generator.ShipExists(arrayOfDrones[i].id);

                                if (test && npc.GetTag(arrayOfDrones[i].id, "typeOfDrone") == "mining")
                                {
                                    var droneCurrentCommandList = storage.GetGlobal("currentCommand" + arrayOfDrones[i].id);
                                    var droneFormationPos = droneCurrentCommandList.formation;

                                    if (droneCurrentCommandList.command != 1) {
                                        topic.AddChoice(310 + i, "Send drone ID" + arrayOfDrones[i].id + " located at " + arrayOfStringPos[i] + " to go mine minerals!");
                                    }
                                    else if (droneCurrentCommandList.command == 1) {
                                        topic.AddPhrase("The drone ID" + arrayOfDrones[i].id + " located at " + arrayOfStringPos[i] + " is already mining!");
                                    }
                                }
                            }
                            //topic.AddChoice(301, "Which drone?");
                        }
                        else if (quantityOfDronesToSendMining > 1 && quantityOfDronesToSendMining <= 5)
                        {
                            //topic.AddChoice(301, "Which drones");

                            for (var i = 0; i < arrayOfDrones.length; i++) {
                                var test = generator.ShipExists(arrayOfDrones[i].id);

                                if (test && npc.GetTag(arrayOfDrones[i].id, "typeOfDrone") == "mining") {
                                    var droneCurrentCommandList = storage.GetGlobal("currentCommand" + arrayOfDrones[i].id);
                                    var droneFormationPos = droneCurrentCommandList.formation;

                                    if (droneCurrentCommandList.command == 0) {
                                        topic.AddChoice(310 + i, "Send drone ID" + arrayOfDrones[i].id + " located at " + arrayOfStringPos[i] + " to go mine minerals!");
                                    }
                                    else if (droneCurrentCommandList.command == 1) {
                                        topic.AddPhrase("The drone ID" + arrayOfDrones[i].id + " located at " + arrayOfStringPos[i] + " is already mining!");
                                    }
                                }
                            }
                        }
                    }
                    else {
                        topic.AddPhrase("You do not have mining drones!");
                    }
                }
                else {
                    topic.AddPhrase("You need to release drones for them to mine!");
                    topic.AddChoice(800, "Release Drone");
                }
            }

            else if (input == 301)
            {
                var currentDroneCounter = SC_AI_Drone_Manager_Utilities.resetGlobalArray(player_id);
                arrayOfDrones = SC_AI_Drone_Manager_Utilities.getIDSOfDrones(player_id, selectedDroneType);

                for (var i = 0; i < arrayOfDrones.length; i++)
                {
                    var test = generator.ShipExists(arrayOfDrones[i].id);

                    if (test &&  npc.GetTag(arrayOfDrones[i].id, "typeOfDrone") == "mining")
                    {
                        var droneCurrentCommandList = storage.GetGlobal("currentCommand" + arrayOfDrones[i].id);
                        var droneFormationPos = droneCurrentCommandList.formation;

                        if (droneCurrentCommandList.command == 0)
                        {
                            topic.AddChoice(310 + i, "Send drone ID" + arrayOfDrones[i].id + " located at " + arrayOfStringPos[i] + " to go mine minerals!");
                        }
                        else if (droneCurrentCommandList.command == 1)
                        {
                            topic.AddPhrase("The drone ID" + arrayOfDrones[i].id + " located at " + arrayOfStringPos[i] + " is already mining!");
                        }
                    }
                }
            }

            else if (input >= 310 && input <= 314)
            {
                var indexOfDrone = input - 310;
                var droneCurrentCommandList = storage.GetGlobal("currentCommand" + arrayOfDrones[indexOfDrone].id);
                droneCurrentCommandList.command = 11;
                storage.SetGlobal("currentCommand" + arrayOfDrones[indexOfDrone].id, droneCurrentCommandList);

                topic.AddChoice(301, "Send more drones to mine!");
                topic.AddChoice(9, " Quit the drone manager!");
            }

            else if (input == 302) {
                var currentDroneCounter = SC_AI_Drone_Manager_Utilities.resetGlobalArray(player_id);
                arrayOfDrones = SC_AI_Drone_Manager_Utilities.getIDSOfDrones(player_id, selectedDroneType);

                for (var i = 0; i < arrayOfDrones.length; i++) {
                    var test = generator.ShipExists(arrayOfDrones[i].id);

                    if (test&&  npc.GetTag(arrayOfDrones[i].id, "typeOfDrone") == "mining") 
					{
                        var droneCurrentCommandList = storage.GetGlobal("currentCommand" + arrayOfDrones[i].id);
                        var droneFormationPos = droneCurrentCommandList.formation;

                        if (droneCurrentCommandList.command != 0)
						{
                            topic.AddChoice(315 + i, "Send drone ID" + arrayOfDrones[i].id + " that is mining " + arrayOfStringPos[i] + " back in formation!");
                        }
                        else if (droneCurrentCommandList.command == 1) {
                            topic.AddPhrase("The drone ID" + arrayOfDrones[i].id + " located at " + arrayOfStringPos[i] + " is already back in it's formation!");
                        }
                    }
                }
            }

            else if (input >= 315 && input <= 319) {
                var indexOfDrone = input - 315;
                var droneCurrentCommandList = storage.GetGlobal("currentCommand" + arrayOfDrones[indexOfDrone].id);
                droneCurrentCommandList.command = 0;
                storage.SetGlobal("currentCommand" + arrayOfDrones[indexOfDrone].id, droneCurrentCommandList);

                topic.AddChoice(302, "Recall more drones!");
                topic.AddChoice(9, " Quit the drone manager!");
            }

            else if (input == 303) {
                var currentDroneCounter = SC_AI_Drone_Manager_Utilities.resetGlobalArray(player_id);

                arrayOfDrones = SC_AI_Drone_Manager_Utilities.getIDSOfDrones(player_id, selectedDroneType);

                if (currentDroneCounter.droneCounter > 0 || arrayOfDrones.length > 0) {
                    var quantityOfDronesToSendMining = arrayOfDrones.length;

                    if (quantityOfDronesToSendMining > 0) {
                        if (quantityOfDronesToSendMining <= 1) {
                            topic.AddChoice(304, "Which drone?");
                        }
                        else if (quantityOfDronesToSendMining > 1 && quantityOfDronesToSendMining <= 5) {
                            topic.AddChoice(304, "Which drones");
                        }
                    }
                    else {
                        topic.AddPhrase("You do not have mining drones!");
                    }
                }
                else {
                    topic.AddPhrase("You need to release drones for them to deposit anything!");
                    topic.AddChoice(800, "Release Drone");
                }
            }


            else if (input == 304) {
                var currentDroneCounter = SC_AI_Drone_Manager_Utilities.resetGlobalArray(player_id);
                arrayOfDrones = SC_AI_Drone_Manager_Utilities.getIDSOfDrones(player_id, selectedDroneType);

                for (var i = 0; i < arrayOfDrones.length; i++) {
                    var test = generator.ShipExists(arrayOfDrones[i].id);

                    if (test &&  npc.GetTag(arrayOfDrones[i].id, "typeOfDrone") == "mining") 
					{
                        var droneCurrentCommandList = storage.GetGlobal("currentCommand" + arrayOfDrones[i].id);
                        var droneFormationPos = droneCurrentCommandList.formation;

                        topic.AddChoice(320 + i, "Send drone ID" + arrayOfDrones[i].id + " located at " + arrayOfStringPos[i] + " to deposit cargo!");
                    }
                }
            }

            else if (input >= 320 && input <= 324) {
                var indexOfDrone = input - 320;
                var droneCurrentCommandList = storage.GetGlobal("currentCommand" + arrayOfDrones[indexOfDrone].id);
                droneCurrentCommandList.command = 2;
                storage.SetGlobal("currentCommand" + arrayOfDrones[indexOfDrone].id, droneCurrentCommandList);

                topic.AddChoice(302, "Recall more drones!");
                topic.AddChoice(9, " Quit the drone manager!");
            }
			//------------MINING DRONES--------------------
			//------------MINING DRONES--------------------
			//------------MINING DRONES--------------------


















            else if (input >= 800 && input <= 800 + totalDronesQuant)
            {
                var currentSelectedDroneIndex = input - 800;
                selectedDroneXML = tempDroneCombatArray[currentSelectedDroneIndex];

                //var obj = generator.GetItemByXmlID(selectedDroneXML);

                var playerContainerID = items.GetGameObjectContainerId(player_id);
                var itemsAndCargo = items.GetItemsAndCargo(playerContainerID);

                for (var i = 0; i < itemsAndCargo.length; i++)
                {
                    if (itemsAndCargo[i].xml_id == selectedDroneXML)
                    {
                        quantityOfDrones = itemsAndCargo[i].quantity;
                        break;
                    }
                }
                
                if (quantityOfDrones > 0)
                {
                    topic.AddPhrase("Choose a quantity to release");

                    if (quantityOfDrones <= 1)
                    {
                        topic.AddChoice(820, "1");
                    }
                    else if (quantityOfDrones <= 2) {
                        topic.AddChoice(820, "1");
                        topic.AddChoice(821, "2");
                    }
                    else if (quantityOfDrones <= 3) {
                        topic.AddChoice(820, "1");
                        topic.AddChoice(821, "2");
                        topic.AddChoice(822, "3");
                    }
                    else if (quantityOfDrones <= 4) {
                        topic.AddChoice(820, "1");
                        topic.AddChoice(821, "2");
                        topic.AddChoice(822, "3");
                        topic.AddChoice(823, "4");
                    }
                    else if (quantityOfDrones <= 5) {
                        topic.AddChoice(820, "1");
                        topic.AddChoice(821, "2");
                        topic.AddChoice(822, "3");
                        topic.AddChoice(823, "4");
                        topic.AddChoice(824, "5");
                    }
                }
                else {
                    topic.AddPhrase("You do not have drones!");
                }
                topic.AddChoice(lastInput, "Go back!");
                lastInput = -1;
            }

            else if (input == 820)
            {
                quantityOfDronesToRelease = 1;
                var currentDroneCounter = storage.GetGlobal("data" + player_id);
                var droneCounter = currentDroneCounter.droneCounter;

                if (quantityOfDronesToRelease <= 5 - droneCounter) {

                    topic.AddPhrase("1 drone selected! Choose its position in the formation."); // now choose its position in the formation.
                    //topic.AddChoice(830, "Choose a Position");
                    var currentDroneCounter = storage.GetGlobal("data" + player_id);
                    var arrayOfPoser = currentDroneCounter.arrayOfPos;

                    if (arrayOfPoser[0].pos == 0) {
                        topic.AddChoice(1000, " Random Position");
                    }

                    if (arrayOfPoser[1].pos == 0) {
                        topic.AddChoice(1001, " Front Right");

                    }

                    if (arrayOfPoser[2].pos == 0) {
                        topic.AddChoice(1002, " Front Left");

                    }

                    if (arrayOfPoser[3].pos == 0) {
                        topic.AddChoice(1003, " Back Right");
                    }

                    if (arrayOfPoser[4].pos == 0) {
                        topic.AddChoice(1004, " Back Left");

                    }
                    if (arrayOfPoser[5].pos == 0) {
                        topic.AddChoice(1005, " Middle Back");
                    }
                }
                else {
                    topic.AddPhrase("You cannot release that many drones. Please make another selection.");
                }
                topic.AddChoice(lastInput, "Go back!");
                lastInput = -1;
            }

            else if (input == 821)
            {
                var currentDroneCounter = storage.GetGlobal("data" + player_id);
                var droneCounter = currentDroneCounter.droneCounter;
                quantityOfDronesToRelease = 2;
                if (quantityOfDronesToRelease <= 5 - droneCounter) {

                    topic.AddPhrase("2 drone selected! Choose its position in the formation."); // now choose its position in the formation.
                    topic.AddChoice(831, "Choose a Position");
                }
                else {
                    topic.AddPhrase("You cannot release that many drones. Please make another selection.");
                }
                topic.AddChoice(lastInput, "Go back!");
                lastInput = -1;
            }
            else if (input == 822)
            {
                var currentDroneCounter = storage.GetGlobal("data" + player_id);
                var droneCounter = currentDroneCounter.droneCounter;
                quantityOfDronesToRelease = 3;
                if (quantityOfDronesToRelease <= 5 - droneCounter) {

                    topic.AddPhrase("3 drone selected! Choose its position in the formation."); // now choose its position in the formation.
                    topic.AddChoice(831, "Choose a Position");
                }
                else {
                    topic.AddPhrase("You cannot release that many drones. Please make another selection.");
                }
                topic.AddChoice(lastInput, "Go back!");
                lastInput = -1;
            }

            else if (input == 823) {
                var currentDroneCounter = storage.GetGlobal("data" + player_id);
                var droneCounter = currentDroneCounter.droneCounter;
                quantityOfDronesToRelease = 4;
                if (quantityOfDronesToRelease <= 5 - droneCounter) {

                    topic.AddPhrase("4 drone selected! Choose its position in the formation."); // now choose its position in the formation.
                    topic.AddChoice(831, "Choose a Position");
                }
                else {
                    topic.AddPhrase("You cannot release that many drones. Please make another selection.");
                }
                topic.AddChoice(lastInput, "Go back!");
                lastInput = -1;
            }
            else if (input == 824) {
                var currentDroneCounter = storage.GetGlobal("data" + player_id);
                var droneCounter = currentDroneCounter.droneCounter;
                quantityOfDronesToRelease = 5;
                if (quantityOfDronesToRelease <= 5 - droneCounter) {

                    topic.AddPhrase("5 drone selected! Choose its position in the formation."); // now choose its position in the formation.
                    topic.AddChoice(831, "Choose a Position");
                }
                else {
                    topic.AddPhrase("You cannot release that many drones. Please make another selection.");
                }
                lastInput = -1;
            }
            else if (input == 831) {
                topic.AddPhrase("Choose a position!");
                topic.AddChoice(1000, " Random Position");
                topic.AddChoice(lastInput, "Go back!");
                lastInput = -1;
            }

          /*else if (input == 10)
            {
                behavior = 0;
                topic.AddPhrase("Behavior changed to Offensive Everything!");
            }
            else if (input == 11) 
            {
                behavior = 1;
                topic.AddPhrase("Behavior changed to Offensive Partial!");
            }
            else if (input == 12) 
            {
                behavior = 2;
                topic.AddPhrase("Behavior changed to Defensive!");
            }
            else if (input == 13) 
            {
                behavior = 3;
                topic.AddPhrase("Behavior changed to Idle!");
            }*/

            else if (input == 1000 || input == 1001 || input == 1002 || input == 1003 || input == 1004 || input == 1005) {
                var maxQuantity = 0;

                maxQuantity = quantityOfDronesToRelease;
				var tempDroneXML = selectedDroneXML;
                if (maxQuantity > 0)
                {
					
                    for (var r = 0; r < maxQuantity; r++)
                    {
                        if (input == 1000)
                        {
                            chosenPosition = SC_AI_Drone_Manager_Utilities.randomInArray(player_id);
                        }
                        else
                        {
                            chosenPosition = input - 1000;
                        }

                        var randomPos = Math.floor(Math.random() * (360 - 0) + 0);

                        //itemsAndCargo[i].xml_id.substring(0, 12) == "drone_combat"

                        //var currentDroneItemStart = selectedDroneXML.substring(0, 12);

                        var weaponPropulsionType = "";
                        var weaponDistanceType = "";
                        var index = -1;

						weaponPropulsionType = null;
						weaponDistanceType = null;
						selectedDroneAI = droneAIMining + chosenPosition;
						//console.PrintError(selectedDroneXML);
						selectedDroneXML = "xml_" + tempDroneXML;

						index = SC_AI_Drone_Manager_Utilities.setInitialGlobalArrayMining(weaponDistanceType, chosenPosition);

						var level = ship.GetLevel(player_id);

                        var id1 = generator.AddNPCShipToSystem(selectedDroneType, selectedDroneAI, level, selectedDroneXML, sys_id, coords.x + (5 * Math.cos(randomPos * Math.PI / 180)), coords.y + (5 * Math.cos(randomPos * Math.PI / 180)), { ownerPlayerShipId: player_id, class: "drone", droneIndex: index, wepPropType: weaponPropulsionType, wepDistType: weaponDistanceType,typeOfDrone:"mining", aDock: 1, sDock: 0, aDef: 0, mDef: 0, sDefLoc: 0, aRep: 0, mRep: 0, minHP: 0.15,formation:chosenPosition,xml:tempDroneXML  });
                        //ship.RemoveItemByTypeCount(player_id, "drone_mining", 1);
                        generator.SetNPCAvatarImg(id1, "avatars/unique/drone.png");
						relations.SetShipFaction(id1, "none");
						storage.SetGlobal("currentCommand_switch_" + id1, 1);
                        storage.SetGlobal("currentCommand" + id1, { id: id1, command: 0, formation: chosenPosition, lastCommand: -1, wepPropType: weaponPropulsionType, wepDistType: weaponDistanceType, droneIndex: index, addFriend: 0, baseVicinity: -1 }); //, behavior: behavior

                        //TO FIX LATER
                        var currentDroneCounter = storage.GetGlobal("data" + player_id);
                        currentDroneCounter.arrayOfPos[chosenPosition].id = id1;
                        currentDroneCounter.arrayOfPos[chosenPosition].pos = 1;
                        currentDroneCounter.droneCounter++;
                        storage.SetGlobal("data" + player_id, currentDroneCounter);
                        //TO FIX LATER

                        SC_AI_Drone_Manager_Utilities.setFinalGlobalDroneArrayMining(id1, index, weaponDistanceType, weaponPropulsionType);
						ship.RemoveItemByTypeCount(player_id, tempDroneXML, 1);
						//var PlayerContainerID = items.GetGameObjectContainerId(player_id);
						//var obj = generator.GetItemByXmlID(tempDroneXML);
						//items.RemoveItemQuantity(PlayerContainerID, obj.item_id, 1);
                    }
                    topic.AddChoice(lastInput, "Go back!");
                    topic.AddChoice(9, " Quit the drone manager!");
                    
                    lastInput = -1;
                }
                else
                {
                    topic.AddPhrase("There are no Drones in your inventory!");
                    topic.AddChoice(lastInput, "Go back!");
                    lastInput = -1;
                }
            }
        }
    }
}