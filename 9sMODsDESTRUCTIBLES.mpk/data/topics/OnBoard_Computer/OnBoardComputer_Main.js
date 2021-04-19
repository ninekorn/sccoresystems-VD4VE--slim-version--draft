using(relations);
using(npc);
using(console);
using(ship);
using(generator);
using(game);
using(items);
using(player);
using(storage);
using(server);

include(SC_Utilities.js);

var state;
var input;
var quest_drone_id = 0;
var quest_system_id = 0;
var quest_base_id = 0;
var amIInDialogue = true;

var listOfDrones = [];

var arrayOfStringPos = [];

var frontR = "front right";
var frontL = "front left";
var backR = "back right";
var backL = "back left";
var middleB = "middle back";

var chosenDrone = "";

var arrayOfDrones;
var stringOfAlpha = "";
var listOfPlayers = [];

var arrayOfLivingDrones;
var playerListOfFriendlies;
var playerListOfFriendliesers = [];

function OnCheckRequirements()
{
    var npc_id = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npc_id, "class") == "OnBoardComputer")
    {
        return true;
    }
    return false;
}

function OnDialogue(args)
{
    var npc_id = topic.GetCurrentNpcShipId();
    var player_id = PLAYER_SHIP;

    //var player_id = PLAYER_SHIP;
    //var shipID = args.ship_id;

    var name = game.GetShipOwner(player_id);

    var coords = ship.GetCoordinates(player_id);
    var sys_id = ship.GetSystemID(player_id);
    input = topic.GetInput();
    state = topic.GetState();

    var isPlayerShipDocked = ship.IsDocked(player_id);  

    if (state == 0)
    {
        if (!isPlayerShipDocked)
        {
            if (input == NO_INPUT)
            {
                arrayOfDrones = [];
                arrayOfStringPos = [];
                listOfPlayers =  [];
                arrayOfLivingDrones = null;
				playerListOfFriendliesers = [];

                arrayOfStringPos.push(frontR);
                arrayOfStringPos.push(frontL);
                arrayOfStringPos.push(backR);
                arrayOfStringPos.push(backL);
                arrayOfStringPos.push(middleB);

				var currentDroneCounter = storage.GetGlobal("data" + player_id);

                if (currentDroneCounter == null || currentDroneCounter.droneCounter == null || currentDroneCounter.init != 1 || currentDroneCounter.arrayOfPos[0].pos == null || currentDroneCounter.arrayOfPos[0].id == null) //|| isNaN(currentDroneCounter)
                {
                    //resetGlobalArray(player_id);   
                    storage.Remove("data" + player_id);
                    var arrayOfFormations = [];

                    for (var i = 0; i < 6; i++) {
                        var test = { pos: 0, id: -1, formation: -1 };
                        arrayOfFormations.push(test);
                    }
                    //console.PrintError("init");

                    var currentDroneCounter3 = { init: 1, droneCounter: 0, arrayOfPos: arrayOfFormations };

                    storage.SetGlobal("data" + player_id, currentDroneCounter3);
                }

                topic.AddPhrase("OnBoard Computer");
                //topic.AddChoice(100, "Drone Commands");

                var playerContainerID = items.GetGameObjectContainerId(player_id);
                var itemsAndCargo = items.GetItemsAndCargo(playerContainerID);

				var counterMining = 0;
				var counterRepair = 0;
				var counterCombat = 0;

                /*for (var i = 0; i < itemsAndCargo.length; i++)
				{
                    if (itemsAndCargo[i].xml_id.substring(0, 12) == "drone_combat") 
					{
                        counterCombat++;
                    }
                    else if (itemsAndCargo[i].xml_id.substring(0, 12) == "drone_mining") 
					{
						counterMining++;
                    }
                    else if (itemsAndCargo[i].xml_id.substring(0, 12) == "drone_repair")
                    {
                        counterRepair++;
                    }
                }
				
                if (counterCombat > 0) 
				{
                    topic.AddChoice(120, " Select combat drones");
                }
                if (counterMining > 0) 
				{
                    topic.AddChoice(121, " Select mining drones");
                }
                if (counterRepair > 0)
                {
                    topic.AddChoice(122, " Select repair drones");
                }*/
				topic.AddChoice(120, " Select combat drones");
				topic.AddChoice(121, " Select mining drones");
				topic.AddChoice(122, " Select repair drones");


                topic.AddChoice(117, " Browse Market");
                topic.AddChoice(999, "Add Player to Friendlies");
                topic.AddChoice(9999, "Remove Player from Friendlies");
                //topic.AddChoice(3000, "Destroy all Drones");
            }
            else if (input == 9)
            {
                topic.DialogueBreak();
            }

            else if (input == 999)
            {
                topic.AddPhrase("Choose a Player!");
                //Add player with player list
                //type player name and search in main database?
                //SC_Utilities.isAlphaOnly();
                //var obj = server.IsPlayerExists("dlirry");
                listOfPlayers = server.GetAllPlayers();

                if (listOfPlayers.length > 0)
                {
                    var currentPlayerName = game.GetShipOwner(player_id);
					var friendliesLister = storage.GetGlobal("FriendliesList" + player_id);

                    for (var i = 0; i < listOfPlayers.length; i++)
                    {
                        if (currentPlayerName != listOfPlayers[i] && !SC_Utilities.contains(friendliesLister,listOfPlayers[i].toLowerCase()))
                        {
                            topic.AddChoice(1000 + i, "Add " + listOfPlayers[i].toLowerCase() + " to your list of Friendly Players?!");
                        }
                        else if (currentPlayerName == listOfPlayers[i])
                        {
                            topic.AddPhrase("You cannot add " + listOfPlayers[i].toLowerCase() + " as this player is yourself!"); //
                        }
						else
						{
							topic.AddPhrase("You cannot add " + listOfPlayers[i].toLowerCase() + " as this player is already added.");
						}
                    }
                }
                else
                {
                    topic.AddPhrase("There are no Players in this server, except you!"); // its never going to go here since if there is a player, the length is 1
                }  
            }

            else if (input >= 1000 && input <= 1000 + listOfPlayers.length - 1)
            {
                var currentSelectedPlayerIndex = input - 1000; 
                var currentSelectedPlayer = listOfPlayers[currentSelectedPlayerIndex].toLowerCase();

                var friendlyToAdd = storage.SetGlobal("FriendlyToAdd" + player_id, currentSelectedPlayer);

                var currentDroneCounter = storage.GetGlobal("data" + player_id);

                var droneCounter = 0;
                arrayOfLivingDrones = currentDroneCounter.arrayOfPos;

                for (var i = 0; i < arrayOfLivingDrones.length; i++)
                {
                    if (arrayOfLivingDrones[i].id != -1)
                    {
                        var shipExists = generator.ShipExists(arrayOfLivingDrones[i].id);

                        if (shipExists && ship.GetCurrentValue(arrayOfLivingDrones[i].id, "structure") > 0)
                        {
                            if ("" + player_id == npc.GetTag(arrayOfLivingDrones[i].id, "ownerPlayerShipId"))
                            {
                                arrayOfLivingDrones[i].id = arrayOfLivingDrones[i].id;
                                arrayOfLivingDrones[i].pos = 1;
                                droneCounter++;
                            }
                            else {
                                arrayOfLivingDrones[i].pos = 0;
                                arrayOfLivingDrones[i].id = -1;
                            }
                        }
                        else {
                            arrayOfLivingDrones[i].pos = 0;
                            arrayOfLivingDrones[i].id = -1;
                        }
                    }
                    else {
                        arrayOfLivingDrones[i].pos = 0;
                        arrayOfLivingDrones[i].id = -1;
                    }
                }
                //currentDroneCounter.droneCounter = droneCounter;
                //currentDroneCounter.arrayOfPos = null;
                //currentDroneCounter.arrayOfPos = arrayOfPoser;
                //storage.SetGlobal("data" + player_id, currentDroneCounter);
                //return currentDroneCounter;

                for (var i = 0; i < arrayOfLivingDrones.length; i++)
                {
                    if (arrayOfLivingDrones[i].id != -1)
                    {
                        //topic.AddChoice(950 + i, "Add " + listOfPlayers[i] + " to this drone Friendly list?!");

                        var currentSelectedDrone = arrayOfLivingDrones[i].id;
                        var droneData = storage.GetGlobal("currentCommand" + currentSelectedDrone);

                        droneData.addFriend = 1;

                        storage.SetGlobal("currentCommand" + currentSelectedDrone, droneData);                     
                    }
                }

				if (storage.IsSetGlobal("FriendliesList" + player_id))
                {
                    var playerListOfFriendlies = storage.GetGlobal("FriendliesList" + player_id);
                    var friendlyToAdd = storage.GetGlobal("FriendlyToAdd" + player_id);

                    playerListOfFriendlies.push(friendlyToAdd);

                    storage.SetGlobal("FriendliesList" + player_id, playerListOfFriendlies);
                }
                else {
                    var friendlyToAdd = storage.GetGlobal("FriendlyToAdd" + player_id);
                    var playerListOfFriendlies = [];
                    playerListOfFriendlies.push(friendlyToAdd);
                    storage.SetGlobal("FriendliesList" + player_id, playerListOfFriendlies);
                }
                //storage.SetGlobal("currentCommand" + id1, { id: id1, command: 0, formation: chosenPosition, lastCommand: 0, wepPropType: weaponPropulsionType, wepDistType: weaponDistanceType, droneIndex: index, addFriend: 0 }); //, behavior: behavior
            }

            /*else if (input >= 950 && input <= 950 + listOfPlayers.length - 1) // drones length is 5 max
            {
                var currentSelectedDroneIndex = input - 950;
                var currentSelectedDrone = arrayOfLivingDrones[currentSelectedDroneIndex];
                var droneData = storage.GetGlobal("currentCommand" + currentSelectedDrone.id); 

                droneData.addFriend = 1;

                storage.SetGlobal("currentCommand" + currentSelectedDrone.id, droneData); 

                if (storage.IsSetGlobal("FriendliesList" + player_id))
                {
                    var playerListOfFriendlies = storage.GetGlobal("FriendliesList" + player_id);
                    var friendlyToAdd = storage.GetGlobal("FriendlyToAdd" + player_id);

                    playerListOfFriendlies.push(friendlyToAdd);

                    storage.SetGlobal("FriendliesList" + player_id, playerListOfFriendlies);
                }
                else
                {
                    var friendlyToAdd = storage.GetGlobal("FriendlyToAdd" + player_id);
                    var playerListOfFriendlies = [];
                    playerListOfFriendlies.push(friendlyToAdd);
                    storage.SetGlobal("FriendliesList" + player_id, playerListOfFriendlies);
                }
            }*/
            //SETUP THE REMOVE FROM FRIENDLIES COMMAND.

            else if (input == 9999)
            {
                playerListOfFriendliesers = storage.GetGlobal("FriendliesList" + player_id);

                if (playerListOfFriendliesers.length > 0)
                {  
					var currentPlayerName = game.GetShipOwner(player_id);
                    for (var i = 0; i < playerListOfFriendliesers.length; i++)
                    {                      
                        if (currentPlayerName != playerListOfFriendliesers[i])
                        {
                            topic.AddChoice(10000 + i, "Player " + playerListOfFriendliesers[i].toLowerCase() + " is in your list of Friendlies! Do you want to remove this Player?!");
                        }
                        else
                        {
                            topic.AddPhrase("You cannot remove " + playerListOfFriendliesers[i].toLowerCase() + " from the list of friendlies!"); //
                        }                      
                    }
                }
                else
                {
                    topic.AddPhrase("You have no friendlies");
                }
            }

            else if (input >= 10000 && input <= 10000 + playerListOfFriendliesers.length - 1)
            {          
                var currentSelectedPlayerIndex = input - 10000; 
                var tempPlayerList = storage.GetGlobal("FriendliesList" + player_id);
                tempPlayerList.splice(currentSelectedPlayerIndex,1);
                storage.SetGlobal("FriendliesList" + player_id, tempPlayerList);

                var currentDroneCounter = storage.GetGlobal("data" + player_id);

                var droneCounter = 0;
                arrayOfLivingDrones = currentDroneCounter.arrayOfPos;

                for (var i = 0; i < arrayOfLivingDrones.length; i++) {
                    if (arrayOfLivingDrones[i].id != -1) {
                        var shipExists = generator.ShipExists(arrayOfLivingDrones[i].id);

                        if (shipExists && ship.GetCurrentValue(arrayOfLivingDrones[i].id, "structure") > 0) {
                            if ("" + player_id == npc.GetTag(arrayOfLivingDrones[i].id, "ownerPlayerShipId")) {
                                arrayOfLivingDrones[i].id = arrayOfLivingDrones[i].id;
                                arrayOfLivingDrones[i].pos = 1;
                                droneCounter++;
                            }
                            else {
                                arrayOfLivingDrones[i].pos = 0;
                                arrayOfLivingDrones[i].id = -1;
                            }
                        }
                        else {
                            arrayOfLivingDrones[i].pos = 0;
                            arrayOfLivingDrones[i].id = -1;
                        }
                    }
                    else {
                        arrayOfLivingDrones[i].pos = 0;
                        arrayOfLivingDrones[i].id = -1;
                    }
                }

                for (var i = 0; i < arrayOfLivingDrones.length; i++)
                {
                    if (arrayOfLivingDrones[i].id != -1)
                    {
                        //topic.AddChoice(950 + i, "Add " + listOfPlayers[i] + " to this drone Friendly list?!");

                        var currentSelectedDrone = arrayOfLivingDrones[i].id;
                        var droneData = storage.GetGlobal("currentCommand" + currentSelectedDrone);

                        droneData.addFriend = 3; // to remove friend

                        storage.SetGlobal("currentCommand" + currentSelectedDrone, droneData);

                        var someData = { index: currentSelectedPlayerIndex, pName: playerListOfFriendliesers[currentSelectedPlayerIndex]};

                        storage.SetGlobal("FriendlyToAdd" + player_id, someData);
                    }
                }
            }
           
            ////////////////////////////////////////////////////////DRONE RELEASING COMMANDS///////////////////////////////////////////////////

            //------------MARKET MAIN--------------------
            else if (input == 117)
            {
                var systemBases = game.GetSystemBases(sys_id);

                if (systemBases.length > 0)
                {
                    for (var i = 0; i < systemBases.length;i++)
                    {
                        player.StartDialogue(name, "market" + systemBases[i]);
                        break;
                    }
                }
                else
                {
                    var allSystems = generator.GetAllSystems();
                    var chosenStation = null;

                    for (var i = 0; i < allSystems.length; i++)
                    {
                        var stations = generator.GetBaseByID(allSystems[i]);

                        if (stations.length > 0)
                        {
                            chosenStation = stations[0];
                        }
                    }
                    player.StartDialogue(name, "market" + chosenStation);
                }             
            }
            //--------------------------------------------

            //------------COMBAT DRONES--------------------
            else if (input == 120)
            {
                player.StartDialogue(name, "droneManagerCombat");
            }
            //--------------------------------------------

            //------------MINING DRONES--------------------
            else if (input == 121)
            {
                player.StartDialogue(name, "droneManagerMining");
            }
            //--------------------------------------------

            //------------HEALER DRONES--------------------
            else if (input == 122)
            {
                player.StartDialogue(name, "droneManagerRepair");
            }
            //--------------------------------------------
 
            /*else if (input == 3000) {
                //var ships = game.GetSystemShips(sys_id);

                //var ships = ship.GetShipsInScope(player_id);
                for (var i = 0; i < listOfDrones.length; i++) {
                    generator.RemoveShip(listOfDrones[i]);
                }
                listOfDrones = [];
                resetGlobalArray(player_id);
                topic.SetState(0);
                topic.DialogueBreak();
            }*/
        }
    }
}