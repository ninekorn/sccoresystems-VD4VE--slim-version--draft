using(npc);
using(console);
using(player);
using(station);
using(ship);
using(game);
using(generator);
using(storage);
using(relations);
using(actions);
using(items);

var ni;
var cpb;
var na;
var slvl;
var tnosvl;
var cnpcb;
var stationTypeXML;
var sys_idNPC;
var Aots;
var indexOfPosArray = [];
var counterOfTurrets = 0;
var chosenLocation;

var maxAmountOfTurretTOTAL = 4;
var baseTurretCost = 2500;

function OnCheckRequirements()
{
    var playerShip = PLAYER_SHIP;
  
    ni = topic.GetCurrentNpcShipId();
   
    if (npc.GetTag(ni, "class") == "StationTerminal")
    {
        var docked = ship.IsDocked(playerShip);
        if (docked) {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

function OnDialogue()
{
    //topic.EnableMonospaced(true);

	var playerShip = PLAYER_SHIP;

    var input = topic.GetInput();
    var state = topic.GetState();

    //na = game.GetShipOwner(playerShip);

	if (input == NO_INPUT)
    {
        indexOfPosArray = [];
        counterOfTurrets = 0;
        chosenLocation = -1;

		cpb = ship.GetCurrentBase(playerShip);
		//cnpcb = ship.GetCurrentBase(ni);
	
        topic.AddPhrase("Station Terminal ready for your input!");
        sys_idNPC = npc.GetCurrentSystemID(ni);
       
        Aots = storage.GetGlobal("system_" + sys_idNPC + "_base_" + cpb + "_turrets");

        stationTypeXML = storage.GetGlobal("system_" + sys_idNPC + "_base_" + cpb + "_xmlStationType");

        if (Aots != null && Aots.length > 0)
        {
            for (var i = 0; i < Aots.length; i++)
            {
                if (Aots[i].id != -1) {
                    if (generator.ShipExists(Aots[i].id))
                    {
                        counterOfTurrets++;
                    }
                    else
                    {
                        Aots[i].id = -1;
                        storage.SetGlobal("system_" + sys_idNPC + "_base_" + cpb + "_turrets", Aots);
                        indexOfPosArray.push(i);
                    }
                }
                else
                {
                    indexOfPosArray.push(i);
                }
            }

            if (counterOfTurrets >= 0 && counterOfTurrets < 4)
            {

			    var bsinf = generator.GetBaseByID(cpb);
				var costOfBuildTurret = baseTurretCost * bsinf.danger_level * bsinf.base.tech_level;
				//bsinf.danger_level
				//bsinf.base.tech_level
				var pName = game.GetShipOwner(playerShip);
				var money = player.GetMoney(pName);
				if(money < costOfBuildTurret)
				{
					topic.AddPhrase("Each turrets cost " + costOfBuildTurret + "." + " You do not have enough money. You have only " + money + "$.");
				}
				else
				{
					topic.AddChoice(1, "Build turrets. Each turrets cost " + costOfBuildTurret);
				}




                topic.AddChoice(3, "Disconnect");
            }
            else if (counterOfTurrets == 4)
            {
                topic.AddPhrase("The station cannot install more turrets...");
                topic.AddChoice(3, "Disconnect");
            }
        }
        else
        {
            //rebuild Global Array?
            //check locations around base to see if there are any existing turrets that somehow belong to the base but arent part of the global array anymore.
            //topic.AddChoice(1, "Build turrets");
            //topic.AddChoice(3, "Disconnect");
        }
	}

	else if (input == 1)
    {
        if (stationTypeXML.base_xml.indexOf("outpost") >= 0)
        {
            if (indexOfPosArray.length > 0)
            {
                for (var i = 0; i < indexOfPosArray.length; i++)
                {
                    chosenLocation = indexOfPosArray[i];
                    topic.AddChoice(2 + indexOfPosArray[i], "Build Turret at Location " + chosenLocation);	
                }
            }
        }
        else if (stationTypeXML.base_xml.indexOf("mining") >= 0)
        {
            if (indexOfPosArray.length > 0) {
                for (var i = 0; i < indexOfPosArray.length; i++) {
                    chosenLocation = indexOfPosArray[i];
                    topic.AddChoice(2 + indexOfPosArray[i], "Build Turret at Location " + chosenLocation);
                }
            }
        }
        else if (stationTypeXML.base_xml.indexOf("business") >= 0)
        {
            if (indexOfPosArray.length > 0) {
                for (var i = 0; i < indexOfPosArray.length; i++) {
                    chosenLocation = indexOfPosArray[i];
                    topic.AddChoice(2 + indexOfPosArray[i], "Build Turret at Location " + chosenLocation);
                }
            }
        }
        else if (stationTypeXML.base_xml.indexOf("military") >= 0)
        {
            if (indexOfPosArray.length > 0) {
                for (var i = 0; i < indexOfPosArray.length; i++) {
                    chosenLocation = indexOfPosArray[i];
                    topic.AddChoice(2 + indexOfPosArray[i], "Build Turret at Location " + chosenLocation);
                }
            }
        }
        else if (stationTypeXML.base_xml.indexOf("science") >= 0)
        {
            if (indexOfPosArray.length > 0) {
                for (var i = 0; i < indexOfPosArray.length; i++) {
                    chosenLocation = indexOfPosArray[i];
                    topic.AddChoice(2 + indexOfPosArray[i], "Build Turret at Location " + chosenLocation);
                }
            }
        }
	}

    else if (input >= 2 && input <= 2 + maxAmountOfTurretTOTAL)
    {
        //verify if another player already bought a turret and that it is already placed where the new player wants it... deny the build.
        //verify if there is another object located where the turret needs to be placed and deny the build if there is.
        //make players PAY for the turrets building.
        
        var currentIndexSelected = input - 2;

        if (chosenLocation != -1)
        {
            var startIndex = 0;
            var offsetX = 0;
            var offsetY = 0;

            if (stationTypeXML.base_xml.indexOf("outpost") >= 0)
            {
                startIndex = 11;
                if (currentIndexSelected == 0)
                {
                    offsetX = 30;
                    offsetY = -5;
                }
                else if (currentIndexSelected == 1)
                {
                    offsetX = 30;
                    offsetY = 5; //-18
                }
                else if (currentIndexSelected == 2)
                {
                    offsetX = -30;
                    offsetY = -5;
                }
                else if (currentIndexSelected == 3)
                {
                    offsetX = -30;
                    offsetY = 5; //-18
                }          
            }
            else if (stationTypeXML.base_xml.indexOf("mining") >= 0)
            {
                startIndex = 10;

                if (currentIndexSelected == 0)
                {
                    offsetX = 30;
                    offsetY = -5; //0
                }
                else if (currentIndexSelected == 1)
                {
                    offsetX = 30;
                    offsetY = 5;//15.75
                }
            }
            else if (stationTypeXML.base_xml.indexOf("business") >= 0)
            {
                startIndex = 12;

                if (currentIndexSelected == 0)
                {
                    offsetX = 25;
                    offsetY = -5;
                }
                else if (currentIndexSelected == 1)
                {
                    offsetX = 25;
                    offsetY = 5;//18
                }
                else if (currentIndexSelected == 2)
                {
                    offsetX = -25;
                    offsetY = -5;
                }
                else if (currentIndexSelected == 3)
                {
                    offsetX = -25;
                    offsetY = 5; //18
                }   
            }
            else if (stationTypeXML.base_xml.indexOf("military") >= 0)
            {
                startIndex = 12;
                if (currentIndexSelected == 0)
                {
                    offsetX = 30;
                    offsetY = -5; //0
                }
                else if (currentIndexSelected == 1)
                {
                    offsetX = 30;
                    offsetY = 5 ;//15.75
                }
            }
            else if (stationTypeXML.base_xml.indexOf("science") >= 0)
            {
                startIndex = 11;

                if (currentIndexSelected == 0)
                {
                    offsetX = 20;
                    offsetY = -5; // 0
                }
                else if (currentIndexSelected == 1)
                {
                    offsetX = 20;
                    offsetY = 5; //15.75
                }
            }

            var getSomeIndex = stationTypeXML.base_xml.substring(startIndex, stationTypeXML.length); //outpost_01_0
            var parsedAngle = parseInt(getSomeIndex);

            var bsinf = generator.GetBaseByID(cpb);
            var coords = game.GetObjectCoordinates(sys_idNPC, bsinf.id);

            var pointToRotateX = coords.x + offsetX;
            var pointToRotateY = coords.y + offsetY;
            var pointToRotate = { x: pointToRotateX, y: pointToRotateY };
            var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            //level of player or MAX level of system?...
            //bsinf.base.tech_level;
            //console.PrintError(bsinf.faction);
			//bsinf.danger_level
			//bsinf.base.tech_level
            //inf.danger_level
            var level = bsinf.tech_level + Math.round(MathExt.RandDouble() * 4 - 2);
            level = utils.Clamp(level, 1, 100);

            var id0 = generator.AddNPCShipToSystem(
                "", "Turret", level, "special_human_turret",
                sys_idNPC,
                turret_coord.x,
                turret_coord.y,
                { class: "turret", non_talkable: true });

            ship.SetRotation(id0, 0);
            relations.SetShipFaction(id0, bsinf.faction);

            Aots = storage.GetGlobal("system_" + sys_idNPC + "_base_" + cpb + "_turrets", AOTAS);
            Aots[currentIndexSelected].id = id0;
            storage.SetGlobal("system_" + sys_idNPC + "_base_" + cpb + "_turrets", Aots);




        }
        else
        {
            //?
        }
    }
    else if (input == 3) {
        topic.DialogueBreak();
    }
}


//https://stackoverflow.com/questions/13695317/rotate-a-point-around-another-point
function RotatePoint(pointToRotate, centerPoint, angleInDegrees) {
    var angleInRadians = angleInDegrees * (Math.PI / 180);
    var cosTheta = Math.cos(angleInRadians);
    var sinTheta = Math.sin(angleInRadians);

    var newX = (cosTheta * (pointToRotate.x - centerPoint.x) - sinTheta * (pointToRotate.y - centerPoint.y) + centerPoint.x);
    var newY = (sinTheta * (pointToRotate.x - centerPoint.x) + cosTheta * (pointToRotate.y - centerPoint.y) + centerPoint.y);

    var newPos = { x: newX, y: newY };

    return newPos;
}
