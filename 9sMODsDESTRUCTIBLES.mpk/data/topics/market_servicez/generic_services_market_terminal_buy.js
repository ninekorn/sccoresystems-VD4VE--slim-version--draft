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

function OnCheckRequirements()
{
    ni = topic.GetCurrentNpcShipId();

    if (npc.GetTag(ni, "class") == "StationTerminal")
    {
        return true;
    }
    return false;
}

function OnDialogue()
{
    topic.EnableMonospaced(true);

	var playerShip = PLAYER_SHIP;

    var input = topic.GetInput();
    var state = topic.GetState();


    na = game.GetShipOwner(playerShip);

	if (input == NO_INPUT)
    {
		cpb = ship.GetCurrentBase(playerShip);
		cnpcb = ship.GetCurrentBase(ni);
	
		topic.AddPhrase("Market Terminal ready for your input!");
		topic.AddChoice(1, "Browse Market");	
		topic.AddChoice(3, "Disconnect");
	}

	else if (input == 1)
	{
		topic.AddPhrase("Do you want to buy items on the General Market?");
		topic.AddChoice(2, "Buy!");
		topic.AddChoice(NO_INPUT, "Go Back!");
		topic.AddChoice(3, "Disconnect");
	}

    else if (input == 2)
    {
        slvl = generator.GetAllSystems();

        for (var index = 0; index < slvl.length; index++) {
            var systemId = slvl[index];
            var systemBases = game.GetSystemBases(systemId);

            if (systemBases.length <= 0) {
                slvl.splice(index, 1);
                index--;
            }
        }
        topic.AddPhrase("Which System do you want to browse Assets");

        var k;
        for (k = 0; k < slvl.length; k++) {
            var systemInfo = generator.GetSystemByID(slvl[k]);
            var names = systemInfo.name;
            topic.AddChoice(k + 4, names);
        }
        tnosvl = k;
        topic.AddChoice(NO_INPUT, "Go Back!");
        topic.AddChoice(3, "Disconnect");
    }

    else if (input == 3)
    {
        topic.DialogueBreak();
    }

	else if (input >= 4 && input < 4 + slvl.length)
    {
		var index = input - 4;
		var systemId = slvl[index];
		var currentChosenSystem = storage.SetGlobal("marketTerminalChosenSystem" + playerShip, systemId);
		player.StartDialogue(na, "marketBUYStations");
	}
}