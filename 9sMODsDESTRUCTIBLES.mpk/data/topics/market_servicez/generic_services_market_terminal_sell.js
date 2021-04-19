using(npc);
using(console);
using(player);
using(station);
using(ship);
using(game);
using(generator);
using(storage);
using(relations);
using(items);

var slvl;
var sbvl;
var iiascopvl;
var tnosvl = 0;
var na;

function OnCheckRequirements()
{
    var npc_id = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npc_id, "class") == "StationTerminal")
    {
        return true;
    }
    return false;
}

function OnDialogue() {
    var playerShip = PLAYER_SHIP;

    var input = topic.GetInput();
    var state = topic.GetState();
    na = game.GetShipOwner(playerShip);

    //Enable monospaced font mode, since it is a terminal
    topic.EnableMonospaced(true);

    if (input == NO_INPUT)
    {
        topic.AddPhrase("Market Terminal Ready for your Input");
        topic.AddChoice(1, "Browse Market");
        topic.AddChoice(3, "Disconnect");
    }
 
    else if (input == 1) {

        topic.AddPhrase("Do you want to sell items on the General Market?");
        topic.AddChoice(2, "Sell!");
        topic.AddChoice(NO_INPUT, "Go Back!");
        topic.AddChoice(3, "Disconnect");
    }

    else if (input == 2) {

        slvl = generator.GetAllSystems();

        for (var index = 0; index < slvl.length; index++) {
            var systemId = slvl[index];
            var systemBases = game.GetSystemBases(systemId);

            if (systemBases.length <= 0) {
                slvl.splice(index, 1);
                index--;
            }
        }

        for (var index = 0; index < slvl.length; index++) {
            var systemId = slvl[index];
            //var systemBases = game.GetSystemBases(systemId);
            sbvl = game.GetSystemBases(systemId);

            for (var indexer = 0; indexer < sbvl.length; indexer++) {
                var stationStorageVirtualLocker = items.GetShipStationStorageContainerId(playerShip, sbvl[indexer]);
                //var stationStorageVirtualLocker = items.GetStationStorageContainerId(sbvl[indexer],playerShip);
                iiascopvl = items.GetItemsAndCargo(stationStorageVirtualLocker);

                if (iiascopvl.length <= 0) {
                    sbvl.splice(indexer, 1);//remove the station from the StationList if there is no items belonging to the player in the station.
                    indexer--;
                }
            }
            if (sbvl.length <= 0) {
                slvl.splice(index, 1);
                index--;
            }
        }

        if (slvl.length <= 0) {
            topic.AddPhrase("You have no Assets!");
        }
        else {
            topic.AddPhrase("Which System do you want to browse your Assets");

            var k;
            for (k = 0; k < slvl.length; k++) {
                var systemInfo = generator.GetSystemByID(slvl[k]);
                var names = systemInfo.name;

                topic.AddChoice(k + 4, names);
            }
            tnosvl = k;

        }
        topic.AddChoice(NO_INPUT, "Go Back!");
        topic.AddChoice(3, "Disconnect");
    }
    else if (input == 3) {
        topic.DialogueBreak();
    }

    else if (input >= 4 && input < 4 + slvl.length) {

        var index = input - 4;
        var systemId = slvl[index];
        var currentChosenSystem = storage.SetGlobal("marketTerminalChosenSystem" + playerShip, systemId);
        player.StartDialogue(na, "marketSELLStations");
    }
}
