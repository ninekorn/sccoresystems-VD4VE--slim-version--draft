//------------------------------------------------------
// This script governs the behavior of npc ships
//
//

using(console);
using(game);
using(npc);
using(generator);
using(items);
using(relations);
using(station);

function OnAIInited()
{
    //set update interval
    npc.SetDecisionsPerSecond(SHIP_ID, 20);
}


//-----------------------------------------------------
// Name: Decision
// Desc: 
//-----------------------------------------------------
/* args:
 scope_ships:
 disposition
 id
 is_npc
 name
 distance
 */




var _frameCounter = 0;
var _initiliazeOnce = true;
var _systemsList;

function Decision(args)
{
    //=====================================================================//
    //============================GALAXY MARKET============================//
    //=====================================================================//
    if (_initiliazeOnce)
    {
        _systemsList = generator.GetAllSystems();

        for (var index = 0; index < _systemsList.length; index++)
        {
            var systemId = _systemsList[index];
            var systemBases = game.GetSystemBases(systemId);

            if (systemBases.length <= 0)
            {
                _systemsList.splice(index, 1);
                index--;
            }
        }
        _initiliazeOnce = false;
    }

    if (_frameCounter > 2)
    {
        if (_systemsList.length > 0)
        {
            //systemBasesVirtualLocker = game.GetSystemBases(systemId);

            for (var index = 0; index < _systemsList.length; index++)
            {
                var _systemBasesTotal = game.GetSystemBases(_systemsList[index]);

                for (var j = 0; j < _systemBasesTotal.length; j++)
                {
                    //console.PrintError(_systemBasesTotal[i] + " item generation completed");

                    var base = generator.GetBaseByID(_systemBasesTotal[j]);
                    var faction = relations.GetBaseFaction(_systemBasesTotal[j]);

                    // get all items sorted by types
                    var types = game.GetItemTypes();
                    var allItems = {};
                    for (var i = 0; i < types.length; i++) {
                        allItems[types[i]] = game.GetItems(base.tech_level, types[i], faction);
                    }

                    // get all civilian items
                    var civilianItems = game.GetCivilianItems();
                    for (var i = 0; i < civilianItems.length; i++) {
                        station.AddItem(_systemBasesTotal[j], civilianItems[i]);
                    }

                    // get all item types marked with "spawnSeparately" tag 
                    // - this is required to ensure that the item will be spawned without any randomization
                    var spawnRequiredItems = game.GetSpawnSeparatelyItems(base.tech_level, faction);
                    for (var i = 0; i < spawnRequiredItems.length; i++) {
                        station.AddItem(_systemBasesTotal[j], spawnRequiredItems[i]);
                    }

                    // generate quantities and types for base
                    var dict = {};
                    var distribution = station.GenerateStockDistribution(_systemBasesTotal[j]);
                    for (var type in distribution)
                    {
                        // please note: type can be hull/engine/consumable/etc
                        var itemsOfType = allItems[type];
                        if (itemsOfType.length != 0)
                        {
                            var amount = distribution[type];

                            //now we have type and amount - it's time to generate
                            for (var i = 0; i < amount; i++) {
                                // take random itemType
                                var itemType = utils.SelectRandom(itemsOfType);

                                if (dict[itemType] != null) {
                                    dict[itemType]++;
                                }
                                else {
                                    dict[itemType] = 1;
                                }
                            }
                        }                   
                    }

                    for (var itemType in dict)
                    {
                        var quantity = dict[itemType];
                        station.AddItem(_systemBasesTotal[j], itemType, quantity);
                    }
                    //console.PrintError(_systemBasesTotal[j] + " item generation completed");
                }
                _systemsList.splice(index, 1);
                index--;
                break;
            }
        }
        else
        {
            //console.PrintError("total item generation completed");
            generator.RemoveShip(SHIP_ID);
        }
        _frameCounter = 0;
    }













    _frameCounter++;
    //=====================================================================//
    //============================GALAXY MARKET============================//
    //=====================================================================//
}