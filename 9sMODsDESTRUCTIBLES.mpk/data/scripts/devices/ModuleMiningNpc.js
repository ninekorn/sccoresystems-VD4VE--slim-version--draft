
/*
 Script can contain following functions:
 OnUpdateCache - called when device is equipped or initialized, or some values are changed
 OnStart - called when device is activated (for all)
 OnFrame - called every frame (only for "per frame" mode)
 OnFinished - called when device effect should be applied (only for "per frame" and "on complete")
 */

using(ship);
using(console);
using(generator);
using(game);
using(visual);
using(actions);
using(items);
using(npc);
using(storage);



function OnUpdateCache(args)
{
    //console.Print("Mining module started");
    //(ship's cache)-based updates
    var mining_range_modifier = ship.GetFinalCacheValue(args.ship_id, "mining_range");
    args.range = args.range * mining_range_modifier;

    var mining_amount_modifier = ship.GetFinalCacheValue(args.ship_id, "mining_amount");
    args.custom_parameters.amount = args.custom_parameters.amount * mining_amount_modifier;

    var mining_speed_modifier = ship.GetFinalCacheValue(args.ship_id, "mining_speed");
    args.custom_parameters.speed = args.custom_parameters.speed * mining_speed_modifier;

    // console.Print("Mining range modifier current: " + mining_range_modifier);
    // console.Print("Mining amount modifier current: " + mining_amount_modifier);
    // console.Print("Mining speed modifier current: " + mining_speed_modifier);

    // console.Print("range now is " + args.range);
    // console.Print("amount now is " + args.custom_parameters.amount);
    // console.Print("speed now is " + args.custom_parameters.speed);
}

function OnStart(args) {
    //console.Print("Mining module started");

    //calculate max_length property value based on speed and asteroid's difficulty
    var asteroid = game.GetAsteroidDescriptionByID(args.target_id);
    args.duration = asteroid.difficulty / args.custom_parameters.speed;
    args.cooldown = args.duration;

    visual.DeviceActivateEffectOnObject(args.ship_id, args.device_id, "mining_visual_effect", args.duration, args.target_id);

    // console.Print("max_length now is " + args.duration);
    game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + args.target_id, "special/mining_start.ogg", 0, false);
    game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id, "special/mining_process.ogg", 0.1, true);
}

var lastAngle;
var lastPos;
var cancelDeposits = 0;

function OnFrame(args) {
    //console.Print("Mining module frame. Energy deduction");
    //console.Print("Mining module started");
    var energy = args.custom_parameters.energy * args.seconds_multiplier;
    var current = ship.GetCurrentValue(args.ship_id, "energy");

   

    /*if (radToDeg < 0) {
        radToDeg = 180 + (radToDeg * -1);
    }*/
    //console.PrintError(radToDeg);

    //args.ship_id
    var globalVar = storage.GetGlobal("currentCommand" + args.ship_id);

    if (globalVar != null) {
        var angle = ship.GetRotation(args.ship_id);
        var radToDeg = Math.floor(angle * (180.0 / Math.PI));
        var coordsNPC = ship.GetCoordinates(args.ship_id);

        if (globalVar.lastAngle != null) {
            if (globalVar.lastAngle != radToDeg) {
                game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                npc.DeviceDeactivate(args.ship_id, "device_mining_civilian_NPC");
                visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "mining_visual_effect");
                cancelDeposits = 1;
              
            }
        }
        globalVar.lastAngle = radToDeg;
        storage.SetGlobal("currentCommand" + args.ship_id, globalVar);

        if (globalVar.lastMiningPos!= null) {
            if (globalVar.lastMiningPos.x != null && globalVar.lastMiningPos.y != null) {
                if (globalVar.lastMiningPos.x != coordsNPC.x && globalVar.lastMiningPos.y != coordsNPC.y) {
                    game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                    npc.DeviceDeactivate(args.ship_id, "device_mining_civilian_NPC");
                    visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "mining_visual_effect");
                    cancelDeposits = 1;
                }
            }
        }
        
        globalVar.lastMiningPos = coordsNPC;
        storage.SetGlobal("currentCommand" + args.ship_id, globalVar);






        /*if (lastPos != null)
        {
            if (lastPos.x != null && lastPos.y != null)
            {
                if (coordsNPC.x != lastPos.x && coordsNPC.y != lastPos.y)
                {
                    game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                    npc.DeviceDeactivate(args.ship_id, "device_mining_civilian_NPC");
                    visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "mining_visual_effect");
                    cancelDeposits = 1;
                }
            }
        }

        if (lastAngle != null)
        {
            if (radToDeg != lastAngle)
            {
                game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                npc.DeviceDeactivate(args.ship_id, "device_mining_civilian_NPC");
                visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "mining_visual_effect");
                cancelDeposits = 1;
            }
        }
        lastPos = coordsNPC;
        lastAngle = radToDeg;*/

    }

   




  






    /*

    if (lastAngle != null)
    {
        if (radToDeg != lastAngle)
        {
            game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);

            npc.DeviceDeactivate(args.ship_id, "device_mining_civilian_NPC");
            visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "mining_visual_effect");
            cancelDeposits = 1;
        }
    }*/

    //lastPos = coordsNPC;
    //lastAngle = radToDeg;

    /*if (current > energy)
    {
        ship.SetCurrentValue(args.ship_id, "energy", current - energy);
    }
    else
    {
        ship.DeviceDeactivate(args.ship_id, args.slot_id, true);
        game.SendNotificationError(game.GetShipOwner(args.ship_id), $n0001, $n0002); // Mining failed: Energy depleted.
    }*/
}

function OnFinished(args)
{
    //lastAngle = null;
    //lastPos = null;
    var globalVar = storage.GetGlobal("currentCommand" + args.ship_id);
    globalVar.lastMiningPos = null;
    globalVar.lastAngle = null;
    storage.SetGlobal("currentCommand" + args.ship_id, globalVar);


    if (cancelDeposits == 0)
    {
        //console.PrintError("mining end0");
        visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "mining_visual_effect");
        //console.Print("Mining module started");
        // console.Print("Mining module finished");
        actions.InvokeTrigger("onAsteroidMined", { asteroid_id: args.target_id });

        //parse asteroids contents, add whatever we want
        var shipId = args.ship_id;
        var asteroidId = args.target_id;
        var cont = game.GetAsteroidMiningListByID(asteroidId);
        var amount = args.custom_parameters.amount;
        var Q = 2.0; //dependency of mining amount and extraction difficulty
        var total_amount;
        var extraction_diff;
        var extracted;
        var items_money_added = 0;
        var something_extracted = 0;
        var cargo_full = false;
        var bSomethingLeft = false;

        // console.Print("Mining module finished with amount "+amount);


        /*var npcContainerID = items.GetGameObjectContainerId(shipId);
        items.AddItem(npcContainerID, cont[i].name, extracted);
        console.Print(extracted);
    
        if (something_extracted < 1 && extracted < 1) {
            generator.RemoveAsteroid(asteroidId);
        }
    
        var player_id = npc.GetTag(args.ship_id, "ownerPlayerShipId");
        var name = game.GetShipOwner(player_id);
        */

        //var player_id = npc.GetTag(args.ship_id, "ownerPlayerShipId");
        //var name = game.GetShipOwner(player_id);

        for (var i = 0; i < cont.length; i++) {
            //console.PrintError("mining end1");
            //formula of mining extraction

            total_amount = Math.max(0, Math.floor(Math.pow(Math.max(amount - 50, 0), 0.25) * cont[i].extraction) - 2);
            //console.PrintError("mining end2");
            if (total_amount == 0 && cont[i].quantity > 0) {
                bSomethingLeft = true;
                continue;
            }
            //console.PrintError("mining end3");

            // console.Print("Extracting " + cont[i].name + " : _diff" + extraction_diff + " total: " + total_amount);
            extracted = game.TryToRemoveContentFromAsteroid(asteroidId, cont[i].name, total_amount);
            //console.PrintError(extracted);
            if (extracted > 0) {
                something_extracted++;
                //console.PrintError(extracted);

                //var res = ship.AddItem(shipId, cont[i].name, extracted);
                var npcContainerID = items.GetGameObjectContainerId(shipId);
                var res = items.AddItem(npcContainerID, cont[i].name, extracted);

                /*var isNpcInventoryFull = AICheckStateOfInventory(args);
    
                if (isNpcInventoryFull == 1)
                {
                    //console.PrintError("NPC CARGO !FULL");
                }
                else
                {
                    //console.PrintError("NPC CARGO IS FULL");
    
                }*/
                /*if (res != null) {
                    items_money_added += game.GetItemPrice(cont[i].name) * res.quantity;
    
                    //if item is resource, but wasn't added fully, then assume cargo is full
                    if (res.type == "resource" && res.quantity < extracted) {
                        cargo_full = true;
                    }
                }
                else {
                    //if nothing was added, check if resource - nad if yes, cargo is full
                    if (game.IsResource(cont[i].name)) {
                        cargo_full = true;
                    }
                }*/
            }
        }

        /*// give some experience
        if (items_money_added > 0) {
            var mining_exp_mult = 1 + ship.GetFinalCacheValue(args.ship_id, "mining_experience");
            ship.SetCurrentValueDelta(shipId, "experience", (mining_exp_mult * items_money_added) / 7);
            // console.Print("Mining experience multiplier: " + mining_exp_mult);
        }*/

        if (something_extracted <= 0) {
            if (!bSomethingLeft) {
                generator.RemoveAsteroid(asteroidId);
                //game.SendNotificationError(game.GetShipOwner(shipId), $n0003, $n0004); // Asteroid is depleted.

            }
            else {
                //game.SendNotificationError(game.GetShipOwner(shipId), $n0005, $n0006); // Cannot extract.: To extract certain minerals, you need better mining equipment.
            }
        }

        if (cargo_full) {
            //console.Print("Mining module started");
            //var player = game.GetShipOwner(shipId);

            var player_id = npc.GetTag(args.ship_id, "ownerPlayerShipId");
            var player = game.GetShipOwner(player_id);

            game.SendNotificationError(player, $n0007, $n0008); // Your cargo hold is full.
            game.PlaySound(player, "items/generic_on_empty.ogg");
            game.PlayVoice(player, "cargo_hold_is_full");
        }

        game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
        game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + args.target_id, "special/mining_complete.ogg", 0, false);

        /*if (ship.GetFinalCacheValue(shipId, "see_asteroid_contents") > 0)
        {
            // console.Print("Sending asteroid contents");
            //ship.SendAsteroidContents(shipId, asteroidId);
        }*/

    }
    else
    {
        visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "mining_visual_effect");
        game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
        game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + args.target_id, "special/mining_complete.ogg", 0, false);
    }
    cancelDeposits = 0;
    //game.AddStat(shipId, "asteroids_mined", 1);
}

function OnCancel(args) {

    //console.PrintError("Mining module canceled of " + args.cancel_reason);
    //console.Print("Mining module started");
    if (args.cancel_reason.indexOf("took_damage") >= 0)
    {
        game.SendNotificationError(game.GetShipOwner(args.ship_id), $n0009, $n0010); // Mining failed: You cannot mine if you are being attacked.
    }
    else if (args.cancel_reason.indexOf("move") >= 0)
    {
        game.SendNotificationError(game.GetShipOwner(args.ship_id), $n0011, $n0012); // Mining failed: You must not move during the mining process.
    }
    // else if (args.cancel_reason.indexOf("canceled") >= 0)
    // {
    // game.SendNotificationError(game.GetShipOwner(args.ship_id), $n0013, $n0014); // Mining canceled: You've canceled the mining process.
    // }

    visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "mining_visual_effect");

    game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
}


function AICheckStateOfInventory(args) {
    var shipId = args.ship_id;
    var NPCContainerID = items.GetGameObjectContainerId(shipId);
    var itemsInCargoOfNPC = items.GetItemsAndCargo(NPCContainerID);

    for (var i = 0; i < 1; i++) {

        var totalSizeOfNPCStorage = 0;
        var emptySpaceToRemoveNPC = 0;
        var initialQuantityOfItemsInNpcStorage = 0;

        if (itemsInCargoOfNPC != null) {
            if (itemsInCargoOfNPC.length > 0) {
                if (itemsInCargoOfNPC[0] != null) {
                    for (var i = 0; i < itemsInCargoOfNPC.length; i++) {
                        initialQuantityOfItemsInNpcStorage += itemsInCargoOfNPC[i].quantity;
                    }
                }
                else {
                    initialQuantityOfItemsInNpcStorage = 0;
                }
            }
            else {
                initialQuantityOfItemsInNpcStorage = 0;
            }
        }
        else {
            initialQuantityOfItemsInNpcStorage = 0;
        }

        items.AddItem(NPCContainerID, "goods_emptyspace", 10000);
        var itemsInCargoOfNPCAfterAddingEmptySpace = items.GetItemsAndCargo(NPCContainerID);
        if (itemsInCargoOfNPCAfterAddingEmptySpace != null) {
            if (itemsInCargoOfNPCAfterAddingEmptySpace.length > 0) {
                if (itemsInCargoOfNPCAfterAddingEmptySpace[0] != null) {
                    for (var i = 0; i < itemsInCargoOfNPCAfterAddingEmptySpace.length; i++) {
                        totalSizeOfNPCStorage += itemsInCargoOfNPCAfterAddingEmptySpace[i].quantity;
                    }
                }
                else {
                    totalSizeOfNPCStorage = 0;
                }
            }
            else {
                totalSizeOfNPCStorage = 0;
            }
        }
        else {
            totalSizeOfNPCStorage = 0;
        }



        var emptySpaceToRemoveNPC = totalSizeOfNPCStorage - initialQuantityOfItemsInNpcStorage;
        items.RemoveCargo(NPCContainerID, "goods_emptyspace", emptySpaceToRemoveNPC);


        if (initialQuantityOfItemsInNpcStorage < totalSizeOfNPCStorage) {

            return 1;
            //intSwitchTest = 2;
        }
        else {
            return 0;
			/*switchForStopNpc = false;

			if (game.IsAsteroidExists(lastListOfAsteroids[0]) == true)
			{
				AIIsPlayerCargoFull(lastListOfAsteroids[0]);
				break;
			}
			else
			{
				intSwitchTest = 0;
				break;
			}*/
        }
    }
}