
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
using(timer);

include(SC_Utilities.js);
include(SC_Salvage_Object_Storage.js);
include(SC_Salvage_Object_Timer.js);
include(SC_Salvaging_Utilities.js);


//depending on what the ship had equipped and what was in it's cargo before being destroyed, if it had minerals in its cargo, then some minerals should also be in the yield. to finish later.
var arrayOfSalvageableItems =
    [
        "goods_scrap_metal",
        "goods_bio_material",
        "goods_electronics",
        "goods_food",
        "goods_luxuries",
        "goods_medicine",
        "goods_polymers",
        "components_composites",
        "components_control",
        "components_electronic",
        "components_mechanical",
        "components_power",
        "components_structural"
    ];









function OnUpdateCache(args)
{
    /*//console.Print("Mining module started");
    //(ship's cache)-based updates
    var mining_range_modifier = ship.GetFinalCacheValue(args.ship_id, "mining_range");
    args.range = args.range * mining_range_modifier;

    var mining_amount_modifier = ship.GetFinalCacheValue(args.ship_id, "mining_amount");
    args.custom_parameters.amount = args.custom_parameters.amount * mining_amount_modifier;

    var mining_speed_modifier = ship.GetFinalCacheValue(args.ship_id, "mining_speed");
    args.custom_parameters.speed = args.custom_parameters.speed * mining_speed_modifier;
    */
    // console.Print("Mining range modifier current: " + mining_range_modifier);
    // console.Print("Mining amount modifier current: " + mining_amount_modifier);
    // console.Print("Mining speed modifier current: " + mining_speed_modifier);

    // console.Print("range now is " + args.range);
    // console.Print("amount now is " + args.custom_parameters.amount);
    // console.Print("speed now is " + args.custom_parameters.speed);
}

var systemSalvageableObjects = [];
var objectNDist = [];
var sys_id;
var deactivationcounter = -1;
var lasttimerFrameCounter = -1;
var lasttimerFrameCounterSwtch = 0;
var mainFrameCounter = 0;
var lastAngle;
var lastPos;
var cancelDeposits = 0;
var lastObjectCoords;
var lastShipCoords;
var timerFrameCounter = 0;

function OnStart(args)
{
    var playerCargoSpace = SC_Salvaging_Utilities.AIIsPlayerCargoFull(args.ship_id);


    if (playerCargoSpace == null)
    {
        if (playerCargoSpace.space == null)
        {
            //console.PrintError("null playerCargoSpace object data");
        }
        else {
            //console.PrintError("playerCargoSpace: " + playerCargoSpace.space);
        }
    }
    else {
        //console.PrintError("null playerCargoSpace object");
    }

    if (playerCargoSpace.space > 0)
    {
        sys_id = ship.GetSystemID(args.ship_id);

        storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id + "_swtch", 0);

        var stopDevice = SC_Salvage_Object_Timer.ClearCurrentDevice(sys_id, args.device_id, args.ship_id, args.slot_id);

        //SC_Salvage_Object_Timer.ClearCurrentDevice(sys_id, args.device_id, args.ship_id, args.slot_id);

        if (stopDevice == 1)
        {
            /*var counterDevice = SC_Salvage_Object_Timer.GetCounterDevice();
            SC_Salvage_Object_Timer.ResetCounterDevice();

            var randomYieldChooser = Math.floor(Math.random() * (5 - 0) + 0);

            var randomYieldToRemove = 0;

            if (randomYieldChooser == 5) {
                randomYieldToRemove = Math.floor(Math.random() * (5 - 0) + 0);
            }
            else {
                randomYieldToRemove = Math.floor(Math.random() * (3 - 1) + 1);
            }

            if (SC_Salvage_Object_Storage.GetSpaceObjectYield(args.sys_id, args.target_id) > 0)
            {
                var PlayerContainerID = items.GetGameObjectContainerId(args.ship_id);
                //console.PrintError("test0");

                //ship.AddItem(args.ship_id, "goods_scrap_metal", randomYieldToRemove);
                //items.AddItem(PlayerContainerID, "goods_scrap_metal", randomYieldToRemove);
                //items.AddItem(PlayerContainerID, "goods_emptyspace", randomYieldToRemove);//adds items to cargo hold

                ship.AddItem(args.ship_id, "goods_scrap_metal", randomYieldToRemove);
                //console.PrintError("test1");


                //var someGlobalIndex = storage.GetGlobal("GlobalIndex_Player_" + playerName);
                SC_Salvage_Object_Storage.RemoveFromYield(args.sys_id, args.target_id, randomYieldToRemove);
                //console.PrintError("test2");
            }
            else {

                SC_Salvage_Object_Storage.SetDepleted(args.sys_id, args.target_id);

                generator.RemoveSpaceObject(args.target_id);

                game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id); // + "_" + args.target_id
                visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");

                game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "Space object yield depleted");
                SC_Salvage_Object_Timer.ResetCounterDevice();

                var data = { swtch: 1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: null, reset: 0 };
                storage.SetGlobal("systemid_" + args.sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, data);

                timer.ClearTimer(args.timer_id);
            }*/
        }
        else
        {
            if (storage.IsSetGlobal("systemid_" + sys_id + "_salvage"))
            {
                systemSalvageableObjects = storage.GetGlobal("systemid_" + sys_id + "_salvage");

                if (systemSalvageableObjects != null)
                {
                    if (systemSalvageableObjects.length > 0)
                    {
                        objectNDist = [];

                        var coordsShip = ship.GetCoordinates(args.ship_id);

                        for (var s = 0; s < systemSalvageableObjects.length; s++)
                        {
                            var coordsObject = game.GetObjectCoordinates(sys_id, systemSalvageableObjects[s].id);
                            var objectDist = SC_Utilities.npcCheckDistance(coordsShip, coordsObject);

                            if (coordsObject == null)
                            {
                                //console.PrintError("null");
                            }
                            else
                            {
                                var someData = { id: systemSalvageableObjects[s].id, dist: objectDist, coordsX: coordsObject.x, coordsY: coordsObject.y };
                                objectNDist.push(someData);
                            }
                   
                        }

                        objectNDist.sort(function (a, b)
                        {
                            var adist = a.dist;
                            var bdist = b.dist;
                            //var acdam = a.damage;
                            //var bcdam = b.damage;

                            if (adist < bdist) {
                                return -1;
                            }
                            else if (adist > bdist) {
                                return 1;
                            }
                            else {
                                return 0;
                            }
                        });

                        if (objectNDist[0].dist < args.custom_parameters.maxDistance)
                        {
                            //console.PrintError("start mining");
                            timer.SetTimer(0.15, "CustomOnFrame", { sys_id: sys_id, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, duration: args.duration, target_id: objectNDist[0].id, frame: 0, countdownTimer: args.custom_parameters.countdownTimer, force: args.custom_parameters.force, techLevel: args.custom_parameters.techLevel, isInverted: args.custom_parameters.isInverted, manualOrAuto: args.custom_parameters.manualOrAuto, maxDistance: args.custom_parameters.maxDistance }, 0); //
                            //visual.DeviceActivateEffectOnObject(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, objectNDist[0].id);
                            visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", 0.25, objectNDist[0].coordsX, objectNDist[0].coordsY);

                            game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id, "special/mining_start.ogg", 0, false); // + "_" + objectNDist[0].id
                            game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id, "special/mining_process.ogg", 0.1, false); // + "_" + objectNDist[0].id

                            var dirX = coordsShip.x - objectNDist[0].coordsX;
                            var dirY = coordsShip.y - objectNDist[0].coordsY;

                            var someOtherMAG = Math.sqrt(((coordsShip.x - objectNDist[0].coordsX) * (coordsShip.x - objectNDist[0].coordsX)) + ((coordsShip.y - objectNDist[0].coordsY) * (coordsShip.y - objectNDist[0].coordsY)));

                            var dirNPCToPointInFrontOfWaypointX = ((dirX) / someOtherMAG) * objectNDist[0].dist * 0.00001;
                            var dirNPCToPointInFrontOfWaypointY = ((dirY) / someOtherMAG) * objectNDist[0].dist * 0.00001;

                            game.ApplyForceToObject(objectNDist[0].id, dirNPCToPointInFrontOfWaypointX, dirNPCToPointInFrontOfWaypointY);

                            lastObjectCoords = coordsObject;
                        }
                        else {
                            game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "The space object you are trying to salvage is too far.");
                        }

                        lastShipCoords = coordsShip;
                        lastObjectCoords = { x: objectNDist[0].coordsX, y: objectNDist[0].coordsY };

                    }
                    else {
                        game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "There is no salvage item in range.");
                    }
                }
                else {
                    game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "There is no salvage item in range.");
                }
            }
        }
    }
    else
    {
        game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "There is not enough space in your cargo.");
    }
    //timer.SetTimer(0.15, "CustomOnFrame", { sys_id: sys_id, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, duration: args.duration }, 0);
}


var swtchOnce = 0;

function CustomOnFrame(args)
{

    if (storage.GetGlobal("systemid_" + args.sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id + "_swtch") == 0)
    {
        SC_Salvage_Object_Timer.AddDeviceID(args.sys_id, args.device_id, args.slot_id, args.ship_id, args.timer_id);

        //console.PrintError("add device timer: " + args.timer_id);

        storage.SetGlobal("systemid_" + args.sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id + "_swtch", 1);
    }

    var counterDevice = SC_Salvage_Object_Timer.GetCounterDevice();

    if (counterDevice > args.countdownTimer)
    {
        var PlayerContainerID = items.GetGameObjectContainerId(args.ship_id);
        //items.AddItem(PlayerContainerID, itemsInCargoOfNPC[t].xml_id, q);

        var randomItemChooser = Math.floor(Math.random() * (arrayOfSalvageableItems.length - 0) + 0);

        var randomItemYieldChooser = Math.floor(Math.random() * (3 - 0) + 0);

        var randomItemYieldToRemove = 0;

        if (randomItemYieldChooser == 2) {
            randomItemYieldToRemove = Math.floor(Math.random() * (3 - 0) + 0);
        }
        else {
            randomItemYieldToRemove = Math.floor(Math.random() * (2 - 1) + 1);
        }


        var randomYieldChooser = Math.floor(Math.random() * (5 - 0) + 0);      

        var randomYieldToRemove = 0;

        if (randomYieldChooser == 4)
        {
            randomYieldToRemove = Math.floor(Math.random() * (5 - 0) + 0);
        }
        else
        {
            randomYieldToRemove = Math.floor(Math.random() * (3 - 1) + 1);
        }


        if (SC_Salvage_Object_Storage.GetSpaceObjectYield(args.sys_id, args.target_id) > 0)
        {
            var PlayerContainerID = items.GetGameObjectContainerId(args.ship_id);

            //console.PrintError("test0");

            //ship.AddItem(args.ship_id, "goods_scrap_metal", randomYieldToRemove);
            //items.AddItem(PlayerContainerID, "goods_scrap_metal", randomYieldToRemove);
            //items.AddItem(PlayerContainerID, "goods_emptyspace", randomYieldToRemove);//adds items to cargo hold

            ship.AddItem(args.ship_id, arrayOfSalvageableItems[randomItemChooser], randomYieldToRemove);
            ship.AddItem(args.ship_id, "goods_scrap_metal", randomItemYieldToRemove);




            //console.PrintError("test1");


            //var someGlobalIndex = storage.GetGlobal("GlobalIndex_Player_" + playerName);
            SC_Salvage_Object_Storage.RemoveFromYield(args.sys_id, args.target_id, randomYieldToRemove);
            //console.PrintError("test2");
        }
        else
        {

            SC_Salvage_Object_Storage.SetDepleted(args.sys_id, args.target_id);

            //generator.RemoveSpaceObject(args.target_id);
            game.DeleteGameObject(args.target_id);






            game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id); // + "_" + args.target_id
            visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");

            game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "Space object yield depleted");
            SC_Salvage_Object_Timer.ResetCounterDevice();

            var data = { swtch: 1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: null, reset: 0 };
            storage.SetGlobal("systemid_" + args.sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, data);



            timer.ClearTimer(args.timer_id);
        }


        if (args.manualOrAuto == 1)
        {
            game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id); // + "_" + args.target_id
            visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
            SC_Salvage_Object_Timer.ResetCounterDevice();
        }
        else
        {
            game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id); // + "_" + args.target_id
            visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
            SC_Salvage_Object_Timer.ResetCounterDevice();
            timer.ClearTimer(args.timer_id);
        }
    }
    else
    {
        /*if (args == null || args.ship_id == null || args.target_id == null || args.slot_id == null || args.device_id == null)
        {
            if (args == null)
            {
                console.PrintError("null args");
            }
            if (args.ship_id == null) {

                console.PrintError("null ship_id");
            }
            if (args.target_id == null) {

                console.PrintError("null target_id");
            }
            if (args.slot_id == null) {

                console.PrintError("null slot_id");
            }
            if (args.device_id == null) {

                console.PrintError("null device_id");
            }
        }

        if (args.maxDistance == null)
        {
            console.PrintError("null maxDistance");
        }*/

        var coordsShip = ship.GetCoordinates(args.ship_id);
        var coordsObject = game.GetObjectCoordinates(sys_id, args.target_id);
        var objectDist = SC_Utilities.npcCheckDistance(coordsShip, coordsObject);
        var coordsCurx = Math.round(coordsObject.x * 10) * 0.1;
        var coordsCury = Math.round(coordsObject.y * 10) * 0.1;
        var coordsShipCurx = Math.round(coordsShip.x * 10) * 0.1;
        var coordsShipCury = Math.round(coordsShip.y * 10) * 0.1;

        if (counterDevice == 0)
        {
            //game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id); // + "_" + args.target_id
            visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");

            visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, coordsObject.x, coordsObject.y);

            game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + args.target_id, "special/mining_start.ogg", 0, false);
            game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id, "special/mining_process.ogg", 0, true); // + "_" + args.target_id

            var dirX = coordsShip.x - coordsCurx;
            var dirY = coordsShip.y - coordsCury;

            var someOtherMAG = Math.sqrt(((coordsShip.x - coordsCurx) * (coordsShip.x - coordsCurx)) + ((coordsShip.y - coordsCury) * (coordsShip.y - coordsCury)));

            var dirNPCToPointInFrontOfWaypointX = ((dirX) / someOtherMAG) * objectDist * 0.00001;
            var dirNPCToPointInFrontOfWaypointY = ((dirY) / someOtherMAG) * objectDist * 0.00001;

            game.ApplyForceToObject(args.target_id, dirNPCToPointInFrontOfWaypointX, dirNPCToPointInFrontOfWaypointY);

        }

        //console.PrintError("dist: " + objectDist);
        if (lastShipCoords != null && lastObjectCoords != null)
        {
            var coordsShipLastx = Math.round(lastShipCoords.x * 10) * 0.1;
            var coordsShipLasty = Math.round(lastShipCoords.y * 10) * 0.1;

            var coordsLastx = Math.round(lastObjectCoords.x * 10) * 0.1;
            var coordsLasty = Math.round(lastObjectCoords.y * 10) * 0.1;

            if (coordsShipCurx != coordsShipLastx || coordsShipCury != coordsShipLasty || coordsCurx != coordsLastx || coordsCury != coordsLasty) {
                //console.PrintError(args.maxDistance);

                if (objectDist < args.maxDistance)
                {
                    //game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id); // + "_" + args.target_id
                    visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");

                    visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, coordsObject.x, coordsObject.y);

                    //game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id, "special/mining_start.ogg", 0, false); // + "_" + args.target_id
                    //game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id, "special/mining_process.ogg", 0, true); // + "_" + args.target_id

                    var dirX = coordsShip.x - coordsCurx;
                    var dirY = coordsShip.y - coordsCury;

                    var someOtherMAG = Math.sqrt(((coordsShip.x - coordsCurx) * (coordsShip.x - coordsCurx)) + ((coordsShip.y - coordsCury) * (coordsShip.y - coordsCury)));

                    var dirNPCToPointInFrontOfWaypointX = ((dirX) / someOtherMAG) * objectDist * 0.00001;
                    var dirNPCToPointInFrontOfWaypointY = ((dirY) / someOtherMAG) * objectDist * 0.00001;

                    game.ApplyForceToObject(args.target_id, dirNPCToPointInFrontOfWaypointX, dirNPCToPointInFrontOfWaypointY);

                }
                else {
                    game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id); // + "_" + args.target_id
                    visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
                    game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "space object too far");
                    timer.ClearTimer(args.timer_id);
                    SC_Salvage_Object_Timer.ResetCounterDevice();

                    var data = { swtch: 1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: null, reset: 0 };
                    storage.SetGlobal("systemid_" + args.sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, data);
                }
            }
            else
            {

            }
        }
        else
        {

        }
        lastShipCoords = coordsShip;
        lastObjectCoords = coordsObject;
        SC_Salvage_Object_Timer.AddCounterDevice();
    }
}

function WaitingForDeviceDeactivation(args)
{
    deactivationcounter++;
}

function OnFinished(args)
{

}

function OnCancel(args)
{

}

function OnFrame(args)
{
    //console.PrintError("OnFrameCounter: " + OnFrameCounter);
    //OnFrameCounter++;
}



        /*var deviceData = SC_Salvage_Object_Timer.SetDevice(args.sys_id, args.device_id, args.ship_id, args.timer_id);

        if (deviceData.timer_id == null)
        {

        }

        console.PrintError("timer_id: " + deviceData.timer_id);*/

        /*if (deviceData != null)
        {
            if (deviceData.timer_id != null)
            {
                console.PrintError("timer is supposedly cleared");
                timer.ClearTimer(deviceData.timer_id);
            }
            else 
            {
                console.PrintError("null timer");
            }
        }*/

    //console.PrintError("counter " + counterDevice);

    //args.custom_parameters.countdownTimer

    /*if (counterDevice > args.countdownTimer)
    {
        timer.ClearTimer(deviceData.timer_id);
        SC_Salvage_Object_Timer.ResetCounterDevice();
    }*/



                     /*game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "You cannot move the ship when salvaging.");
                        game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                        visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
                        timer.ClearTimer(args.timer_id);
                    }
                    else
                    {
                        var coordsCurx = Math.round(coordsObject.x * 10) * 0.1;
                        var coordsCury = Math.round(coordsObject.y * 10) * 0.1;
                        var coordsLastx = Math.round(lastObjectCoords.x * 10) * 0.1;
                        var coordsLasty = Math.round(lastObjectCoords.y * 10) * 0.1;

                        if (coordsCurx != coordsLastx || coordsCury != coordsLasty) {
                            /*game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "space object too far");
                            game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                            visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
                            timer.ClearTimer(args.timer_id);
                            game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                            visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");

                            storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: 1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: args.timer_id, timerFrameCounter: timerFrameCounter });

                            visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, coordsObject.x, coordsObject.y);
                            game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + args.target_id, "special/mining_start.ogg", 0, false);
                            game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id, "special/mining_process.ogg", 0.1, true);

                            var dirX = coordsShip.x - coordsCurx;
                            var dirY = coordsShip.y - coordsCury;


                            var someOtherMAG = Math.sqrt(((coordsShip.x - coordsCurx) * (coordsShip.x - coordsCurx)) + ((coordsShip.y - coordsCury) * (coordsShip.y - coordsCury)));

                            var dirNPCToPointInFrontOfWaypointX = ((dirX) / someOtherMAG) * objectDist * 0.00001;
                            var dirNPCToPointInFrontOfWaypointY = ((dirY) / someOtherMAG) * objectDist * 0.00001;

                            game.ApplyForceToObject(args.target_id, dirNPCToPointInFrontOfWaypointX, dirNPCToPointInFrontOfWaypointY);
                        }
                        else {
                            if (objectDist < args.maxDistance) {
                                game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                                visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");

                                storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: 1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: args.timer_id, timerFrameCounter: timerFrameCounter  });

                                visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, coordsObject.x, coordsObject.y);

                                game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + args.target_id, "special/mining_start.ogg", 0, false);
                                game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id, "special/mining_process.ogg", 0.1, true);

                                var dirX = coordsShip.x - coordsCurx;
                                var dirY = coordsShip.y - coordsCury;

                                var someOtherMAG = Math.sqrt(((coordsShip.x - coordsCurx) * (coordsShip.x - coordsCurx)) + ((coordsShip.y - coordsCury) * (coordsShip.y - coordsCury)));

                                var dirNPCToPointInFrontOfWaypointX = ((dirX) / someOtherMAG) * objectDist * 0.00001;
                                var dirNPCToPointInFrontOfWaypointY = ((dirY) / someOtherMAG) * objectDist * 0.00001;

                                game.ApplyForceToObject(args.target_id, dirNPCToPointInFrontOfWaypointX, dirNPCToPointInFrontOfWaypointY);
                            }
                            else {
                                timerFrameCounter = 0;
                                console.PrintError("> args.maxDistance");
                                game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                                visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
                                timer.ClearTimer(args.timer_id);
                                SC_Salvage_Object_Timer.ResetCounterDevice();
                                //storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, null);
                                storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: -1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: args.timer_id, timerFrameCounter: timerFrameCounter  });
                            }
                        }
                            //visual.DeviceActivateEffectOnObject(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, args.target_id);
                            //visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, coordsObject.x, coordsObject.y);
                            //game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + args.target_id, "special/mining_start.ogg", 0, false);
                            //game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id, "special/mining_process.ogg", 0.1, true);

                    }
                }
            }

            lastShipCoords = coordsShip;
            lastObjectCoords = coordsObject;
            */








    //SC_Salvage_Object_Storage.AddSpaceObjectToStorage(sys_id, null);
    //var length = SC_Salvage_Object_Storage.GetSpaceObjectStorageLength();
    //console.PrintError();

    //SC_Salvage_Object_Timer.AddCounterFrame(); //var currentCount = 
    //var currentCount = SC_Salvage_Object_Timer.GetCounterFrame();
    //console.PrintError("currentCount: " + currentCount);







    /*else //if (storage.IsSetGlobal("systemid_" + sys_id + "_salvage_ship_" + args.ship_id + "_device_" + args.device_id))
    {

        game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
        visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
        timer.ClearTimer(args.timer_id);
        SC_Salvage_Object_Timer.ResetCounterDevice();

        storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, null);

        if (storage.IsSetGlobal("systemid_" + sys_id + "_salvage")) {
            systemSalvageableObjects = storage.GetGlobal("systemid_" + sys_id + "_salvage");

            if (systemSalvageableObjects != null) {
                if (systemSalvageableObjects.length > 0) {
                    objectNDist = [];

                    var coordsShip = ship.GetCoordinates(args.ship_id);

                    for (var s = 0; s < systemSalvageableObjects.length; s++) {
                        var coordsObject = game.GetObjectCoordinates(sys_id, systemSalvageableObjects[s].id);
                        var objectDist = SC_Utilities.npcCheckDistance(coordsShip, coordsObject);

                        var someData = { id: systemSalvageableObjects[s].id, dist: objectDist, coordsX: coordsObject.x, coordsY: coordsObject.y };
                        objectNDist.push(someData);
                    }


                    objectNDist.sort(function (a, b) {
                        var adist = a.dist;
                        var bdist = b.dist;
                        //var acdam = a.damage;
                        //var bcdam = b.damage;

                        if (adist < bdist) {
                            return -1;
                        }
                        else if (adist > bdist) {
                            return 1;
                        }
                        else {
                            return 0;
                        }
                    });

                    if (objectNDist[0].dist < 150) {
                        timer.SetTimer(0.15, "CustomOnFrame", { ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, duration: args.duration, target_id: objectNDist[0].id, frame: 0, countdownTimer: args.custom_parameters.countdownTimer, force: args.custom_parameters.force, techLevel: args.custom_parameters.techLevel, isInverted: args.custom_parameters.isInverted, manualOrAuto: args.custom_parameters.manualOrAuto }, 0); //
                        //visual.DeviceActivateEffectOnObject(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, objectNDist[0].id);
                        visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", 0.25, objectNDist[0].coordsX, objectNDist[0].coordsY);

                        game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + objectNDist[0].id, "special/mining_start.ogg", 0, false);
                        game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id + "_" + objectNDist[0].id, "special/mining_process.ogg", 0.1, true);


                        var dirX = coordsShip.x - objectNDist[0].coordsX;
                        var dirY = coordsShip.y - objectNDist[0].coordsY;


                        var someOtherMAG = Math.sqrt(((coordsShip.x - objectNDist[0].coordsX) * (coordsShip.x - objectNDist[0].coordsX)) + ((coordsShip.y - objectNDist[0].coordsY) * (coordsShip.y - objectNDist[0].coordsY)));

                        var dirNPCToPointInFrontOfWaypointX = ((dirX) / someOtherMAG) * objectNDist[0].dist * 0.00001;
                        var dirNPCToPointInFrontOfWaypointY = ((dirY) / someOtherMAG) * objectNDist[0].dist * 0.00001;

                        game.ApplyForceToObject(objectNDist[0].id, dirNPCToPointInFrontOfWaypointX, dirNPCToPointInFrontOfWaypointY);

                        lastObjectCoords = coordsObject;


                        storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: 1 });
                    }
                    else {
                        game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "space object too far");
                    }

                    lastShipCoords = coordsShip;
                    lastObjectCoords = { x: objectNDist[0].coordsX, y: objectNDist[0].coordsY };

                    /*var coordsShip = ship.GetCoordinates(args.ship_id);
                    for (var s = 0; s < systemSalvageableObjects.length; s++)
                    {
                        var coordsObject = game.GetObjectCoordinates(sys_id, systemSalvageableObjects[s].id);
                        var droneDist = SC_Utilities.npcCheckDistance(coordsShip, coordsObject);
    
                        if (droneDist < 250)
                        {
                            //visual.DeviceActivateEffectOnObject(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, systemSalvageableObjects[0].id);
                            visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration,coordsObject.x, coordsObject.y);
                            game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + systemSalvageableObjects[0].id, "special/mining_start.ogg", 0, false);
                            game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id + "_" + systemSalvageableObjects[0].id, "special/mining_process.ogg", 0.1, true);
                        }
                    }
                }
                else {
                    console.PrintError("there is no salvage item in range 1");
                }
            }
            else {
                console.PrintError("there is no salvage item in range 0");

            }
        }

        storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: 1 });

    }*/



                    /*if (coordsShipCurx != coordsShipLastx || coordsShipCury != coordsShipLasty || coordsCurx != coordsLastx || coordsCury != coordsLasty)
                    {
                        game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "You cannot move the ship when salvaging.");
                        game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                        visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
                        timer.ClearTimer(args.timer_id);
                    }
                    else
                    {
                        //game.ApplyForceToObject(153, 10, 0);
                        if (objectDist < 150)
                        {
                            var coordsCurx = Math.round(coordsObject.x * 10) * 0.1;
                            var coordsCury = Math.round(coordsObject.y * 10) * 0.1;
                            var coordsLastx = Math.round(lastObjectCoords.x * 10) * 0.1;
                            var coordsLasty = Math.round(lastObjectCoords.y * 10) * 0.1;
    
                            if (coordsCurx != coordsLastx || coordsCury != coordsLasty) {
                                game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "space object too far");
                                game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                                visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
                                timer.ClearTimer(args.timer_id);
                            }
                            //visual.DeviceActivateEffectOnObject(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, args.target_id);
                            //visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, coordsObject.x, coordsObject.y);
                            //game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + args.target_id, "special/mining_start.ogg", 0, false);
                            //game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id, "special/mining_process.ogg", 0.1, true);
                        }
                        else {
                            game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                            visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
                            game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "space object too far");
                            timer.ClearTimer(args.timer_id);
                        }
                    }*/
    //console.Print("Mining module started");

    //calculate max_length property value based on speed and asteroid's difficulty
    /*var asteroid = game.GetAsteroidDescriptionByID(args.target_id);
    args.duration = asteroid.difficulty / args.custom_parameters.speed;
    args.cooldown = args.duration;

    visual.DeviceActivateEffectOnObject(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, args.target_id);

    // console.Print("max_length now is " + args.duration);
    game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + args.target_id, "special/mining_start.ogg", 0, false);
    game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id, "special/mining_process.ogg", 0.1, true);*/






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
        /*// give some experience
        if (items_money_added > 0) {
            var mining_exp_mult = 1 + ship.GetFinalCacheValue(args.ship_id, "mining_experience");
            ship.SetCurrentValueDelta(shipId, "experience", (mining_exp_mult * items_money_added) / 7);
            // console.Print("Mining experience multiplier: " + mining_exp_mult);
        }*/




    //console.Print("Mining module frame. Energy deduction");
    //console.Print("Mining module started");

    /*if (radToDeg < 0) {
        radToDeg = 180 + (radToDeg * -1);
    }*/
    //console.PrintError(radToDeg);

    //args.ship_id




        /*if (lastPos != null)
        {
            if (lastPos.x != null && lastPos.y != null)
            {
                if (coordsNPC.x != lastPos.x && coordsNPC.y != lastPos.y)
                {
                    game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                    npc.DeviceDeactivate(args.ship_id, "device_mining_civilian_NPC");
                    visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
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
                visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
                cancelDeposits = 1;
            }
        }
        lastPos = coordsNPC;
        lastAngle = radToDeg;*/
    /*

    if (lastAngle != null)
    {
        if (radToDeg != lastAngle)
        {
            game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);

            npc.DeviceDeactivate(args.ship_id, "device_mining_civilian_NPC");
            visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
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















    /*
    sys_id = ship.GetSystemID(args.ship_id);

    //console.PrintError("start");

    if (!storage.IsSetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id))
    {
        console.PrintError("activated 0");

        if (storage.IsSetGlobal("systemid_" + sys_id + "_salvage"))
        {
            systemSalvageableObjects = storage.GetGlobal("systemid_" + sys_id + "_salvage");

            if (systemSalvageableObjects != null)
            {
                if (systemSalvageableObjects.length > 0)
                {
                    objectNDist = [];

                    var coordsShip = ship.GetCoordinates(args.ship_id);

                    for (var s = 0; s < systemSalvageableObjects.length; s++) {
                        var coordsObject = game.GetObjectCoordinates(sys_id, systemSalvageableObjects[s].id);
                        var objectDist = SC_Utilities.npcCheckDistance(coordsShip, coordsObject);

                        var someData = { id: systemSalvageableObjects[s].id, dist: objectDist, coordsX: coordsObject.x, coordsY: coordsObject.y };
                        objectNDist.push(someData);
                    }


                    objectNDist.sort(function (a, b) {
                        var adist = a.dist;
                        var bdist = b.dist;
                        //var acdam = a.damage;
                        //var bcdam = b.damage;

                        if (adist < bdist) {
                            return -1;
                        }
                        else if (adist > bdist) {
                            return 1;
                        }
                        else {
                            return 0;
                        }
                    });

                    if (objectNDist[0].dist < args.custom_parameters.maxDistance) {
                        mainFrameCounter = 1;
                        timer.SetTimer(0.15, "CustomOnFrame", { ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, duration: args.duration, target_id: objectNDist[0].id, frame: 0, countdownTimer: args.custom_parameters.countdownTimer, force: args.custom_parameters.force, techLevel: args.custom_parameters.techLevel, isInverted: args.custom_parameters.isInverted, manualOrAuto: args.custom_parameters.manualOrAuto, maxDistance: args.custom_parameters.maxDistance }, 0); //
                        //visual.DeviceActivateEffectOnObject(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, objectNDist[0].id);
                        visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", 0.25, objectNDist[0].coordsX, objectNDist[0].coordsY);

                        game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + objectNDist[0].id, "special/mining_start.ogg", 0, false);
                        game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id + "_" + objectNDist[0].id, "special/mining_process.ogg", 0.1, true);


                        var dirX = coordsShip.x - objectNDist[0].coordsX;
                        var dirY = coordsShip.y - objectNDist[0].coordsY;


                        var someOtherMAG = Math.sqrt(((coordsShip.x - objectNDist[0].coordsX) * (coordsShip.x - objectNDist[0].coordsX)) + ((coordsShip.y - objectNDist[0].coordsY) * (coordsShip.y - objectNDist[0].coordsY)));

                        var dirNPCToPointInFrontOfWaypointX = ((dirX) / someOtherMAG) * objectNDist[0].dist * 0.00001;
                        var dirNPCToPointInFrontOfWaypointY = ((dirY) / someOtherMAG) * objectNDist[0].dist * 0.00001;

                        game.ApplyForceToObject(objectNDist[0].id, dirNPCToPointInFrontOfWaypointX, dirNPCToPointInFrontOfWaypointY);

                        lastObjectCoords = coordsObject;


                        storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: 1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: null });
                    }
                    else {
                        game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "space object too far");
                    }

                    lastShipCoords = coordsShip;
                    lastObjectCoords = { x: objectNDist[0].coordsX, y: objectNDist[0].coordsY };
                }
                else {
                    console.PrintError("there is no salvage item in range 1");
                }
            }
            else {
                console.PrintError("there is no salvage item in range 0");
            }
        }
    }
    else //if (data.swtch == 2)
    {
        var data = storage.GetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id);

        if (data.timer_id != null)
        {
            //console.PrintError("timer is not equal and so there is a current running timer function for this device 1");
            timer.ClearTimer(data.timer_id);
        }


        if (storage.IsSetGlobal("systemid_" + sys_id + "_salvage")) {
            systemSalvageableObjects = storage.GetGlobal("systemid_" + sys_id + "_salvage");

            if (systemSalvageableObjects != null) {
                if (systemSalvageableObjects.length > 0) {
                    objectNDist = [];

                    var coordsShip = ship.GetCoordinates(args.ship_id);

                    for (var s = 0; s < systemSalvageableObjects.length; s++) {
                        var coordsObject = game.GetObjectCoordinates(sys_id, systemSalvageableObjects[s].id);
                        var objectDist = SC_Utilities.npcCheckDistance(coordsShip, coordsObject);

                        var someData = { id: systemSalvageableObjects[s].id, dist: objectDist, coordsX: coordsObject.x, coordsY: coordsObject.y };
                        objectNDist.push(someData);
                    }


                    objectNDist.sort(function (a, b) {
                        var adist = a.dist;
                        var bdist = b.dist;
                        //var acdam = a.damage;
                        //var bcdam = b.damage;

                        if (adist < bdist) {
                            return -1;
                        }
                        else if (adist > bdist) {
                            return 1;
                        }
                        else {
                            return 0;
                        }
                    });

                    if (objectNDist[0].dist < args.custom_parameters.maxDistance) {
                        mainFrameCounter = 1;
                        timer.SetTimer(0.15, "CustomOnFrame", { ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, duration: args.duration, target_id: objectNDist[0].id, frame: 0, countdownTimer: args.custom_parameters.countdownTimer, force: args.custom_parameters.force, techLevel: args.custom_parameters.techLevel, isInverted: args.custom_parameters.isInverted, manualOrAuto: args.custom_parameters.manualOrAuto, maxDistance: args.custom_parameters.maxDistance }, 0); //
                        //visual.DeviceActivateEffectOnObject(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, objectNDist[0].id);
                        visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", 0.25, objectNDist[0].coordsX, objectNDist[0].coordsY);

                        game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + objectNDist[0].id, "special/mining_start.ogg", 0, false);
                        game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id + "_" + objectNDist[0].id, "special/mining_process.ogg", 0.1, true);


                        var dirX = coordsShip.x - objectNDist[0].coordsX;
                        var dirY = coordsShip.y - objectNDist[0].coordsY;


                        var someOtherMAG = Math.sqrt(((coordsShip.x - objectNDist[0].coordsX) * (coordsShip.x - objectNDist[0].coordsX)) + ((coordsShip.y - objectNDist[0].coordsY) * (coordsShip.y - objectNDist[0].coordsY)));

                        var dirNPCToPointInFrontOfWaypointX = ((dirX) / someOtherMAG) * objectNDist[0].dist * 0.00001;
                        var dirNPCToPointInFrontOfWaypointY = ((dirY) / someOtherMAG) * objectNDist[0].dist * 0.00001;

                        game.ApplyForceToObject(objectNDist[0].id, dirNPCToPointInFrontOfWaypointX, dirNPCToPointInFrontOfWaypointY);

                        lastObjectCoords = coordsObject;


                        storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: 1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: null });
                    }
                    else {
                        game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "space object too far");
                    }

                    lastShipCoords = coordsShip;
                    lastObjectCoords = { x: objectNDist[0].coordsX, y: objectNDist[0].coordsY };
                }
                else {
                    console.PrintError("there is no salvage item in range 1");
                }
            }
            else {
                console.PrintError("there is no salvage item in range 0");
            }
        }
        */

        //storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, null);



        /*
        if (storage.IsSetGlobal("systemid_" + sys_id + "_salvage"))
        {
            systemSalvageableObjects = storage.GetGlobal("systemid_" + sys_id + "_salvage");

            if (systemSalvageableObjects != null)
            {
                if (systemSalvageableObjects.length > 0)
                {
                    objectNDist = [];

                    var coordsShip = ship.GetCoordinates(args.ship_id);

                    for (var s = 0; s < systemSalvageableObjects.length; s++)
                    {
                        var coordsObject = game.GetObjectCoordinates(sys_id, systemSalvageableObjects[s].id);
                        var objectDist = SC_Utilities.npcCheckDistance(coordsShip, coordsObject);

                        var someData = { id: systemSalvageableObjects[s].id, dist: objectDist, coordsX: coordsObject.x, coordsY: coordsObject.y };
                        objectNDist.push(someData);
                    }



                    objectNDist.sort(function (a, b)
                    {
                        var adist = a.dist;
                        var bdist = b.dist;
                        //var acdam = a.damage;
                        //var bcdam = b.damage;

                        if (adist < bdist) {
                            return -1;
                        }
                        else if (adist > bdist) {
                            return 1;
                        }
                        else {
                            return 0;
                        }
                    });




                    if (objectNDist[0].dist < args.custom_parameters.maxDistance)
                    {
                        mainFrameCounter = 1;

                        //storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: 1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: null });
                        timer.SetTimer(0.15, "WaitingForDeviceDeactivation", { ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, duration: args.duration, target_id: objectNDist[0].id, frame: 0, countdownTimer: args.custom_parameters.countdownTimer, force: args.custom_parameters.force, techLevel: args.custom_parameters.techLevel, isInverted: args.custom_parameters.isInverted, manualOrAuto: args.custom_parameters.manualOrAuto, maxDistance: args.custom_parameters.maxDistance, timerduration:3 }, 0); //
                        //visual.DeviceActivateEffectOnObject(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, objectNDist[0].id);
                        /*visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", 0.25, objectNDist[0].coordsX, objectNDist[0].coordsY);

                        game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + objectNDist[0].id, "special/mining_start.ogg", 0, false);
                        game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id + "_" + objectNDist[0].id, "special/mining_process.ogg", 0.1, true);


                        var dirX = coordsShip.x - objectNDist[0].coordsX;
                        var dirY = coordsShip.y - objectNDist[0].coordsY;


                        var someOtherMAG = Math.sqrt(((coordsShip.x - objectNDist[0].coordsX) * (coordsShip.x - objectNDist[0].coordsX)) + ((coordsShip.y - objectNDist[0].coordsY) * (coordsShip.y - objectNDist[0].coordsY)));

                        var dirNPCToPointInFrontOfWaypointX = ((dirX) / someOtherMAG) * objectNDist[0].dist * 0.00001;
                        var dirNPCToPointInFrontOfWaypointY = ((dirY) / someOtherMAG) * objectNDist[0].dist * 0.00001;

                        game.ApplyForceToObject(objectNDist[0].id, dirNPCToPointInFrontOfWaypointX, dirNPCToPointInFrontOfWaypointY);

                        lastObjectCoords = coordsObject;


                        storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: 1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: null });

                    }
                    else
                    {
                        game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "space object too far");
                    }

                    lastShipCoords = coordsShip;
                    lastObjectCoords = { x: objectNDist[0].coordsX, y: objectNDist[0].coordsY };
                }
                else {
                    console.PrintError("there is no salvage item in range 1");
                }
            }
            else {
                console.PrintError("there is no salvage item in range 0");
            }
        }
    }*/












    /*

    if (lasttimerFrameCounterSwtch == 0)
    {
        var data = storage.GetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id);

        if (data != null) {

            lasttimerFrameCounter = data.timerFrameCounter;
        }
        else {
            console.PrintError("null data");
        }
        lasttimerFrameCounterSwtch = 1;
    }

    console.PrintError("WaitingForDeviceDeactivation: " + deactivationcounter);

    if (args == null || args.ship_id == null || args.target_id == null || args.slot_id == null || args.device_id == null)
    {
        if (args == null)
        {
            console.PrintError("null args");
        }
        if (args.ship_id == null)
        {
            console.PrintError("null ship_id");
        }
        if (args.target_id == null)
        {
            console.PrintError("null target_id");
        }
        if (args.slot_id == null)
        {
            console.PrintError("null slot_id");
        }
        if (args.device_id == null)
        {
            console.PrintError("null device_id");
        }
    }



    if (deactivationcounter >= args.timerduration)
    {
        if (sys_id == null)
        {
            sys_id = ship.GetSystemID(args.ship_id);

        }
        var data = storage.GetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id);

        if (data != null) {

            if (data.swtch == -1)
            {

                if (storage.IsSetGlobal("systemid_" + sys_id + "_salvage")) {
                    systemSalvageableObjects = storage.GetGlobal("systemid_" + sys_id + "_salvage");

                    if (systemSalvageableObjects != null) {
                        if (systemSalvageableObjects.length > 0) {
                            objectNDist = [];

                            var coordsShip = ship.GetCoordinates(args.ship_id);

                            for (var s = 0; s < systemSalvageableObjects.length; s++) {
                                var coordsObject = game.GetObjectCoordinates(sys_id, systemSalvageableObjects[s].id);
                                var objectDist = SC_Utilities.npcCheckDistance(coordsShip, coordsObject);

                                var someData = { id: systemSalvageableObjects[s].id, dist: objectDist, coordsX: coordsObject.x, coordsY: coordsObject.y };
                                objectNDist.push(someData);
                            }


                            objectNDist.sort(function (a, b) {
                                var adist = a.dist;
                                var bdist = b.dist;
                                //var acdam = a.damage;
                                //var bcdam = b.damage;

                                if (adist < bdist) {
                                    return -1;
                                }
                                else if (adist > bdist) {
                                    return 1;
                                }
                                else {
                                    return 0;
                                }
                            });

                            if (objectNDist[0].dist < args.maxDistance) {
                                lasttimerFrameCounter = -1;
                                lasttimerFrameCounterSwtch = 0;
                                deactivationcounter = 0;
                                mainFrameCounter = 1;

                                timer.SetTimer(0.15, "CustomOnFrame", { ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, duration: args.duration, target_id: objectNDist[0].id, frame: 0, countdownTimer: args.countdownTimer, force: args.force, techLevel: args.techLevel, isInverted: args.isInverted, manualOrAuto: args.manualOrAuto, maxDistance: args.maxDistance }, 0); //
                                //visual.DeviceActivateEffectOnObject(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, objectNDist[0].id);
                                visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", 0.25, objectNDist[0].coordsX, objectNDist[0].coordsY);

                                game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + objectNDist[0].id, "special/mining_start.ogg", 0, false);
                                game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id + "_" + objectNDist[0].id, "special/mining_process.ogg", 0.1, true);


                                var dirX = coordsShip.x - objectNDist[0].coordsX;
                                var dirY = coordsShip.y - objectNDist[0].coordsY;


                                var someOtherMAG = Math.sqrt(((coordsShip.x - objectNDist[0].coordsX) * (coordsShip.x - objectNDist[0].coordsX)) + ((coordsShip.y - objectNDist[0].coordsY) * (coordsShip.y - objectNDist[0].coordsY)));

                                var dirNPCToPointInFrontOfWaypointX = ((dirX) / someOtherMAG) * objectNDist[0].dist * 0.00001;
                                var dirNPCToPointInFrontOfWaypointY = ((dirY) / someOtherMAG) * objectNDist[0].dist * 0.00001;

                                game.ApplyForceToObject(objectNDist[0].id, dirNPCToPointInFrontOfWaypointX, dirNPCToPointInFrontOfWaypointY);

                                lastObjectCoords = coordsObject;


                                //storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: 1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: null });
                                timer.ClearTimer(args.timer_id);
                            }
                            else {
                                game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "space object too far");
                            }

                            lastShipCoords = coordsShip;
                            lastObjectCoords = { x: objectNDist[0].coordsX, y: objectNDist[0].coordsY };
                        }
                        else {
                            console.PrintError("there is no salvage item in range 1");
                        }
                    }
                    else {
                        console.PrintError("there is no salvage item in range 0");
                    }
                }
               
            }
            else
            {
                if (lasttimerFrameCounter == data.timerFrameCounter)
                {
                    console.PrintError("timer is equal and so there are no current running timer function for this device 0");
                    /*if (storage.IsSetGlobal("systemid_" + sys_id + "_salvage"))
                    {
                        systemSalvageableObjects = storage.GetGlobal("systemid_" + sys_id + "_salvage");

                        if (systemSalvageableObjects != null) {
                            if (systemSalvageableObjects.length > 0) {
                                objectNDist = [];

                                var coordsShip = ship.GetCoordinates(args.ship_id);

                                for (var s = 0; s < systemSalvageableObjects.length; s++) {
                                    var coordsObject = game.GetObjectCoordinates(sys_id, systemSalvageableObjects[s].id);
                                    var objectDist = SC_Utilities.npcCheckDistance(coordsShip, coordsObject);

                                    var someData = { id: systemSalvageableObjects[s].id, dist: objectDist, coordsX: coordsObject.x, coordsY: coordsObject.y };
                                    objectNDist.push(someData);
                                }


                                objectNDist.sort(function (a, b) {
                                    var adist = a.dist;
                                    var bdist = b.dist;
                                    //var acdam = a.damage;
                                    //var bcdam = b.damage;

                                    if (adist < bdist) {
                                        return -1;
                                    }
                                    else if (adist > bdist) {
                                        return 1;
                                    }
                                    else {
                                        return 0;
                                    }
                                });

                                if (objectNDist[0].dist < args.maxDistance) {


                                    mainFrameCounter = 1;

                                    timer.SetTimer(0.15, "CustomOnFrame", { ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, duration: args.duration, target_id: objectNDist[0].id, frame: 0, countdownTimer: args.countdownTimer, force: args.force, techLevel: args.techLevel, isInverted: args.isInverted, manualOrAuto: args.manualOrAuto, maxDistance: args.maxDistance }, 0); //
                                    //visual.DeviceActivateEffectOnObject(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, objectNDist[0].id);
                                    visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", 0.25, objectNDist[0].coordsX, objectNDist[0].coordsY);

                                    game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + objectNDist[0].id, "special/mining_start.ogg", 0, false);
                                    game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id + "_" + objectNDist[0].id, "special/mining_process.ogg", 0.1, true);


                                    var dirX = coordsShip.x - objectNDist[0].coordsX;
                                    var dirY = coordsShip.y - objectNDist[0].coordsY;


                                    var someOtherMAG = Math.sqrt(((coordsShip.x - objectNDist[0].coordsX) * (coordsShip.x - objectNDist[0].coordsX)) + ((coordsShip.y - objectNDist[0].coordsY) * (coordsShip.y - objectNDist[0].coordsY)));

                                    var dirNPCToPointInFrontOfWaypointX = ((dirX) / someOtherMAG) * objectNDist[0].dist * 0.00001;
                                    var dirNPCToPointInFrontOfWaypointY = ((dirY) / someOtherMAG) * objectNDist[0].dist * 0.00001;

                                    game.ApplyForceToObject(objectNDist[0].id, dirNPCToPointInFrontOfWaypointX, dirNPCToPointInFrontOfWaypointY);

                                    lastObjectCoords = coordsObject;


                                    //storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: 1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: null });
                                    timer.ClearTimer(args.timer_id);
                                }
                                else {
                                    game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "space object too far");
                                }

                                lastShipCoords = coordsShip;
                                lastObjectCoords = { x: objectNDist[0].coordsX, y: objectNDist[0].coordsY };
                            }
                            else {
                                console.PrintError("there is no salvage item in range 1");
                            }
                        }
                        else {
                            console.PrintError("there is no salvage item in range 0");
                        }
                    }
                }
                else
                {
                    //console.PrintError("timer is not equal and so there is a current running timer function for this device 0");

                    if (data.timer_id != null && lasttimerFrameCounter != -1)
                    {
                        console.PrintError("timer is not equal and so there is a current running timer function for this device 1");
                        timer.ClearTimer(data.timer_id);

                        if (storage.IsSetGlobal("systemid_" + sys_id + "_salvage"))
                        {
                            systemSalvageableObjects = storage.GetGlobal("systemid_" + sys_id + "_salvage");

                            if (systemSalvageableObjects != null)
                            {
                                if (systemSalvageableObjects.length > 0)
                                {
                                    objectNDist = [];

                                    var coordsShip = ship.GetCoordinates(args.ship_id);

                                    for (var s = 0; s < systemSalvageableObjects.length; s++)
                                    {
                                        var coordsObject = game.GetObjectCoordinates(sys_id, systemSalvageableObjects[s].id);
                                        var objectDist = SC_Utilities.npcCheckDistance(coordsShip, coordsObject);

                                        var someData = { id: systemSalvageableObjects[s].id, dist: objectDist, coordsX: coordsObject.x, coordsY: coordsObject.y };
                                        objectNDist.push(someData);
                                    }


                                    objectNDist.sort(function (a, b)
                                    {
                                        var adist = a.dist;
                                        var bdist = b.dist;
                                        //var acdam = a.damage;
                                        //var bcdam = b.damage;

                                        if (adist < bdist) {
                                            return -1;
                                        }
                                        else if (adist > bdist) {
                                            return 1;
                                        }
                                        else {
                                            return 0;
                                        }
                                    });

                                    if (objectNDist[0].dist < args.maxDistance)
                                    {
                                        lasttimerFrameCounter = -1;
                                        lasttimerFrameCounterSwtch = 0;
                                        deactivationcounter = 0;
                                        mainFrameCounter = 1;

                                        timer.SetTimer(0.15, "CustomOnFrame", { ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, duration: args.duration, target_id: objectNDist[0].id, frame: 0, countdownTimer: args.countdownTimer, force: args.force, techLevel: args.techLevel, isInverted: args.isInverted, manualOrAuto: args.manualOrAuto, maxDistance: args.maxDistance }, 0); //
                                        //visual.DeviceActivateEffectOnObject(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, objectNDist[0].id);
                                        visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", 0.25, objectNDist[0].coordsX, objectNDist[0].coordsY);

                                        game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + objectNDist[0].id, "special/mining_start.ogg", 0, false);
                                        game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id + "_" + objectNDist[0].id, "special/mining_process.ogg", 0.1, true);


                                        var dirX = coordsShip.x - objectNDist[0].coordsX;
                                        var dirY = coordsShip.y - objectNDist[0].coordsY;


                                        var someOtherMAG = Math.sqrt(((coordsShip.x - objectNDist[0].coordsX) * (coordsShip.x - objectNDist[0].coordsX)) + ((coordsShip.y - objectNDist[0].coordsY) * (coordsShip.y - objectNDist[0].coordsY)));

                                        var dirNPCToPointInFrontOfWaypointX = ((dirX) / someOtherMAG) * objectNDist[0].dist * 0.00001;
                                        var dirNPCToPointInFrontOfWaypointY = ((dirY) / someOtherMAG) * objectNDist[0].dist * 0.00001;

                                        game.ApplyForceToObject(objectNDist[0].id, dirNPCToPointInFrontOfWaypointX, dirNPCToPointInFrontOfWaypointY);

                                        lastObjectCoords = coordsObject;


                                        //storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: 1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: null });
                                        timer.ClearTimer(args.timer_id);
                                    }
                                    else
                                    {
                                        game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "space object too far");
                                    }

                                    lastShipCoords = coordsShip;
                                    lastObjectCoords = { x: objectNDist[0].coordsX, y: objectNDist[0].coordsY };
                                }
                                else
                                {
                                    console.PrintError("there is no salvage item in range 1");
                                }
                            }
                            else
                            {
                                console.PrintError("there is no salvage item in range 0");
                            }
                        }
                    }

                }
            }
        }
        else
        {
            deactivationcounter = 0;
            console.PrintError("null 0");
            timer.ClearTimer(args.timer_id);
        }
    }*/







    /*
    var counterFrame = 0;//SC_Salvage_Object_Timer.GetCounterFrame();


    var data = storage.GetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id);

    if (data != null)
    {
        if (data.swtch == 1)
        {
            var counterDevice = SC_Salvage_Object_Timer.GetCounterDevice();

            //console.PrintError("counter " + counterDevice);

            //args.custom_parameters.countdownTimer

            if (counterDevice > args.countdownTimer)
            {
                console.PrintError("test3");
                game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
                timer.ClearTimer(args.timer_id);

                SC_Salvage_Object_Timer.ResetCounterDevice();

                if (args.manualOrAuto == 1)
                {
                    timer.SetTimer(0.15, "CustomOnFrame", { ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, duration: args.duration, target_id: args.target_id, frame: 0, countdownTimer: args.countdownTimer, force: args.force, techLevel: args.techLevel, isInverted: args.isInverted, manualOrAuto: args.manualOrAuto, maxDistance: args.maxDistance }, 0); //
                }
            }
            else {
                if (args == null || args.ship_id == null || args.target_id == null || args.slot_id == null || args.device_id == null) {
                    if (args == null) {

                        console.PrintError("null args");
                    }
                    if (args.ship_id == null) {

                        console.PrintError("null ship_id");
                    }
                    if (args.target_id == null) {

                        console.PrintError("null target_id");
                    }
                    if (args.slot_id == null) {

                        console.PrintError("null slot_id");
                    }
                    if (args.device_id == null) {

                        console.PrintError("null device_id");
                    }
                }

                var coordsShip = ship.GetCoordinates(args.ship_id);
                var coordsObject = game.GetObjectCoordinates(sys_id, args.target_id);
                var objectDist = SC_Utilities.npcCheckDistance(coordsShip, coordsObject);

                //console.PrintError("dist: " + objectDist);
                if (lastShipCoords != null && lastObjectCoords != null) {
                    var coordsShipCurx = Math.round(coordsShip.x * 10) * 0.1;
                    var coordsShipCury = Math.round(coordsShip.y * 10) * 0.1;
                    var coordsShipLastx = Math.round(lastShipCoords.x * 10) * 0.1;
                    var coordsShipLasty = Math.round(lastShipCoords.y * 10) * 0.1;

                    var coordsCurx = Math.round(coordsObject.x * 10) * 0.1;
                    var coordsCury = Math.round(coordsObject.y * 10) * 0.1;
                    var coordsLastx = Math.round(lastObjectCoords.x * 10) * 0.1;
                    var coordsLasty = Math.round(lastObjectCoords.y * 10) * 0.1;

                    if (coordsShipCurx != coordsShipLastx || coordsShipCury != coordsShipLasty || coordsCurx != coordsLastx || coordsCury != coordsLasty) {
                        if (objectDist < args.maxDistance) {
                            game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                            visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");

                            storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: 1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: args.timer_id, timerFrameCounter: timerFrameCounter });

                            visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, coordsObject.x, coordsObject.y);

                            game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + args.target_id, "special/mining_start.ogg", 0, false);
                            game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id, "special/mining_process.ogg", 0.1, true);

                            var dirX = coordsShip.x - coordsCurx;
                            var dirY = coordsShip.y - coordsCury;

                            var someOtherMAG = Math.sqrt(((coordsShip.x - coordsCurx) * (coordsShip.x - coordsCurx)) + ((coordsShip.y - coordsCury) * (coordsShip.y - coordsCury)));

                            var dirNPCToPointInFrontOfWaypointX = ((dirX) / someOtherMAG) * objectDist * 0.00001;
                            var dirNPCToPointInFrontOfWaypointY = ((dirY) / someOtherMAG) * objectDist * 0.00001;

                            game.ApplyForceToObject(args.target_id, dirNPCToPointInFrontOfWaypointX, dirNPCToPointInFrontOfWaypointY);
                        }
                        else {
                            game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                            visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
                            game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "space object too far");
                            timer.ClearTimer(args.timer_id);
                        }

                        /*game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "You cannot move the ship when salvaging.");
                        game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                        visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
                        timer.ClearTimer(args.timer_id);
                    }
                    else
                    {
                        var coordsCurx = Math.round(coordsObject.x * 10) * 0.1;
                        var coordsCury = Math.round(coordsObject.y * 10) * 0.1;
                        var coordsLastx = Math.round(lastObjectCoords.x * 10) * 0.1;
                        var coordsLasty = Math.round(lastObjectCoords.y * 10) * 0.1;

                        if (coordsCurx != coordsLastx || coordsCury != coordsLasty) {
                            /*game.SendNotificationError(game.GetShipOwner(args.ship_id), "Salvaging failed", "space object too far");
                            game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                            visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
                            timer.ClearTimer(args.timer_id);
                            game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                            visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");

                            storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: 1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: args.timer_id, timerFrameCounter: timerFrameCounter });

                            visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, coordsObject.x, coordsObject.y);
                            game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + args.target_id, "special/mining_start.ogg", 0, false);
                            game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id, "special/mining_process.ogg", 0.1, true);

                            var dirX = coordsShip.x - coordsCurx;
                            var dirY = coordsShip.y - coordsCury;


                            var someOtherMAG = Math.sqrt(((coordsShip.x - coordsCurx) * (coordsShip.x - coordsCurx)) + ((coordsShip.y - coordsCury) * (coordsShip.y - coordsCury)));

                            var dirNPCToPointInFrontOfWaypointX = ((dirX) / someOtherMAG) * objectDist * 0.00001;
                            var dirNPCToPointInFrontOfWaypointY = ((dirY) / someOtherMAG) * objectDist * 0.00001;

                            game.ApplyForceToObject(args.target_id, dirNPCToPointInFrontOfWaypointX, dirNPCToPointInFrontOfWaypointY);
                        }
                        else {
                            if (objectDist < args.maxDistance) {
                                game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                                visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");

                                storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: 1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: args.timer_id, timerFrameCounter: timerFrameCounter  });

                                visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, coordsObject.x, coordsObject.y);

                                game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + args.target_id, "special/mining_start.ogg", 0, false);
                                game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id, "special/mining_process.ogg", 0.1, true);

                                var dirX = coordsShip.x - coordsCurx;
                                var dirY = coordsShip.y - coordsCury;

                                var someOtherMAG = Math.sqrt(((coordsShip.x - coordsCurx) * (coordsShip.x - coordsCurx)) + ((coordsShip.y - coordsCury) * (coordsShip.y - coordsCury)));

                                var dirNPCToPointInFrontOfWaypointX = ((dirX) / someOtherMAG) * objectDist * 0.00001;
                                var dirNPCToPointInFrontOfWaypointY = ((dirY) / someOtherMAG) * objectDist * 0.00001;

                                game.ApplyForceToObject(args.target_id, dirNPCToPointInFrontOfWaypointX, dirNPCToPointInFrontOfWaypointY);
                            }
                            else {
                                timerFrameCounter = 0;
                                console.PrintError("> args.maxDistance");
                                game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
                                visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
                                timer.ClearTimer(args.timer_id);
                                SC_Salvage_Object_Timer.ResetCounterDevice();
                                //storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, null);
                                storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: -1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: args.timer_id, timerFrameCounter: timerFrameCounter  });
                            }
                        }
                            //visual.DeviceActivateEffectOnObject(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, args.target_id);
                            //visual.DeviceActivateEffectOnTarget(args.ship_id, args.device_id, "sc_visual_effect_salvaging", args.duration, coordsObject.x, coordsObject.y);
                            //game.ShipPlaySound(args.ship_id, "mining_start_" + args.slot_id + "_" + args.target_id, "special/mining_start.ogg", 0, false);
                            //game.ShipPlaySound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id, "special/mining_process.ogg", 0.1, true);

                    }
                }
            }

            lastShipCoords = coordsShip;
            lastObjectCoords = coordsObject;

        }
        else //if (data.swtch == 2)
        {
            timerFrameCounter = 0;
            //storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: 1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: args.timer_id });

            console.PrintError("data.swtch == 2");
            game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
            visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
            timer.ClearTimer(args.timer_id);
            SC_Salvage_Object_Timer.ResetCounterDevice();
            //storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, null);
            storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: -1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: args.timer_id, timerFrameCounter: timerFrameCounter });
        }
    }
    else {
        //storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: 1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: args.timer_id });
        timerFrameCounter = 0;
        console.PrintError("data == null");
        game.ShipStopSound(args.ship_id, "mining_process_" + args.slot_id + "_" + args.target_id);
        visual.DeviceDeactivateEffect(args.ship_id, args.device_id, "sc_visual_effect_salvaging");
        timer.ClearTimer(args.timer_id);
        SC_Salvage_Object_Timer.ResetCounterDevice();
        storage.SetGlobal("systemid_" + sys_id + "_ship_" + args.ship_id + "_salvage_device_" + args.device_id, { swtch: -1, ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, timer_id: args.timer_id, timerFrameCounter: timerFrameCounter  });
    }*/





    /*if (objectNDist == null || objectNDist.length == 0 || args == null || args.ship_id == null)
    {
        console.PrintError("test00");
        timer.ClearTimer(args.timer_id);

        if (storage.IsSetGlobal("systemid_" + sys_id + "_salvage"))
        {
            systemSalvageableObjects = storage.GetGlobal("systemid_" + sys_id + "_salvage");

            if (systemSalvageableObjects != null)
            {
                if (systemSalvageableObjects.length > 0)
                {
                    objectNDist = [];
                    var coordsShip = ship.GetCoordinates(args.ship_id);
                    for (var s = 0; s < systemSalvageableObjects.length; s++) {
                        var coordsObject = game.GetObjectCoordinates(sys_id, systemSalvageableObjects[s].id);
                        var objectDist = SC_Utilities.npcCheckDistance(coordsShip, coordsObject);

                        var someData = { id: systemSalvageableObjects[s].id, dist: objectDist, coordsX: coordsObject.x, coordsY: coordsObject.y };
                        objectNDist.push(someData);
                    }

                    lastShipCoords = coordsShip;

                    objectNDist.sort(function (a, b) {
                        var adist = a.dist;
                        var bdist = b.dist;
                        //var acdam = a.damage;
                        //var bcdam = b.damage;

                        if (adist < bdist) {
                            return -1;
                        }
                        else if (adist > bdist) {
                            return 1;
                        }
                        else {
                            return 0;
                        }
                    });
                }
            }
        }
        else
        {
            console.PrintError("there is no salvage item in range 1");
        }
    }
    else
    {
        
        //counterFrame = 0;
        //args.frame = -1;

    }
    //counterFrame++;

    SC_Salvage_Object_Timer.AddCounterFrame();
    SC_Salvage_Object_Timer.AddCounterDevice();
    timerFrameCounter++;*/
    //timer.AddOrUpdate("CustomOnFrame", "swtch", 0);

    //timer.AddOrUpdate(0.25, "CustomOnFrame", { ship_id: args.ship_id, device_id: args.device_id, slot_id: args.slot_id, duration: args.duration, target_id: args.target_id, frame: args.frame++ }, 0); //
    //timer.AddOrUpdate("guard_spawn_timer", 60, "OnGuardSpawn", {}, 0);