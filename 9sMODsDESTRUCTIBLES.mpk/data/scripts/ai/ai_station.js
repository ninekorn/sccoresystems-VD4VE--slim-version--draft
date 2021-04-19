using(npc);
using(console);
using(player);
using(ship);
using(game);
using(generator);
using(storage);
using(station);
using(items);
using(timer);
using(server);

include(SystemsPresets.js);
include(CSGen.js);
include(SC_Utilities.js);

function OnAIInited() {
    npc.SetDecisionsPerSecond(SHIP_ID, 20);

    npc.SetBehavior(SHIP_ID, "avoid_asteroids", false);
    npc.SetBehavior(SHIP_ID, "avoid_ships", false);
    npc.SetBehavior(SHIP_ID, "avoid_bases", false);
    npc.SetBehavior(SHIP_ID, "avoid_debris", false);
}

var someSwitchFront = true;
var someSwitchBack = true;

var someOtherSwitchFront = false;
var someOtherSwitchBack = false;

var mainFrameCounter = 0;

var mainFrameCounterFront = 0;
var mainFrameCounterBack = 0;


var someOtherSwitch = 0;
var correctSystem;



var lastJumpGate;

var lastSpawnedJumpgateSameSystemFront;
var lastSpawnedJumpgateSystemInteriorFront;
var lastSpawnedJumpgateSameSystemBack;
var lastSpawnedJumpgateSystemInteriorBack;


var angleMainFrameCounter = 0;

var playerInSystem;

var correctAngleFront;
var correctAngleBack;

var stationLoadingCounter = 0;
var stationLoadingCounterMAX = 5;

var planetJumpLoadingCounter = 0;
var planetJumpLoadingCounterMAX = 5;

var SC_anglesQuarterNumber = 360 * 10;


var offset = 21;
var offsetY = -13;


//ship.EnteringJumpgate(follow_ship);
//ship.SetShipAsArrivedFromJumpgate(id, jag_ids[0]);


var spawningJumpgateSwtch = 0;
var spawningPlanetJumpSwtch = 0;

var jumpgateArray = [];

var arrayOfPlayerShips = [];
var player_id;
var sys_idNPC;

var mainFrameCounterPlanetJump = 0;

var counterForResetRoof = 0;
var counterForResetRoofMAX = 40;

var playerInRange = -1;
var frameJumpGateSwitchTestCounter = 0;
var frameJumpGateSwitchTestCounterMAX = 10;
var frameJumpGateSwitchTestCounterSwtc = 0;


function Decision(args) {

    //player_id = npc.GetTag(nextCommandToDispatch.id, "ownerPlayerShipId");
    //playerName = game.GetShipOwner(player_id);
    //sys_idPlayer = ship.GetSystemID(player_id);
    //npcCoord = ship.GetCoordinates(nextCommandToDispatch.id);

    sys_idNPC = npc.GetCurrentSystemID(SHIP_ID);
    var npcCoord = ship.GetCoordinates(SHIP_ID);
    //var systemBases = game.GetSystemBases(sys_idNPC);
    //var jumpgates = game.GetSystemJumpgates(sys_idNPC);

    //var name = game.GetShipOwner(123142);
    var whatStationType = npc.GetTag(SHIP_ID, "class");

    if (spawningJumpgateSwtch == 0) {

        //if (whatStationType == "stationEXT") {
        if (stationLoadingCounter >= stationLoadingCounterMAX) //>=2
        {
            //console.PrintError("station can now warp you.");
            if (mainFrameCounter >= 2) //>=2
            {
                var ships = game.GetSystemShips(sys_idNPC);

                if (ships != null) {
                    for (var i = 0; i < ships.length; i++) {
                        if (ships[i] != null) {
                            var playerSysID = ship.GetSystemID(ships[i]);

                            if (playerSysID == sys_idNPC) {
                                //var isPlayer = !ship.IsNpc(ships[i]);
                                var isPlayer = game.IsShipPlayerControlled(ships[i]);
                                if (isPlayer) {
                                    player_id = ships[i];
                                    //var dist = ship.GetDistanceToObj(SHIP_ID, ships[i]);
                                    arrayOfPlayerShips.push({ id: ships[i] });

                                    var inf = generator.GetSystemByID(sys_idNPC);
                                    if (inf.name == "player base")
                                    {

                                        //con
                                        //SPAWNING STATION INTERIOR JUMPGATES
                                        //SPAWNING STATION INTERIOR JUMPGATES
                                        //SPAWNING STATION INTERIOR JUMPGATES
                                        var stationDataINT = storage.GetGlobal("stationINT" + sys_idNPC);
                                        var getSomeIndex = stationDataINT.xml_id.substring(11, stationDataINT.xml_id.length); //outpost_01_0
                                        var parsedAngle = parseInt(getSomeIndex);


                                        var degToRad = (parsedAngle * Math.PI / 180);
                                        //SPAWNING STATION INTERIOR JUMPGATES
                                        //SPAWNING STATION INTERIOR JUMPGATES
                                        //SPAWNING STATION INTERIOR JUMPGATES
                                        var coordsPlayerShip = ship.GetCoordinates(ships[i]);

                                        var distPlayerFront = Math.sqrt(((stationDataINT.x - coordsPlayerShip.x) * (stationDataINT.x - coordsPlayerShip.x)) + ((stationDataINT.y - coordsPlayerShip.y) * (stationDataINT.y - coordsPlayerShip.y)));

                                        if (distPlayerFront <= 50) {

                                            //magPlayer = Math.sqrt(veloPlayer.x * veloPlayer.x + veloPlayer.y * veloPlayer.y);


                                            console.PrintError("is the frame coming here");

                                            //if (correctTag == "stationINT") {

                                            /*var stationDataINT = storage.GetGlobal("stationEXT" + systems[j]);
                                            var coordsStationINT = { x: stationDataINT.x, y: stationDataINT.y };
 
                                            var randomIndexStationOutpost = Math.floor(Math.random() * stationDataINT.xml_id.length);
 
                                            var getSomeIndex = stationDataINT.xml_id.substring(11, stationDataINT.xml_id.length); //outpost_01_0
 
                                            //var index = indexOfStuff(getSomeIndex);
                                            var parsedAngle = parseInt(getSomeIndex);*/



                                            //SPAWNING STATION EXTERIOR JUMPGATES
                                            //SPAWNING STATION EXTERIOR JUMPGATES
                                            //SPAWNING STATION EXTERIOR JUMPGATES
                                            var stationDataEXT = storage.GetGlobal("stationEXT" + stationDataINT.sys_id_link);
                                            //var randomIndexStationOutpost = Math.floor(Math.random() * stationDataINT.xml_id.length);       

                                            var tempCoordINTLEFTX = stationDataINT.x - 37.5;
                                            var tempCoordINTLEFTY = stationDataINT.y - 12.5;
                                            var tempCoordINTLEFT = { x: tempCoordINTLEFTX, y: tempCoordINTLEFTY };

                                            var tempCoordINTRIGHTX = stationDataINT.x + 37.5;
                                            var tempCoordINTRIGHTY = stationDataINT.y - 12.5;
                                            var tempCoordINTRIGHT = { x: tempCoordINTRIGHTX, y: tempCoordINTRIGHTY };


                                            var rotatedCoordINTLEFT = SC_Utilities.RotatePoint(tempCoordINTLEFT, stationDataINT, parsedAngle);
                                            var rotatedCoordINTRIGHT = SC_Utilities.RotatePoint(tempCoordINTRIGHT, stationDataINT, parsedAngle);


                                            var degToRadTWO = ((parsedAngle + 180) * Math.PI / 180);
                                            //var degToRadTWO = (degToRad + 180);
                                            //var angler = 360 - radToDeg;
                                            var jg_id0 = generator.AddJumpgate(
                                                stationDataEXT.sys_id_link,
                                                stationDataINT.sys_id_link,
                                                rotatedCoordINTLEFT.x,
                                                rotatedCoordINTLEFT.y,
                                                degToRadTWO,
                                                "jumpgate_station");

                                            if (degToRad < 0) {
                                                degToRad *= -1;
                                            }
                                            var jg_id1 = generator.AddJumpgate(
                                                stationDataEXT.sys_id_link,
                                                stationDataINT.sys_id_link,
                                                rotatedCoordINTRIGHT.x,
                                                rotatedCoordINTRIGHT.y,
                                                degToRad,
                                                "jumpgate_station");

                                            //jumpgateArray.push({ id: jg_id0, linkid: jg_id1, sys_id: stationDataEXT.sys_id_link, syslink: stationDataINT.sys_id_link, coords: rotatedCoordINTLEFT, rots: degToRadTWO, index: 0 });
                                            //jumpgateArray.push({ id: jg_id1, linkid: jg_id0, sys_id: stationDataEXT.sys_id_link, syslink: stationDataINT.sys_id_link, coords: rotatedCoordINTRIGHT, rots: degToRad, index: 1  });




                                            //SPAWNING STATION INTERIOR JUMPGATES
                                            //SPAWNING STATION INTERIOR JUMPGATES
                                            //SPAWNING STATION INTERIOR JUMPGATES
                                            //var stationDataINT = storage.GetGlobal("stationINT" + stationDataINT.sys_id_link);
                                            //var randomIndexStationOutpost = Math.floor(Math.random() * stationDataINT.xml_id.length);

                                            //var getSomeIndex = stationDataINT.xml_id.substring(11, stationDataINT.xml_id.length); //outpost_01_0
                                            //var parsedAngle = parseInt(getSomeIndex);
                                            //var degToRad = (parsedAngle * Math.PI / 180);

                                            var tempCoordINTLEFTX = stationDataINT.x - 15;
                                            var tempCoordINTLEFTY = stationDataINT.y - 12.5;
                                            var tempCoordINTLEFT = { x: tempCoordINTLEFTX, y: tempCoordINTLEFTY };

                                            var tempCoordINTRIGHTX = stationDataINT.x + 15;
                                            var tempCoordINTRIGHTY = stationDataINT.y - 12.5;
                                            var tempCoordINTRIGHT = { x: tempCoordINTRIGHTX, y: tempCoordINTRIGHTY };


                                            var rotatedCoordINTLEFT = SC_Utilities.RotatePoint(tempCoordINTLEFT, stationDataINT, parsedAngle);
                                            var rotatedCoordINTRIGHT = SC_Utilities.RotatePoint(tempCoordINTRIGHT, stationDataINT, parsedAngle);

                                            var degToRadTWO = ((parsedAngle + 180) * Math.PI / 180);
                                            //var angler = 360 - radToDeg;
                                            var jg_id0 = generator.AddJumpgate(
                                                stationDataINT.sys_id_link,
                                                stationDataEXT.sys_id_link,
                                                rotatedCoordINTLEFT.x,
                                                rotatedCoordINTLEFT.y,
                                                degToRad,
                                                "jumpgate_station");

                                            if (degToRadTWO < 0) {
                                                degToRadTWO *= -1;
                                            }
                                            var jg_id1 = generator.AddJumpgate(
                                                stationDataINT.sys_id_link,
                                                stationDataEXT.sys_id_link,
                                                rotatedCoordINTRIGHT.x,
                                                rotatedCoordINTRIGHT.y,
                                                degToRadTWO,
                                                "jumpgate_station");



                                            //jumpgateArray.push({ id: jg_id0, linkid: jg_id1, sys_id: stationDataINT.sys_id_link, syslink: stationDataEXT.sys_id_link, coords: rotatedCoordINTLEFT, rots: degToRad , index: 2 });
                                            //jumpgateArray.push({ id: jg_id1, linkid: jg_id0, sys_id: stationDataINT.sys_id_link, syslink: stationDataEXT.sys_id_link, coords: rotatedCoordINTRIGHT, rots: degToRadTWO, index: 3  });


                                            //SPAWNING STATION INTERIOR JUMPGATES
                                            //SPAWNING STATION INTERIOR JUMPGATES
                                            //SPAWNING STATION INTERIOR JUMPGATES






                                            spawningJumpgateSwtch = 1;
                                            mainFrameCounterFront = 0;

                                        }
                                        else if (inf.name == "player base") {
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                mainFrameCounter = 0;
            }
            //stationLoadingCounter = 0;
        }
    }






























    //SPAWNING PLANET JUMPGATES
    //SPAWNING PLANET JUMPGATES
    //SPAWNING PLANET JUMPGATES
    //SPAWNING PLANETS JUMP SWTCH
    if (spawningPlanetJumpSwtch == 0) {

        //if (whatStationType == "stationEXT") {
        if (planetJumpLoadingCounter >= planetJumpLoadingCounterMAX) //>=2
        {
            //console.PrintError("planet can now warp you.");
            if (mainFrameCounterPlanetJump >= 2) //>=2
            {
                var ships = game.GetSystemShips(sys_idNPC);

                if (ships != null) {
                    for (var i = 0; i < ships.length; i++) {
                        if (ships[i] != null) {
                            var playerSysID = ship.GetSystemID(ships[i]);

                            if (playerSysID == sys_idNPC) {
                                //var isPlayer = !ship.IsNpc(ships[i]);
                                var isPlayer = game.IsShipPlayerControlled(ships[i]);
                                if (isPlayer) {
                                    player_id = ships[i];
                                    //TO READD
                                    //arrayOfPlayerShips.push({ id: ships[i] });
                                    //TO READD

                                    //storage.SetGlobal("planet earth" + sys_id2, { id: null, x: null, y: null, sys_id: sys_id2, sys_id_link: sys_id1, xml_id: "xml_station_exterior", DegAngle: parsedAngle, class: tagClass });
                                    //var planetEarthGlobalVariable = storage.GetGlobal("planet earth" + sys_id2);
                                    //var curinf = generator.GetSystemByID(sys_id2);
                                    //var system_vec = { x: curinf.coord_x + 50, y: curinf.coord_y };
                                    //storage.SetGlobal("planet earth INT" + sys_id2, { id: null, x: system_vec.x, y: system_vec.y, sys_id: sys_id0, sys_id_link: sys_id1, xml_id: "planet_earth_night", DegAngle: parsedAngle, class: tagClass });
                                    //storage.SetGlobal("planet earth EXT" + sys_id2, { id: null, x: system_vec.x, y: system_vec.y, sys_id: sys_id1, sys_id_link: sys_id0, xml_id: "planet_earth_night", DegAngle: parsedAngle, class: tagClass });

                                    //SPAWNING STATION INTERIOR JUMPGATES
                                    //SPAWNING STATION INTERIOR JUMPGATES
                                    //SPAWNING STATION INTERIOR JUMPGATES
                                    //var stationDataINT = storage.GetGlobal("stationINT" + sys_idNPC);

                                    //magPlayer = Math.sqrt(veloPlayer.x * veloPlayer.x + veloPlayer.y * veloPlayer.y);

                                    var systems = generator.GetAllSystems();

                                    for (var j = 0; j < systems.length; j++) {
                                        var inf = generator.GetSystemByID(systems[j]);

                                        if (inf.name == "Solar System") {
                                            //console.PrintError("000 is the frame coming here");

                                            //if (correctTag == "stationINT") {

                                            /*var stationDataINT = storage.GetGlobal("stationEXT" + systems[j]);
                                            var coordsStationINT = { x: stationDataINT.x, y: stationDataINT.y };
 
                                            var randomIndexStationOutpost = Math.floor(Math.random() * stationDataINT.xml_id.length);
 
                                            var getSomeIndex = stationDataINT.xml_id.substring(11, stationDataINT.xml_id.length); //outpost_01_0
 
                                            //var index = indexOfStuff(getSomeIndex);
                                            var parsedAngle = parseInt(getSomeIndex);*/



                                            //SPAWNING PLANET JUMPGATES
                                            //SPAWNING PLANET JUMPGATES
                                           //SPAWNING PLANET JUMPGATES
                                            var planetEarthGlobalVariableINT = storage.GetGlobal("planet earth EXT" + systems[j]);
                                            //var planetEarthGlobalVariableEXT = storage.GetGlobal("planet earth EXT" + systems[j]);
                                            //var getSomeIndex = planetEarthGlobalVariableINT.xml_id.substring(11, planetEarthGlobalVariableINT.xml_id.length); //outpost_01_0
                                            //var parsedAngle = parseInt(getSomeIndex);


                                            //var degToRad = (parsedAngle * Math.PI / 180);
                                            var coordsPlayerShip = ship.GetCoordinates(ships[i]);

                                            var distPlayerFront = Math.sqrt(((planetEarthGlobalVariableINT.x - coordsPlayerShip.x) * (planetEarthGlobalVariableINT.x - coordsPlayerShip.x)) + ((planetEarthGlobalVariableINT.y - coordsPlayerShip.y) * (planetEarthGlobalVariableINT.y - coordsPlayerShip.y)));

                                            //console.PrintError("dist to planet: " + distPlayerFront);

                                            if (distPlayerFront <= 50) {
                                                //var stationDataEXT = storage.GetGlobal("stationEXT" + planetEarthGlobalVariableINT.sys_id_link);
                                                //var randomIndexStationOutpost = Math.floor(Math.random() * planetEarthGlobalVariableINT.xml_id.length);       

                                                var tempCoordINTLEFTX = planetEarthGlobalVariableINT.x;
                                                var tempCoordINTLEFTY = planetEarthGlobalVariableINT.y;
                                                var tempCoordINTLEFT = { x: tempCoordINTLEFTX, y: tempCoordINTLEFTY };

                                                var tempCoordINTRIGHTX = planetEarthGlobalVariableINT.x;
                                                var tempCoordINTRIGHTY = planetEarthGlobalVariableINT.y;
                                                var tempCoordINTRIGHT = { x: tempCoordINTRIGHTX, y: tempCoordINTRIGHTY };


                                                var rotatedCoordINTLEFT = SC_Utilities.RotatePoint(tempCoordINTLEFT, planetEarthGlobalVariableINT, 0);
                                                var rotatedCoordINTRIGHT = SC_Utilities.RotatePoint(tempCoordINTRIGHT, planetEarthGlobalVariableINT, 0);


                                                //var degToRadTWO = ((parsedAngle + 180) * Math.PI / 180);
                                                //var degToRadTWO = (degToRad + 180);
                                                //var angler = 360 - radToDeg;
                                                var jg_id0 = generator.AddJumpgate(
                                                    planetEarthGlobalVariableINT.sys_id_link,
                                                    planetEarthGlobalVariableINT.sys_id,
                                                    planetEarthGlobalVariableINT.x,
                                                    planetEarthGlobalVariableINT.y,
                                                    0,
                                                    "jumpgate_planet");

                                                /*if (degToRad < 0) {
                                                    degToRad *= -1;
                                                }
                                                var jg_id1 = generator.AddJumpgate(
                                                    stationDataEXT.sys_id_link,
                                                    planetEarthGlobalVariableINT.sys_id_link,
                                                    rotatedCoordINTRIGHT.x,
                                                    rotatedCoordINTRIGHT.y,
                                                    degToRad,
                                                    "jumpgate_station");*/

                                                //jumpgateArray.push({ id: jg_id0, linkid: jg_id1, sys_id: stationDataEXT.sys_id_link, syslink: planetEarthGlobalVariableINT.sys_id_link, coords: rotatedCoordINTLEFT, rots: degToRadTWO, index: 0 });
                                                //jumpgateArray.push({ id: jg_id1, linkid: jg_id0, sys_id: stationDataEXT.sys_id_link, syslink: planetEarthGlobalVariableINT.sys_id_link, coords: rotatedCoordINTRIGHT, rots: degToRad, index: 1  });


                                                //SPAWNING STATION INTERIOR JUMPGATES
                                                //SPAWNING STATION INTERIOR JUMPGATES
                                                //SPAWNING STATION INTERIOR JUMPGATES
                                                //var planetEarthGlobalVariableINT = storage.GetGlobal("stationINT" + planetEarthGlobalVariableINT.sys_id_link);
                                                //var randomIndexStationOutpost = Math.floor(Math.random() * planetEarthGlobalVariableINT.xml_id.length);

                                                //var getSomeIndex = planetEarthGlobalVariableINT.xml_id.substring(11, planetEarthGlobalVariableINT.xml_id.length); //outpost_01_0
                                                //var parsedAngle = parseInt(getSomeIndex);
                                                //var degToRad = (parsedAngle * Math.PI / 180);

                                                var tempCoordINTLEFTX = planetEarthGlobalVariableINT.x;
                                                var tempCoordINTLEFTY = planetEarthGlobalVariableINT.y;
                                                var tempCoordINTLEFT = { x: tempCoordINTLEFTX, y: tempCoordINTLEFTY };

                                                var tempCoordINTRIGHTX = planetEarthGlobalVariableINT.x;
                                                var tempCoordINTRIGHTY = planetEarthGlobalVariableINT.y;
                                                var tempCoordINTRIGHT = { x: tempCoordINTRIGHTX, y: tempCoordINTRIGHTY };


                                                var rotatedCoordINTLEFT = SC_Utilities.RotatePoint(tempCoordINTLEFT, planetEarthGlobalVariableINT, 0);
                                                var rotatedCoordINTRIGHT = SC_Utilities.RotatePoint(tempCoordINTRIGHT, planetEarthGlobalVariableINT, 0);

                                                //var degToRadTWO = ((parsedAngle + 180) * Math.PI / 180);
                                                //var angler = 360 - radToDeg;
                                                var jg_id0 = generator.AddJumpgate(
                                                    planetEarthGlobalVariableINT.sys_id,
                                                    planetEarthGlobalVariableINT.sys_id_link,
                                                    planetEarthGlobalVariableINT.x,
                                                    planetEarthGlobalVariableINT.y,
                                                    0,
                                                    "jumpgate_planet");

                                                /*if (degToRadTWO < 0) {
                                                    degToRadTWO *= -1;
                                                }
                                                var jg_id1 = generator.AddJumpgate(
                                                    stationDataINT.sys_id_link,
                                                    stationDataEXT.sys_id_link,
                                                    rotatedCoordINTRIGHT.x,
                                                    rotatedCoordINTRIGHT.y,
                                                    degToRadTWO,
                                                    "jumpgate_station");*/



                                                //jumpgateArray.push({ id: jg_id0, linkid: jg_id1, sys_id: stationDataINT.sys_id_link, syslink: stationDataEXT.sys_id_link, coords: rotatedCoordINTLEFT, rots: degToRad , index: 2 });
                                                //jumpgateArray.push({ id: jg_id1, linkid: jg_id0, sys_id: stationDataINT.sys_id_link, syslink: stationDataEXT.sys_id_link, coords: rotatedCoordINTRIGHT, rots: degToRadTWO, index: 3  });


                                                //SPAWNING STATION INTERIOR JUMPGATES
                                                //SPAWNING STATION INTERIOR JUMPGATES
                                                //SPAWNING STATION INTERIOR JUMPGATES


                                                //mainFrameCounterFront = 0;
                                                spawningPlanetJumpSwtch = 1;
                                                mainFrameCounterPlanetJump = 0;


                                                //lastSpawnedJumpgateSameSystemFront = jg_id0;

                                                //console.PrintError("TEST");     

                                                //var stationShips = game.GetSystemShips(systems[j]);
                                                //console.PrintError(stationShips.length + " stationShips.length ");     

                                                //var whatStationType = npc.GetTag(stationData.id, "class");

                                                //var correctTag = whatStationType;

                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            //planetJumpLoadingCounter = 0;
        }
        else {
            if (planetJumpLoadingCounter >= planetJumpLoadingCounterMAX) //>=2
            {
                //console.PrintError("planet can now warp you.");
                if (mainFrameCounterPlanetJump >= 2) //>=2
                {
                    var ships = game.GetSystemShips(sys_idNPC);

                    if (ships != null) {
                        for (var i = 0; i < ships.length; i++) {
                            if (ships[i] != null) {
                                var playerSysID = ship.GetSystemID(ships[i]);

                                if (playerSysID == sys_idNPC) {
                                    //var isPlayer = !ship.IsNpc(ships[i]);
                                    var isPlayer = game.IsShipPlayerControlled(ships[i]);
                                    if (isPlayer) {
                                        player_id = ships[i];
                                        //TO READD
                                        //arrayOfPlayerShips.push({ id: ships[i] });
                                        //TO READD

                                        //storage.SetGlobal("planet earth" + sys_id2, { id: null, x: null, y: null, sys_id: sys_id2, sys_id_link: sys_id1, xml_id: "xml_station_exterior", DegAngle: parsedAngle, class: tagClass });
                                        //var planetEarthGlobalVariable = storage.GetGlobal("planet earth" + sys_id2);
                                        //var curinf = generator.GetSystemByID(sys_id2);
                                        //var system_vec = { x: curinf.coord_x + 50, y: curinf.coord_y };
                                        //storage.SetGlobal("planet earth INT" + sys_id2, { id: null, x: system_vec.x, y: system_vec.y, sys_id: sys_id0, sys_id_link: sys_id1, xml_id: "planet_earth_night", DegAngle: parsedAngle, class: tagClass });
                                        //storage.SetGlobal("planet earth EXT" + sys_id2, { id: null, x: system_vec.x, y: system_vec.y, sys_id: sys_id1, sys_id_link: sys_id0, xml_id: "planet_earth_night", DegAngle: parsedAngle, class: tagClass });

                                        //SPAWNING STATION INTERIOR JUMPGATES
                                        //SPAWNING STATION INTERIOR JUMPGATES
                                        //SPAWNING STATION INTERIOR JUMPGATES
                                        //var stationDataINT = storage.GetGlobal("stationINT" + sys_idNPC);

                                        //magPlayer = Math.sqrt(veloPlayer.x * veloPlayer.x + veloPlayer.y * veloPlayer.y);

                                        var systems = generator.GetAllSystems();

                                        for (var j = 0; j < systems.length; j++) {
                                            var inf = generator.GetSystemByID(systems[j]);

                                            if (inf.name == "Solar System") {
                                                //console.PrintError("000 is the frame coming here");

                                                //if (correctTag == "stationINT") {

                                                /*var stationDataINT = storage.GetGlobal("stationEXT" + systems[j]);
                                                var coordsStationINT = { x: stationDataINT.x, y: stationDataINT.y };
     
                                                var randomIndexStationOutpost = Math.floor(Math.random() * stationDataINT.xml_id.length);
     
                                                var getSomeIndex = stationDataINT.xml_id.substring(11, stationDataINT.xml_id.length); //outpost_01_0
     
                                                //var index = indexOfStuff(getSomeIndex);
                                                var parsedAngle = parseInt(getSomeIndex);*/



                                                //SPAWNING STATION EXTERIOR JUMPGATES
                                                //SPAWNING STATION EXTERIOR JUMPGATES
                                                //SPAWNING STATION EXTERIOR JUMPGATES
                                                var planetEarthGlobalVariableINT = storage.GetGlobal("planet earth EXT" + systems[j]);
                                                //var planetEarthGlobalVariableEXT = storage.GetGlobal("planet earth EXT" + systems[j]);
                                                //var getSomeIndex = planetEarthGlobalVariableINT.xml_id.substring(11, planetEarthGlobalVariableINT.xml_id.length); //outpost_01_0
                                                //var parsedAngle = parseInt(getSomeIndex);


                                                //var degToRad = (parsedAngle * Math.PI / 180);
                                                //SPAWNING STATION INTERIOR JUMPGATES
                                                //SPAWNING STATION INTERIOR JUMPGATES
                                                //SPAWNING STATION INTERIOR JUMPGATES
                                                var coordsPlayerShip = ship.GetCoordinates(ships[i]);

                                                var distPlayerFront = Math.sqrt(((planetEarthGlobalVariableINT.x - coordsPlayerShip.x) * (planetEarthGlobalVariableINT.x - coordsPlayerShip.x)) + ((planetEarthGlobalVariableINT.y - coordsPlayerShip.y) * (planetEarthGlobalVariableINT.y - coordsPlayerShip.y)));


                                                game.SendNotification("ninekorn", "Server notification", "distance to planet Earth " + distPlayerFront);
                                                //console.PrintError("dist to planet: " + distPlayerFront);

                                                /*if (distPlayerFront <= 50)
                                                {

                                                }*/
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    mainFrameCounterPlanetJump++;
    mainFrameCounter++;
    mainFrameCounterFront++;
    mainFrameCounterBack++;
    angleMainFrameCounter++;
    stationLoadingCounter++;
    planetJumpLoadingCounter++;
}
