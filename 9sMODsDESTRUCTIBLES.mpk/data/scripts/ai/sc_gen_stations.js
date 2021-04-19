include(IMCache.js);
include(SC_Utilities.js);


using(console);
using(game);
using(npc);
using(generator);
using(items);
using(relations);
using(station);
using(ship);
using(storage);



function OnAIInited() {
    //set update interval
    npc.SetDecisionsPerSecond(SHIP_ID, 1);
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


var player_id;
var playerName;
var sys_idNPC;
var sys_idPlayer;
var npcCoord;

var startOnce = true;
var arrayOfTerrain = [];

var width = 10;
var height = 10;

var planeWidth = 10;
var playerCoord;
var spawnStation = 0;
var frameCounterToSpawnrepairUnit = 0;
var canStartCounter = 0;
var startingStationID;


function Decision(args) {
    //player_id = npc.GetTag(SHIP_ID, "ownerPlayerShipId");
    //playerName = game.GetShipOwner(player_id);
    //sys_idPlayer = ship.GetSystemID(player_id);
    //npcCoord = ship.GetCoordinates(SHIP_ID);
    //playerCoord = ship.GetCoordinates(player_id);


    sys_idNPC = npc.GetCurrentSystemID(SHIP_ID);

    var ships = game.GetSystemPlayersShips(sys_idNPC);

    for (var i = 0; i < ships.length; i++)
    {
        sys_idPlayer = ship.GetSystemID(ships[i]);
        if (startOnce)
        {
            playerCoord = ship.GetCoordinates(ships[i]);

            console.PrintError("playerCoordx: " + playerCoord.x + " playerCoord.y " + playerCoord.y);
            var posX = Math.floor(playerCoord.x);
            var posY = Math.floor(playerCoord.y);


            /*for (var xi = -width; xi < width; xi += 10)
            {
                for (var yi = -height; yi < height; yi += 10)
                {
                    var PoserX = posX;// + xi; //posX + 
                    var PoserY = posY;// + yi; // posY +

                    var coords = { x: PoserX, y: PoserY };
                    var rotation = { X: 0, Y: 0, Z: 0 };
                    var rotationSpeed = { X: 0, Y: 0, Z: 0 };
                    var distance = 0;
                    var scale = 1;

                    if (!generator.AreThereAnyObjects(sys_idPlayer, coords.x, coords.y, 9))
                    {
                        var id = generator.AddNPCShipToSystem("station test", "ai_station", 100, "xml_station_test", sys_idPlayer, coords.x, coords.y, { someTag: "stationTest", class: "stationTest", greeting: "terminal", unique_id: "stationTest", stationID: "player base" });
                    }
                }
            }*/





            var PoserX = posX;// + xi; //posX + 
            var PoserY = posY;// + yi; // posY +

            var coords = { x: PoserX, y: PoserY };
            var rotation = { X: 0, Y: 0, Z: 0 };
            var rotationSpeed = { X: 0, Y: 0, Z: 0 };
            var distance = 0;
            var scale = 1;


            /*if (_frameCounter > 10)
            {
                if (spawnStation == 0) {
                    var systems = generator.GetAllSystems();

                    for (var j = 0; j < systems.length; j++)
                    {
                        var inf = generator.GetSystemByID(systems[j]);
                        if (inf.name == "player base") {

                            var stationData = storage.GetGlobal("stationINT" + systems[j]);

                            //startingStationID = generator.AddNPCShipToSystem("stationINT", "ai_station_interior", 1, "xml_station_interior", sys_idPlayer, coords.x + 50, coords.y, { class: "stationDialog", someTag: "stationINT", greeting: "terminal", stationID: "stationINT" }); //, unique_id: "stationDialog"
                            var id = generator.AddNPCShipToSystem("station test", "ai_station", 100, "xml_station_exterior", sys_idPlayer, coords.x + 20, coords.y, { someTag: "stationEXT", class: "stationEXT", greeting: "terminal", unique_id: "stationTest", stationID: "player base" });

                            //var radToDegAngle = ;//SC_Utilities.DegreeToRadian(angle);
                            ship.SetRotation(id, stationData.DegAngle);

                            console.PrintError("spawned station interior");
                            canStartCounter = 1;
                            spawnStation = 1;
                        }
                    }
                }
            } */       

            var angle = ship.GetRotation(startingStationID);
            var radToDeg = angle * (180.0 / Math.PI);
            var coordsNPC = ship.GetCoordinates(startingStationID);

            //x:-18 y:-3.15
            //x:-8  y:-3.15

            /*var repairMovementZone = { x: -18, y: -3.15 };
            var repairVisualZone = { x: -8, y: -3.15 };

            var repairZoneMoveX = (1 * Math.cos(radToDeg * Math.PI / 180)) + coordsNPC.x;
            var repairZoneMoveY = (1 * Math.sin(radToDeg * Math.PI / 180)) + coordsNPC.y;
            */


            //var repairZoneMoveX = (1 * Math.cos(radToDeg * Math.PI / 180)) + (coordsNPC.x);
            //var repairZoneMoveY = (1 * Math.sin(radToDeg * Math.PI / 180)) + (coordsNPC.y);

            //repairMovementZone.x
            

            /*if (frameCounterToSpawnrepairUnit >= 15)
            {
                var id3 = generator.AddNPCShipToSystem("station repair high", "ai_repair_high", 1, "xml_repair_high", sys_idNPC, repairZoneMoveX, repairZoneMoveY, { class: "stationDialog", someTag: "stationREPAIRHIGH", greeting: "terminal", stationID: startingStationID }); //, unique_id: "stationDialog"

                startOnce = false;
            }*/
















            /*if (!generator.AreThereAnyObjects(sys_idPlayer, coords.x + 5, coords.y, 1))
            {

            }*/

            //var id0 = generator.AddNPCShipToSystem("station exterior", "ai_station", 100, "xml_station_interior", sys_idPlayer, coords.x + 50, coords.y, { class: "stationDialog", someTag: "stationEXT", greeting: "terminal", stationID: "player baseEXT" }); //, unique_id: "stationDialog"



            /*var angle = ship.GetRotation(id0);
            var radToDeg = (angle * (180.0 / Math.PI)) % 360;
            var coordsNPC = ship.GetCoordinates(id0);

            var pointFrontX = (10 * Math.cos(angle)) + coordsNPC.x; //radToDeg * Math.PI / 180
            var pointFrontY = (10 * Math.sin(angle)) + coordsNPC.y; //radToDeg * Math.PI / 180)

            //magPlayer = Math.sqrt(veloPlayer.x * veloPlayer.x + veloPlayer.y * veloPlayer.y);
            var pointBackX = (-10 * Math.cos(angle)) + coordsNPC.x; //radToDeg * Math.PI / 180
            var pointBackY = (-10 * Math.sin(angle)) + coordsNPC.y; // radToDeg * Math.PI / 180

            var distPlayerFront = Math.sqrt(((pointFrontX - coordsNPC.x) * (pointFrontX - coordsNPC.x)) + ((pointFrontY - coordsNPC.y) * (pointFrontY - coordsNPC.y)));
            var distPlayerBack = Math.sqrt(((pointBackX - coordsNPC.x) * (pointBackX - coordsNPC.x)) + ((pointBackY - coordsNPC.y) * (pointBackY - coordsNPC.y)));

            var id2 = generator.AddNPCShipToSystem("station repair low", "ai_station_interior", 100, "xml_repair_low", systems[j], pointFrontX, pointFrontY, { class: "stationDialog", someTag: "stationREPAIRLOW", greeting: "terminal", stationID: "stationREPAIRLOW" }); //, unique_id: "stationDialog"
            var id3 = generator.AddNPCShipToSystem("station repair high", "ai_station_interior", 100, "xml_repair_high", systems[j], pointBackX, pointBackY, { class: "stationDialog", someTag: "stationREPAIRHIGH", greeting: "terminal", stationID: "stationREPAIRHIGH" }); //, unique_id: "stationDialog"
            */




            /*var systems = generator.GetAllSystems();

            for (var j = 0; j < systems.length; j++) {
                var inf = generator.GetSystemByID(systems[j]);

                if (inf.name == "player base")
                {
                    var id1 = generator.AddNPCShipToSystem("stationINT", "ai_station_interior", 100, "xml_station_interior", systems[j], coords.x + 50, coords.y, { class: "stationDialog", someTag: "stationINT", greeting: "terminal", stationID: "stationINT" }); //, unique_id: "stationDialog"

                    var angle = ship.GetRotation(id1);
                    var radToDeg = (angle * (180.0 / Math.PI)) % 360;
                    var coordsNPC = ship.GetCoordinates(id1);

                    //x:-18 y:-3.15
                    //x:-8  y:-3.15

                    var repairMovementZone = { x: -18, y: -3.15 };
                    var repairVisualZone = { x: -8, y: -3.15 };


                    var repairZoneMoveX = (1 * Math.cos(radToDeg * Math.PI / 180)) + (coordsNPC.x);
                    var repairZoneMoveY = (1 * Math.sin(radToDeg * Math.PI / 180)) + (coordsNPC.y);


                    //repairMovementZone.x


                    var id3 = generator.AddNPCShipToSystem("station repair high", "ai_repair_high", 100, "xml_repair_high", systems[j], repairZoneMoveX, repairZoneMoveY, { class: "stationDialog", someTag: "stationREPAIRHIGH", greeting: "terminal", stationID: id1}); //, unique_id: "stationDialog"








                    /*var repairZoneMoveX = (-repairMovementZone.x * Math.cos(radToDeg * Math.PI / 180)) + coordsNPC.x;
                    var repairZoneMoveY = (-repairMovementZone.y * Math.sin(radToDeg * Math.PI / 180)) + coordsNPC.y;

                    var repairVisualZoneX = (-repairVisualZone.x * Math.cos(radToDeg * Math.PI / 180)) + coordsNPC.x;
                    var repairVisualZoneY = (-repairVisualZone.y * Math.sin(radToDeg * Math.PI / 180)) + coordsNPC.y;

                    var dirToVisualX = repairVisualZoneX - repairZoneMoveX;
                    var dirToVisualY = repairVisualZoneY - repairZoneMoveY;

                    var pointFrontNPCX = (1 * Math.cos(radToDeg * Math.PI / 180)) + coordsNPC.x; //radToDeg * Math.PI / 180
                    var pointFrontNPCY = (1 * Math.sin(radToDeg * Math.PI / 180)) + coordsNPC.y; //radToDeg * Math.PI / 180)

                    var dirFrontNPCX = pointFrontNPCX - coordsNPC.x;
                    var dirFrontNPCY = pointFrontNPCY - coordsNPC.y;
                    











                    //var pointFrontX = (3 * Math.cos(angle)) + coordsNPC.x; //radToDeg * Math.PI / 180
                    //var pointFrontY = (3 * Math.sin(angle)) + coordsNPC.y; //radToDeg * Math.PI / 180)

                    //magPlayer = Math.sqrt(veloPlayer.x * veloPlayer.x + veloPlayer.y * veloPlayer.y);
                    //var pointBackX = (-3 * Math.cos(angle)) + coordsNPC.x; //radToDeg * Math.PI / 180
                    //var pointBackY = (-3 * Math.sin(angle)) + coordsNPC.y; // radToDeg * Math.PI / 180

                    //var distPlayerFront = Math.sqrt(((pointFrontX - coordsNPC.x) * (pointFrontX - coordsNPC.x)) + ((pointFrontY - coordsNPC.y) * (pointFrontY - coordsNPC.y)));
                    //var distPlayerBack = Math.sqrt(((pointBackX - coordsNPC.x) * (pointBackX - coordsNPC.x)) + ((pointBackY - coordsNPC.y) * (pointBackY - coordsNPC.y)));

                    //var id2 = generator.AddNPCShipToSystem("station repair low", "ai_station_interior", 100, "xml_repair_low", systems[j], pointFrontX, pointFrontY, { class: "stationDialog", someTag: "stationREPAIRLOW", greeting: "terminal", stationID: "stationREPAIRLOW" }); //, unique_id: "stationDialog"
                    //var id3 = generator.AddNPCShipToSystem("station repair high", "ai_repair_high", 100, "xml_repair_high", systems[j], pointBackX, pointBackY, { class: "stationDialog", someTag: "stationREPAIRHIGH", greeting: "terminal", stationID: id1}); //, unique_id: "stationDialog"
                    
                }
            }*/




            //startOnce = false;
        }
    }


    if (canStartCounter == 1)
    {
        frameCounterToSpawnrepairUnit++;
    }
    _frameCounter++;

}







function Dot(aX, aY, bX, bY) {
    return (aX * bX) + (aY * bY);
}