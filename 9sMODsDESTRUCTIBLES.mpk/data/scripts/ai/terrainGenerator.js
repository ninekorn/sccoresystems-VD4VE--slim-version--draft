include(SC_Utilities.js);

using(console);
using(game);
using(npc);
using(generator);
using(items);
using(relations);
using(station);
using(ship);
using(timer);
using(storage);


function OnAIInited() {
    //set update interval
    //set update interval
    npc.SetDecisionsPerSecond(SHIP_ID, 1);

    player_id = npc.GetTag(SHIP_ID, "ownerPlayerShipId");

    sys_idNPC = ship.GetSystemID(SHIP_ID);
    sys_idPlayer = ship.GetSystemID(player_id);

    if (sys_idNPC == sys_idPlayer) {

    }
    else {
        var timerClass = timer.SetTimer(5, "PlayerIsNotInTheSameSystemKeepNpcBrainAlive", { what_to_say: "hello" }, 0);
    }
}
var widthLeft = 5;
var widthRight = 4;

var heightBottom = 5;
var heightTop = 4;

var terrainSquareSize = 100;

var offsetPos = { x: 0, y: 0 };


var systemName = "Earth";
var minimumDistance = 500;

var loadingTimeSwtc = 1;
var loadingTimeCounter = 0;
var loadingTimeCounterMAX = 4;

var terrainDepth = 25;


var terrainWidthLLimit = (terrainSquareSize * widthLeft) - terrainSquareSize;
var terrainWidthRLimit = (terrainSquareSize * widthRight) - terrainSquareSize;
var terrainHeightBLimit = (terrainSquareSize * heightBottom) - terrainSquareSize;
var terrainHeightTLimit = (terrainSquareSize * heightTop) - terrainSquareSize;

var spawningJumpgateSwtch = 0;
var spawningPlanetJumpSwtch = 0;

var stationLoadingCounter = 0;
var stationLoadingCounterMAX = 5;
var stationLoadingCounterSwtc = 0;

var mainFrameCounter = 0;

var mainFrameCounterFront = 0;
var mainFrameCounterBack = 0;
var arrayOfPlayerShips = [];



var initialPositionSpawned;

var globalEarthInitialTerrainPosIsFirstTile;
var globalEarthInitialTerrainPosIsFirstTileSwtc = 0;

var jg_id0;
var jg_id1;

var lastFrameJumpgateCreatedArray = [];
var lastFrameJumpgateCreatedArrayCounter = 0;
var lastFrameJumpgateCreatedArrayCounterMax = 0;
var lastFrameJumpgateCreatedArraySwtc = 0;

function PlayerIsNotInTheSameSystemKeepNpcBrainAlive(args) {

    if (lastFrameJumpgateCreatedArrayCounter > lastFrameJumpgateCreatedArrayCounterMax) {

        if (lastFrameJumpgateCreatedArraySwtc == 1) {
            //remove everything except the 2 last jumpgates
            for (var i = 0; i < lastFrameJumpgateCreatedArray.length - 2; i++) {
                generator.RemoveJumpgate(lastFrameJumpgateCreatedArray[i]);
            }
            lastFrameJumpgateCreatedArraySwtc = 0;
        }
        else {
            for (var i = 0; i < lastFrameJumpgateCreatedArray.length; i++) {
                generator.RemoveJumpgate(lastFrameJumpgateCreatedArray[i]);
            }
        }
        lastFrameJumpgateCreatedArrayCounter = 0;
    }
    lastFrameJumpgateCreatedArrayCounter++;







    if (jg_id0 != null) {
        //game.DeleteGameObject(jg_id0);
        //generator.RemoveJumpgate(jg_id0);
    }
    if (jg_id1 != null) {
        //game.DeleteGameObject(jg_id1);
        //generator.RemoveJumpgate(jg_id1);
    }

    //npcCoord = ship.GetCoordinates(SHIP_ID);
    //playerName = game.GetShipOwner(player_id);
    sys_idNPC = npc.GetCurrentSystemID(SHIP_ID);
    ////console.PrintError("PlayerIsNotInTheSameSystemKeepNpcBrainAlive terrain generator working? " + args.what_to_say);
    player_id = npc.GetTag(SHIP_ID, "ownerPlayerShipId");
    sys_idPlayer = ship.GetSystemID(player_id);







    if (player_id != null) {

        var inf = generator.GetSystemByID(sys_idPlayer);
        if (inf.name == systemName)//if (sys_idNPC === sys_idPlayer)//inf.name == systemName) //sys_idNPC === sys_idPlayer && 
        {
            playerCoords = ship.GetCoordinates(player_id);
            var posX = (Math.floor(playerCoords.x));
            var posY = (Math.floor(playerCoords.y));
            var playerPos = { x: posX, y: posY };


            //for (var x = -widthLeft; x <= widthRight; x++)
            {
                //for (var y = -heightBottom; y <= heightTop; y++)
                {




                    if (storage.IsSetGlobal("planet earth before invasion")) {

                        var globalEarthData = storage.GetGlobal("planet earth before invasion");
                        //console.PrintError("IsSetGlobal planet earth before invasion");
                        if (storage.IsSetGlobal("EarthInitialTerrainPos" + globalEarthData.sys_id)) {

                            var globalEarthInitialTerrainPos = storage.GetGlobal("EarthInitialTerrainPos" + globalEarthData.sys_id);
                            //console.PrintError("IsSetGlobal EarthInitialTerrainPos");


                            var endLoop = 0;

                            for (var ii = 0; ii < globalEarthInitialTerrainPos.length; ii++) {
                                if (globalEarthInitialTerrainPos[ii].isFirstTile == 1) {
                                    //console.PrintError("***FOUND 1ST TILE");
                                    globalEarthInitialTerrainPosIsFirstTile = globalEarthInitialTerrainPos[ii];

                                }
                            }

                            if (globalEarthInitialTerrainPosIsFirstTile != null) {


                                ////console.PrintError("px:" + playerCoords.x + "/py:" + playerCoords.y + "/tx:" + globalEarthInitialTerrainPos.x - terrainWidthLLimit + "/ty:" + globalEarthInitialTerrainPos.y - terrainWidthLLimit);

                                //console.PrintError("px:" + playerCoords.x + "/py:" + playerCoords.y);
                                ////console.PrintError("/0tx:" + -terrainWidthLLimit + "/0ty:" + terrainWidthLLimit);
                                ////console.PrintError("/1tx:" + globalEarthInitialTerrainPosIsFirstTile.x + "/1ty:" + globalEarthInitialTerrainPosIsFirstTile.y);
                                //console.PrintError("/tx:" + (globalEarthInitialTerrainPosIsFirstTile.x - terrainWidthLLimit) + "/tx:" + (globalEarthInitialTerrainPosIsFirstTile.x + terrainWidthRLimit));
                                //console.PrintError("/ty:" + (globalEarthInitialTerrainPosIsFirstTile.y - terrainHeightBLimit) + "/ty:" + (globalEarthInitialTerrainPosIsFirstTile.y + terrainHeightTLimit));



                                if (playerCoords.x <= globalEarthInitialTerrainPosIsFirstTile.x - terrainWidthLLimit || playerCoords.x > globalEarthInitialTerrainPosIsFirstTile.x + terrainWidthRLimit ||
                                    playerCoords.y <= globalEarthInitialTerrainPosIsFirstTile.y - terrainHeightBLimit || playerCoords.y > globalEarthInitialTerrainPosIsFirstTile.y + terrainHeightTLimit) {
                                    ////console.PrintError("player has reached the boundaries of the earth system");
                                    //sys_idNPC = npc.GetCurrentSystemID(SHIP_ID);
                                    //var npcCoord = ship.GetCoordinates(SHIP_ID);
                                    //var systemBases = game.GetSystemBases(sys_idNPC);
                                    //var jumpgates = game.GetSystemJumpgates(sys_idNPC);

                                    //var name = game.GetShipOwner(123142);
                                    //var whatStationType = npc.GetTag(SHIP_ID, "class");

                                    //if (spawningJumpgateSwtch == 0)
                                    {

                                        //if (whatStationType == "stationEXT") {
                                        //if (stationLoadingCounter >= stationLoadingCounterMAX) //>=2
                                        {
                                            //////console.PrintError("station can now warp you.");
                                            //if (mainFrameCounter >= 2) //>=2
                                            {

                                                //console.PrintError("angle:" + angleR);
                                                //var radToDeg = angleR * (180.0 / Math.PI);


                                                var playerCoord = ship.GetCoordinates(player_id);
                                                ////console.PrintError("player is in the earth system.");
                                                if (playerCoords.x <= globalEarthInitialTerrainPosIsFirstTile.x - terrainWidthLLimit + 10 || playerCoords.x > globalEarthInitialTerrainPosIsFirstTile.x + terrainWidthRLimit - 10) {

                                                    //console.PrintError("playerCoords.x <= globalEarthInitialTerrainPosIsFirstTile.x - terrainWidthLLimit || playerCoords.x > globalEarthInitialTerrainPosIsFirstTile.x + terrainWidthRLimit");
                                                    if (playerCoords.x <= globalEarthInitialTerrainPosIsFirstTile.x - terrainWidthLLimit + 10) {
                                                        var angleR = ship.GetRotation(player_id);
                                                        var degToRad = ((angleR) * Math.PI / 180);
                                                        //LEFT SIDE
                                                        jg_id0 = generator.AddJumpgate(
                                                            sys_idPlayer,
                                                            sys_idPlayer,
                                                            playerCoord.x,
                                                            playerCoord.y,
                                                            SC_Utilities.DegreeToRadian(180),
                                                            "jumpgate_station");
                                                        lastFrameJumpgateCreatedArray.push(jg_id0);
                                                        //ship.EnteringJumpgate(follow_ship);
                                                        ship.SetShipAsArrivedFromJumpgate(player_id, jg_id0);

                                                        //var isEnteringAnyJumpgates = ship.EnteringJumpgate(player_id);
                                                        //degToRad *= -1;

                                                        //var angleR = ship.GetRotation(player_id);
                                                        //var degToRad = (SC_Utilities.normalizedegrees((angleR + 180) * Math.PI / 180));
                                                        jg_id1 = generator.AddJumpgate(
                                                            sys_idPlayer,
                                                            sys_idPlayer,
                                                            globalEarthInitialTerrainPosIsFirstTile.x + terrainWidthRLimit - 20,
                                                            playerCoord.y,
                                                            0,
                                                            "jumpgate_station");
                                                        lastFrameJumpgateCreatedArray.push(jg_id1);
                                                        ship.SetShipAsArrivedFromJumpgate(player_id, jg_id1);
                                                        lastFrameJumpgateCreatedArraySwtc = 1;

                                                    }
                                                    else if (playerCoords.x > globalEarthInitialTerrainPosIsFirstTile.x + terrainWidthRLimit - 10) {
                                                        //RIGHT SIDE
                                                        var angleR = ship.GetRotation(player_id);
                                                        var degToRad = ((angleR) * Math.PI / 180);
                                                        jg_id0 = generator.AddJumpgate(
                                                            sys_idPlayer,
                                                            sys_idPlayer,
                                                            playerCoord.x,
                                                            playerCoord.y,
                                                            0,
                                                            "jumpgate_station");
                                                        lastFrameJumpgateCreatedArray.push(jg_id0);
                                                        //ship.EnteringJumpgate(follow_ship);
                                                        ship.SetShipAsArrivedFromJumpgate(player_id, jg_id0);

                                                        //var isEnteringAnyJumpgates = ship.EnteringJumpgate(player_id);
                                                        // var angleR = ship.GetRotation(player_id);
                                                        //var degToRad = ((angleR + 180) * Math.PI / 180);
                                                        //var degToRad = ((SC_Utilities.normalizedegrees(angleR)) * Math.PI / 180);
                                                        jg_id1 = generator.AddJumpgate(
                                                            sys_idPlayer,
                                                            sys_idPlayer,
                                                            globalEarthInitialTerrainPosIsFirstTile.x - terrainWidthLLimit + 20,
                                                            playerCoord.y,
                                                            SC_Utilities.DegreeToRadian(180),
                                                            "jumpgate_station");
                                                        lastFrameJumpgateCreatedArray.push(jg_id1);
                                                        ship.SetShipAsArrivedFromJumpgate(player_id, jg_id1);
                                                        lastFrameJumpgateCreatedArraySwtc = 1;
                                                    }
                                                }
                                                else if (playerCoords.y <= globalEarthInitialTerrainPosIsFirstTile.y - terrainHeightBLimit + 10 || playerCoords.y > globalEarthInitialTerrainPosIsFirstTile.y + terrainHeightTLimit - 10) {

                                                    //console.PrintError("playerCoords.y <= globalEarthInitialTerrainPosIsFirstTile.y - terrainHeightBLimit || playerCoords.y > globalEarthInitialTerrainPosIsFirstTile.y + terrainHeightTLimit");
                                                    //LEFT SIDE
                                                    if (playerCoords.y <= globalEarthInitialTerrainPosIsFirstTile.y - terrainHeightBLimit + 10) {
                                                        //bottom SIDE
                                                        var angleR = ship.GetRotation(player_id);
                                                        var degToRad = ((angleR) * Math.PI / 180);
                                                        jg_id0 = generator.AddJumpgate(
                                                            sys_idPlayer,
                                                            sys_idPlayer,
                                                            playerCoord.x,
                                                            playerCoord.y,
                                                            SC_Utilities.DegreeToRadian(270),
                                                            "jumpgate_station");
                                                        lastFrameJumpgateCreatedArray.push(jg_id0);
                                                        //ship.EnteringJumpgate(follow_ship);
                                                        ship.SetShipAsArrivedFromJumpgate(player_id, jg_id0);

                                                        //var isEnteringAnyJumpgates = ship.EnteringJumpgate(player_id);
                                                        //var angleR = ship.GetRotation(player_id);
                                                        //var degToRad = ((angleR + 180) * Math.PI / 180);

                                                        //var degToRad = (SC_Utilities.normalizedegrees((-angleR) * Math.PI / 180));

                                                        jg_id1 = generator.AddJumpgate(
                                                            sys_idPlayer,
                                                            sys_idPlayer,
                                                            playerCoord.x,
                                                            globalEarthInitialTerrainPosIsFirstTile.y + terrainHeightTLimit - 20,
                                                            SC_Utilities.DegreeToRadian(90),
                                                            "jumpgate_station");
                                                        lastFrameJumpgateCreatedArray.push(jg_id1);
                                                        ship.SetShipAsArrivedFromJumpgate(player_id, jg_id1);
                                                        lastFrameJumpgateCreatedArraySwtc = 1;
                                                    }
                                                    else if (playerCoords.y > globalEarthInitialTerrainPosIsFirstTile.y + terrainHeightTLimit - 10) {
                                                        //RIGHT SIDE

                                                        var angleR = ship.GetRotation(player_id);
                                                        var degToRad = ((angleR) * Math.PI / 180);
                                                        jg_id0 = generator.AddJumpgate(
                                                            sys_idPlayer,
                                                            sys_idPlayer,
                                                            playerCoord.x,
                                                            playerCoord.y,
                                                            SC_Utilities.DegreeToRadian(90),
                                                            "jumpgate_station");
                                                        lastFrameJumpgateCreatedArray.push(jg_id0);
                                                        //ship.EnteringJumpgate(follow_ship);
                                                        ship.SetShipAsArrivedFromJumpgate(player_id, jg_id0);

                                                        //var isEnteringAnyJumpgates = ship.EnteringJumpgate(player_id);
                                                        //var angleR = ship.GetRotation(player_id);
                                                        //var degToRad = ((angleR + 180) * Math.PI / 180);
                                                        //var degToRad = (SC_Utilities.normalizedegrees((-angleR) * Math.PI / 180));
                                                        jg_id1 = generator.AddJumpgate(
                                                            sys_idPlayer,
                                                            sys_idPlayer,
                                                            playerCoord.x,
                                                            globalEarthInitialTerrainPosIsFirstTile.y - terrainHeightBLimit + 20,
                                                            SC_Utilities.DegreeToRadian(270),
                                                            "jumpgate_station");
                                                        lastFrameJumpgateCreatedArray.push(jg_id1);
                                                        ship.SetShipAsArrivedFromJumpgate(player_id, jg_id1);
                                                        lastFrameJumpgateCreatedArraySwtc = 1;
                                                    }
                                                }


                                                //var isPlayer = !ship.IsNpc(ships[i]);
                                                //var isPlayer = game.IsShipPlayerControlled(ships[i]);
                                                //if (isPlayer)
                                                //{

                                                //player_id = ships[i];

                                                //}


                                                /*if (ships != null) {
                                                    for (var i = 0; i < ships.length; i++) {
                                                        if (ships[i] != null) {
            
                                                            //if (sys_idPlayer == sys_idNPC)
                                                            {
                                                                
                                                            }
                                                        }
                                                    }
                                                }*/
                                                mainFrameCounter = 0;
                                            }

                                            stationLoadingCounter = 0;
                                            stationLoadingCounterSwtc = 1;
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
    mainFrameCounter++;
    mainFrameCounterFront++;

    if (stationLoadingCounterSwtc == 1) {


    }
    else {
        stationLoadingCounter++;
    }






    //con
    //SPAWNING STATION INTERIOR JUMPGATES
    //SPAWNING STATION INTERIOR JUMPGATES
    //SPAWNING STATION INTERIOR JUMPGATES
    //var stationDataINT = storage.GetGlobal("stationINT" + sys_idNPC);
    //var getSomeIndex = stationDataINT.xml_id.substring(11, stationDataINT.xml_id.length); //outpost_01_0
    //var parsedAngle = parseInt(getSomeIndex);
    //var degToRad = (parsedAngle * Math.PI / 180);
    //SPAWNING STATION INTERIOR JUMPGATES
    //SPAWNING STATION INTERIOR JUMPGATES
    //SPAWNING STATION INTERIOR JUMPGATES


    //var distPlayerFront = Math.sqrt(((stationDataINT.x - coordsPlayerShip.x) * (stationDataINT.x - coordsPlayerShip.x)) + ((stationDataINT.y - coordsPlayerShip.y) * (stationDataINT.y - coordsPlayerShip.y)));

    /*if (distPlayerFront <= 50) {

        //magPlayer = Math.sqrt(veloPlayer.x * veloPlayer.x + veloPlayer.y * veloPlayer.y);


        ////console.PrintError("is the frame coming here");

        //if (correctTag == "stationINT") {

        /*var stationDataINT = storage.GetGlobal("stationEXT" + systems[j]);
        var coordsStationINT = { x: stationDataINT.x, y: stationDataINT.y };
 
        var randomIndexStationOutpost = Math.floor(Math.random() * stationDataINT.xml_id.length);
 
        var getSomeIndex = stationDataINT.xml_id.substring(11, stationDataINT.xml_id.length); //outpost_01_0
 
        //var index = indexOfStuff(getSomeIndex);
        var parsedAngle = parseInt(getSomeIndex);



        //SPAWNING STATION EXTERIOR JUMPGATES
        //SPAWNING STATION EXTERIOR JUMPGATES
        //SPAWNING STATION EXTERIOR JUMPGATES
        var stationDataEXT = storage.GetGlobal("stationEXT" + stationDataINT.sys_id_link);
        //var randomIndexStationOutpost = Math.floor(Math.random() * stationDataINT.xml_id.length);       

        var tempCoordINTLEFTX = stationDataINT.x - 40;
        var tempCoordINTLEFTY = stationDataINT.y - 12.5;
        var tempCoordINTLEFT = { x: tempCoordINTLEFTX, y: tempCoordINTLEFTY };

        var tempCoordINTRIGHTX = stationDataINT.x + 40;
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

        var tempCoordINTLEFTX = stationDataINT.x;
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

        /*if (degToRadTWO < 0) {
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
    }*/









    if (player_id != null) {

        var inf = generator.GetSystemByID(sys_idPlayer);
        //if (inf.name == systemName) //sys_idNPC === sys_idPlayer && 
        {



            if (startOnceDecision == 0) {
                //////console.PrintError("Generating Terrain for the player.");



                for (var x = -widthLeft; x <= widthRight; x++) {
                    for (var y = -heightBottom; y <= heightTop; y++) {

                        xx = x;
                        yy = y;

                        if (x < 0) {
                            xx *= -1;
                            xx += (widthRight);
                        }

                        if (y < 0) {
                            yy *= -1;
                            yy += (heightTop);
                        }

                        playerCoords = ship.GetCoordinates(player_id);
                        var posX = (Math.floor(playerCoords.x));
                        var posY = (Math.floor(playerCoords.y));
                        var playerPos = { x: posX, y: posY };

                        var terrainPosX = ((Math.floor(posX) / 10) * 10) + (x * terrainSquareSize);
                        var terrainPosY = ((Math.floor(posY) / 10) * 10) + (y * terrainSquareSize);
                        var terrainPos = { x: terrainPosX, y: terrainPosY };
                        terrainPos.x += offsetPos.x;
                        terrainPos.y += offsetPos.y;


                        if (arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] == 0 || arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] == null) {
                            //////console.PrintError("arrayOfTerrain[xx * width + yy] == 0");
                            var someCoordsEarthTerrain = { x: terrainPos.x, y: terrainPos.y };

                            //var item_list = "droplist_empty";
                            //generator.AddContainer(sys_idPlayer, someCoordsEarthTerrain.x, someCoordsEarthTerrain.y, "terrain", item_list, { itemlist: item_list, class: "terrain" });
                            /*
                            var decoID0 = generator.AddDecoration(
                                sys_idPlayer,
                                "planet_earth_night",
                                someCoordsEarthTerrain,
                                10,
                                { X: 0, Y: 0, Z: 0 },
                                { X: 0, Y: 0, Z: 1 },
                                7);*/

                            arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] = 1;
                        }
                        //arrayOfTerrain.push(0);
                    }
                }
                startOnceDecision = 1;
            }



            if (loadingTimeSwtc == 0) {
                loadingTimeSwtc == 1;
            }


            if (loadingTimeSwtc == 1) {
                if (loadingTimeCounter >= loadingTimeCounterMAX) {

                    //////console.PrintError("trying to spawn terrain");
                    playerCoords = ship.GetCoordinates(player_id);
                    var posX = (Math.floor(playerCoords.x));
                    var posY = (Math.floor(playerCoords.y));
                    var playerPos = { x: posX, y: posY };

                    var xx = 0;
                    var yy = 0;

                    for (var x = -widthLeft; x <= widthRight; x++) {
                        for (var y = -heightBottom; y <= heightTop; y++) {
                            xx = x;
                            yy = y;

                            if (x < 0) {
                                xx *= -1;
                                xx += (widthRight);
                            }

                            if (y < 0) {
                                yy *= -1;
                                yy += (heightTop);
                            }
                            var index = xx * (widthLeft + widthRight + 1) + yy;
                            var terrainPosX = ((Math.floor(posX) / 10) * 10) + (x * terrainSquareSize);
                            var terrainPosY = ((Math.floor(posY) / 10) * 10) + (y * terrainSquareSize);
                            var terrainPos = { x: terrainPosX, y: terrainPosY };
                            terrainPos.x += offsetPos.x;
                            terrainPos.y += offsetPos.y;

                            var distancePlayerToForcedStopPivot = SC_Utilities.npcCheckDistance(playerPos, terrainPos) * 0.1;//SC_Utilities.GetDistance(playerPos, terrainPos); //npcCheckDistance

                            //////console.PrintError("dist: " + distancePlayerToForcedStopPivot);
                            //if (distancePlayerToForcedStopPivot < minimumDistance) 
                            {
                                ////////console.PrintError("distancePlayerToForcedStopPivot < 50: " + distancePlayerToForcedStopPivot);


                                //////console.PrintError(arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] );
                                if (arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] == 1 || arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] == null) {
                                    //////console.PrintError("arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] == 0");
                                    var someCoordsEarthTerrain = { x: terrainPos.x, y: terrainPos.y };

                                    /*var globalEarthData = storage.GetGlobal("planet earth before invasion");

                                    if (globalEarthData != null) {

                                        var decoID0 = generator.AddDecoration(
                                            globalEarthData.sys_id,
                                            "terrain",
                                            someCoordsEarthTerrain,
                                            terrainDepth,
                                            { X: 0, Y: 0, Z: 0 },
                                            { X: 0, Y: 0, Z: 0 },
                                            1);


                                        var isFirstTile = 0;
                                        if (x == 0 && y == 0) {
                                            isFirstTile = 1;
                                        }


                                        if (!storage.IsSetGlobal("EarthInitialTerrainPos" + globalEarthData.sys_id)) {
                                            var globalEarthInitialTerrainPos = [];
                                            globalEarthInitialTerrainPos.push({ decoID: decoID0, x: someCoordsEarthTerrain.x, y: someCoordsEarthTerrain.y, sys_id: globalEarthData.sys_id, depth: terrainDepth, isFirstTile: isFirstTile });
                                            storage.SetGlobal("EarthInitialTerrainPos" + globalEarthData.sys_id, globalEarthInitialTerrainPos);
                                            ////console.PrintError("0EarthInitialTerrainPos SET");
                                        }
                                        else {
                                            var globalEarthInitialTerrainPos = storage.GetGlobal("EarthInitialTerrainPos" + globalEarthData.sys_id);

                                            //var globalEarthInitialTerrainPosTWO = [];
                                            globalEarthInitialTerrainPos.push({ decoID: decoID0, x: someCoordsEarthTerrain.x, y: someCoordsEarthTerrain.y, sys_id: globalEarthData.sys_id, depth: terrainDepth, isFirstTile: isFirstTile });

                                            //var newData .x = someCoordsEarthTerrain.x;
                                            //globalEarthInitialTerrainPos[0].y = someCoordsEarthTerrain.y;
                                            //globalEarthInitialTerrainPos[0].sys_id = globalEarthData.sys_id;
                                            //globalEarthInitialTerrainPos[0].depth = terrainDepth;

                                            storage.SetGlobal("EarthInitialTerrainPos" + globalEarthData.sys_id, globalEarthInitialTerrainPos);
                                            ////console.PrintError("1EarthInitialTerrainPos SET");
                                        }
                                    }
                                    else {
                                        ////console.PrintError("null globalEarthData terrainGenerator.js");



                                    }*/

                                    //var item_list = "droplist_empty";
                                    ////console.PrintError("crate terrain added.");
                                    //generator.AddContainer(sys_idPlayer, someCoordsEarthTerrain.x, someCoordsEarthTerrain.y, "terrain", item_list, { itemlist: item_list, class: "terrain" });
                                    loadingTimeCounter = 0;
                                    arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] = 0;
                                }
                            }
                        }
                    }
                }
                loadingTimeCounter++;
            }





            var total = ((widthLeft * widthRight) + 1) * ((heightBottom * heightTop) + 1); // * SC_Globals.tinyChunkDepth

            var xx = 0;
            var yy = 0;
            var zz = 0;

            var switchXX = 1;
            var switchYY = 0;

            for (var t = 0; t < total; t++) {
                var currentByte = arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy];

                var index = xx * (widthLeft + widthRight + 1) + yy;

                /*zz++;
                if (zz >= SC_Globals.tinyChunkDepth) {
                    yy++;
                    zz = 0;
                    switchYY = 1;
                }*/
                ////console.PrintError("x:" + xx + "/y:" + yy);


                if (yy >= heightTop && switchYY == 1) {
                    yy = 0;
                    switchYY = 0;
                    switchXX = 1;
                }
                else {
                    if (yy < heightTop && switchXX == 0) {
                        yy++;
                    }
                }


                if (xx >= widthRight && switchXX == 1) {
                    xx = 0;
                    switchXX = 0;
                    switchYY = 1;
                    break;
                }
                else {
                    if (xx < widthRight && switchXX == 1) {
                        xx++;
                    }
                }
            }










            /*
            for (var x = 0; x < 10; x++)
            {
                for (var y = 0; y < 10; y++)
                {
                    var xx = x*10;
                    var yy = y*10;
    
                    //if (xx < 0)
                    //{
                    //    xx *= -1;
                    //    xx += 10 - 1;
                    //}
                    //
                    //if (yy < 0) {
                    //    yy *= -1;
                   //     yy += 10 - 1;
                   // }
    
                    var currentPosX = (Math.floor(posX) / 10) + x;
                    var currentPosY = (Math.floor(posY) / 10) + y;
    
    
    
                    var PoserX = (Math.floor(posX)) + (xx);
                    var PoserY = (Math.floor(posY)) + (yy);
    
                    if (currentPosX > -width && currentPosX < width && currentPosY > -height && currentPosY < height)
                    {
                        if (arrayOfTerrain[currentPosX * width + currentPosY] === 0)
                        {
                            //var id = generator.AddDecoration(sys_idPlayer, "terrain", { x: PoserX, y: PoserY+10 }, 100, { x: 0, y: 0 }, { x: 0, y: 0 }, 0.terrainDepth);
    
                            var coords = { x: PoserX, y: PoserY };
                            var rotation = { x:0, y: 0 };
                            var rotationSpeed = { x: 0, y: 0 };
                            var distance = 100;
                            var scale = 0.terrainDepth;
    
                            var id = generator.AddDecoration(sys_idPlayer, "terrain", coords, distance, rotation, rotationSpeed, scale);
    
    
    
    
    
    
                            arrayOfTerrain[currentPosX * width + currentPosY] = 1;
                        }
                    }
                }
            }*/
        }
    }


    /*
    if (player_id != null)
    {
        var inf = generator.GetSystemByID(sys_idPlayer);
        if (sys_idNPC === sys_idPlayer && inf.name == systemName) {
            if (startOnce) {
                //////console.PrintError("Generating Terrain for the player.");
                for (var x = -widthLeft; x <= widthRight; x++) {
                    for (var y = -heightBottom; y <= heightTop; y++) {
                        arrayOfTerrain.push(0);
                    }
                }

                /*
                for (var x = -width; x < width; x++)
                {
                    for (var y = -height; y < height; y++)
                    {
                        arrayOfTerrain[x * width + y] = 0;
                    }
                }

                startOnce = false;
            }

            playerCoords = ship.GetCoordinates(player_id);
            var posX = (Math.floor(playerCoords.x));
            var posY = (Math.floor(playerCoords.y));
            var playerPos = { x: posX, y: posY };

            var xx = 0;
            var yy = 0;

            for (var x = -widthLeft; x <= widthRight; x++) {
                for (var y = -heightBottom; y <= heightTop; y++) {
                    xx = x;
                    yy = y;

                    if (x < 0) {
                        xx *= -1;
                        xx += (widthRight);
                    }

                    if (y < 0) {
                        yy *= -1;
                        yy += (heightTop);
                    }

                    var terrainPosX = ((Math.floor(posX))) + (x * terrainSquareSize);
                    var terrainPosY = ((Math.floor(posY))) + (y * terrainSquareSize);
                    var terrainPos = { x: terrainPosX, y: terrainPosY };
                    terrainPos.x += offsetPos.x;
                    terrainPos.y += offsetPos.y;

                    var distancePlayerToForcedStopPivot = SC_Utilities.GetDistance(playerPos, terrainPos);
                    //if (distancePlayerToForcedStopPivot < minimumDistance) {
                        if (arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] == 0 || arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] == null) {

                            var someCoordsEarthTerrain = { x: terrainPos.x, y: terrainPos.y };


                            /*var decoID0 = generator.AddDecoration(
                                sys_idPlayer,
                                "planet_earth_night",
                                someCoordsEarthTerrain,
                                10,
                                { X: 0, Y: 0, Z: 0 },
                                { X: 0, Y: 0, Z: 1 },
                                7);
                            var item_list = "droplist_empty";
                            ////////console.PrintError("visual rooftop added.");
                            generator.AddContainer(sys_idPlayer, someCoordsEarthTerrain.x, someCoordsEarthTerrain.y, "scrap_metal_000", item_list, { itemlist: item_list, class: "terrain" });



                            arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] = 1;
                        }
                    }
                }
            }
        }
        /*
        for (var x = 0; x < 10; x++)
        {
            for (var y = 0; y < 10; y++)
            {
                var xx = x*10;
                var yy = y*10;
 
                //if (xx < 0)
                //{
                //    xx *= -1;
                //    xx += 10 - 1;
                //}
                //
                //if (yy < 0) {
                //    yy *= -1;
               //     yy += 10 - 1;
               // }
 
                var currentPosX = (Math.floor(posX) / 10) + x;
                var currentPosY = (Math.floor(posY) / 10) + y;
 
 
 
                var PoserX = (Math.floor(posX)) + (xx);
                var PoserY = (Math.floor(posY)) + (yy);
 
                if (currentPosX > -width && currentPosX < width && currentPosY > -height && currentPosY < height)
                {
                    if (arrayOfTerrain[currentPosX * width + currentPosY] === 0)
                    {
                        //var id = generator.AddDecoration(sys_idPlayer, "terrain", { x: PoserX, y: PoserY+10 }, 100, { x: 0, y: 0 }, { x: 0, y: 0 }, 0.terrainDepth);
 
                        var coords = { x: PoserX, y: PoserY };
                        var rotation = { x:0, y: 0 };
                        var rotationSpeed = { x: 0, y: 0 };
                        var distance = 100;
                        var scale = 0.terrainDepth;
 
                        var id = generator.AddDecoration(sys_idPlayer, "terrain", coords, distance, rotation, rotationSpeed, scale);
 
 
 
 
 
 
                        arrayOfTerrain[currentPosX * width + currentPosY] = 1;
                    }
                }
            }
        }

    }*/






    //var timerClass = timer.SetTimer(3, "PlayerIsNotInTheSameSystemKeepNpcBrainAlive", { what_to_say: "hello" }, 1);
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

var width = 50;
var height = 50;

var planeWidth = 10;
var playerCoords;

var startOnceDecision = 0;

function Decision(args) {

    if (jg_id0 != null) {
        //game.DeleteGameObject(jg_id0);
        generator.RemoveJumpgate(jg_id0);
    }
    if (jg_id1 != null) {
        //game.DeleteGameObject(jg_id1);
        generator.RemoveJumpgate(jg_id1);
    }



    //playerName = game.GetShipOwner(player_id);
    sys_idNPC = npc.GetCurrentSystemID(SHIP_ID);
    //sys_idPlayer = ship.GetSystemID(player_id);
    //npcCoord = ship.GetCoordinates(SHIP_ID);
    //playerCoords = ship.GetCoordinates(player_id);






    ////console.PrintError("Decision(args) terrain generator working?");





    player_id = npc.GetTag(SHIP_ID, "ownerPlayerShipId");
    ////console.PrintError("pid: " + player_id);
    sys_idPlayer = ship.GetSystemID(player_id);
    ////console.PrintError("sid: " + sys_idPlayer);









































    //storage.SetGlobal("planet earth before invasion", { id: null, x: null, y: null, sys_id: sys_id2, sys_id_link: sys_id1, xml_id: "planet_earth_night", DegAngle: parsedAngle, class: tagClass, name: curinf.name, baseID: null });
    //storage.SetGlobal("planet earth after invasion", { id: null, x: null, y: null, sys_id: sys_id2, sys_id_link: sys_id1, xml_id: "planet_earth_night", DegAngle: parsedAngle, class: tagClass, name: curinf.name, baseID: null });





    if (player_id != null) {

        var inf = generator.GetSystemByID(sys_idPlayer);
        if (sys_idNPC === sys_idPlayer)//inf.name == systemName) //sys_idNPC === sys_idPlayer && 
        {
            if (startOnceDecision == 0) {
                //////console.PrintError("Generating Terrain for the player.");



                for (var x = -widthLeft; x <= widthRight; x++) {
                    for (var y = -heightBottom; y <= heightTop; y++) {

                        xx = x;
                        yy = y;

                        if (x < 0) {
                            xx *= -1;
                            xx += (widthRight);
                        }

                        if (y < 0) {
                            yy *= -1;
                            yy += (heightTop);
                        }

                        playerCoords = ship.GetCoordinates(player_id);
                        var posX = (Math.floor(playerCoords.x));
                        var posY = (Math.floor(playerCoords.y));
                        var playerPos = { x: posX, y: posY };
                        var terrainPosX = (x * terrainSquareSize);
                        var terrainPosY = (y * terrainSquareSize);
                        var terrainPos = { x: terrainPosX, y: terrainPosY };
                        terrainPos.x += offsetPos.x;
                        terrainPos.y += offsetPos.y;


                        if (arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] == 0 || arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] == null) {
                            //////console.PrintError("arrayOfTerrain[xx * width + yy] == 0");
                            var someCoordsEarthTerrain = { x: terrainPos.x, y: terrainPos.y };

                            //var item_list = "droplist_empty";
                            //generator.AddContainer(sys_idPlayer, someCoordsEarthTerrain.x, someCoordsEarthTerrain.y, "terrain", item_list, { itemlist: item_list, class: "terrain" });
                            /*
                            var decoID0 = generator.AddDecoration(
                                sys_idPlayer,
                                "planet_earth_night",
                                someCoordsEarthTerrain,
                                10,
                                { X: 0, Y: 0, Z: 0 },
                                { X: 0, Y: 0, Z: 1 },
                                7);*/

                            arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] = 1;
                        }
                        //arrayOfTerrain.push(0);
                    }
                }
                startOnceDecision = 1;
            }



            if (loadingTimeSwtc == 0) {
                loadingTimeSwtc == 1;
            }


            if (loadingTimeSwtc == 1) {
                if (loadingTimeCounter >= loadingTimeCounterMAX) {

                    //////console.PrintError("trying to spawn terrain");
                    playerCoords = ship.GetCoordinates(player_id);
                    var posX = (Math.floor(playerCoords.x));
                    var posY = (Math.floor(playerCoords.y));
                    var playerPos = { x: posX, y: posY };

                    var xx = 0;
                    var yy = 0;

                    for (var x = -widthLeft; x <= widthRight; x++) {
                        for (var y = -heightBottom; y <= heightTop; y++) {
                            xx = x;
                            yy = y;

                            if (x < 0) {
                                xx *= -1;
                                xx += (widthRight);
                            }

                            if (y < 0) {
                                yy *= -1;
                                yy += (heightTop);
                            }

                            var terrainPosX = (x * terrainSquareSize);
                            var terrainPosY = (y * terrainSquareSize);
                            var terrainPos = { x: terrainPosX, y: terrainPosY };

                            var planetEarthGlobalVariableINT;
                            var systems = generator.GetAllSystems();
                            for (var j = 0; j < systems.length; j++) {
                                var inf = generator.GetSystemByID(systems[j]);
                                if (inf.name == "Solar System") {
                                    planetEarthGlobalVariableINT = storage.GetGlobal("planet earth EXT" + systems[j]);
                                }
                            }

                            if (planetEarthGlobalVariableINT.x != null && planetEarthGlobalVariableINT.y != null) {
                                terrainPos.x += planetEarthGlobalVariableINT.x;
                                terrainPos.y += planetEarthGlobalVariableINT.y;
                            }



                            terrainPos.x += offsetPos.x;
                            terrainPos.y += offsetPos.y;

                            var distancePlayerToForcedStopPivot = SC_Utilities.npcCheckDistance(playerPos, terrainPos) * 0.1;//SC_Utilities.GetDistance(playerPos, terrainPos); //npcCheckDistance

                            //////console.PrintError("dist: " + distancePlayerToForcedStopPivot);
                            ////if (distancePlayerToForcedStopPivot < minimumDistance)
                            {
                                ////////console.PrintError("distancePlayerToForcedStopPivot < 50: " + distancePlayerToForcedStopPivot);


                                //////console.PrintError(arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] );
                                if (arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] == 1 || arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] == null) {
                                    //////console.PrintError("arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] == 0");
                                    var someCoordsEarthTerrain = { x: terrainPos.x, y: terrainPos.y };


                                    var globalEarthData = storage.GetGlobal("planet earth before invasion");

                                    if (globalEarthData != null) {




                                        var decoID0 = generator.AddDecoration(
                                            globalEarthData.sys_id,
                                            "terrain",
                                            someCoordsEarthTerrain,
                                            terrainDepth,
                                            { X: 0, Y: 0, Z: 0 },
                                            { X: 0, Y: 0, Z: 0 },
                                            1);


                                        var isFirstTile = 0;
                                        if (x == 0 && y == 0) {
                                            isFirstTile = 1;
                                        }

                                        if (!storage.IsSetGlobal("EarthInitialTerrainPos" + globalEarthData.sys_id)) {
                                            var globalEarthInitialTerrainPos = [];
                                            globalEarthInitialTerrainPos.push({ decoID: decoID0, x: someCoordsEarthTerrain.x, y: someCoordsEarthTerrain.y, sys_id: globalEarthData.sys_id, depth: terrainDepth, isFirstTile: isFirstTile });
                                            storage.SetGlobal("EarthInitialTerrainPos" + globalEarthData.sys_id, globalEarthInitialTerrainPos);
                                            ////console.PrintError("0EarthInitialTerrainPos SET");
                                        }
                                        else {
                                            var globalEarthInitialTerrainPos = storage.GetGlobal("EarthInitialTerrainPos" + globalEarthData.sys_id);

                                            //var globalEarthInitialTerrainPosTWO = [];
                                            globalEarthInitialTerrainPos.push({ decoID: decoID0, x: someCoordsEarthTerrain.x, y: someCoordsEarthTerrain.y, sys_id: globalEarthData.sys_id, depth: terrainDepth, isFirstTile: isFirstTile });

                                            //var newData .x = someCoordsEarthTerrain.x;
                                            //globalEarthInitialTerrainPos[0].y = someCoordsEarthTerrain.y;
                                            //globalEarthInitialTerrainPos[0].sys_id = globalEarthData.sys_id;
                                            //globalEarthInitialTerrainPos[0].depth = terrainDepth;

                                            storage.SetGlobal("EarthInitialTerrainPos" + globalEarthData.sys_id, globalEarthInitialTerrainPos);
                                            ////console.PrintError("1EarthInitialTerrainPos SET");
                                        }
                                    }
                                    else {
                                        ////console.PrintError("null globalEarthData terrainGenerator.js");



                                    }

                                    //var item_list = "droplist_empty";
                                    ////console.PrintError("crate terrain added.");
                                    //generator.AddContainer(sys_idPlayer, someCoordsEarthTerrain.x, someCoordsEarthTerrain.y, "terrain", item_list, { itemlist: item_list, class: "terrain" });
                                    loadingTimeCounter = 0;
                                    arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy] = 0;
                                }
                            }
                        }
                    }



                    loadingTimeSwtc = 0;
                }
                loadingTimeCounter++;
            }





            var total = ((widthLeft * widthRight) + 1) * ((heightBottom * heightTop) + 1); // * SC_Globals.tinyChunkDepth

            var xx = 0;
            var yy = 0;
            var zz = 0;

            var switchXX = 1;
            var switchYY = 0;

            for (var t = 0; t < total; t++) {
                var currentByte = arrayOfTerrain[xx * (widthLeft + widthRight + 1) + yy];

                var index = xx * (widthLeft + widthRight + 1) + yy;

                /*zz++;
                if (zz >= SC_Globals.tinyChunkDepth) {
                    yy++;
                    zz = 0;
                    switchYY = 1;
                }*/
                ////console.PrintError("x:" + xx + "/y:" + yy);


                if (yy >= heightTop && switchYY == 1) {
                    yy = 0;
                    switchYY = 0;
                    switchXX = 1;
                }
                else {
                    if (yy < heightTop && switchXX == 0) {
                        yy++;
                    }
                }


                if (xx >= widthRight && switchXX == 1) {
                    xx = 0;
                    switchXX = 0;
                    switchYY = 1;
                    break;
                }
                else {
                    if (xx < widthRight && switchXX == 1) {
                        xx++;
                    }
                }
            }










            /*
            for (var x = 0; x < 10; x++)
            {
                for (var y = 0; y < 10; y++)
                {
                    var xx = x*10;
                    var yy = y*10;
    
                    //if (xx < 0)
                    //{
                    //    xx *= -1;
                    //    xx += 10 - 1;
                    //}
                    //
                    //if (yy < 0) {
                    //    yy *= -1;
                   //     yy += 10 - 1;
                   // }
    
                    var currentPosX = (Math.floor(posX) / 10) + x;
                    var currentPosY = (Math.floor(posY) / 10) + y;
    
    
    
                    var PoserX = (Math.floor(posX)) + (xx);
                    var PoserY = (Math.floor(posY)) + (yy);
    
                    if (currentPosX > -width && currentPosX < width && currentPosY > -height && currentPosY < height)
                    {
                        if (arrayOfTerrain[currentPosX * width + currentPosY] === 0)
                        {
                            //var id = generator.AddDecoration(sys_idPlayer, "terrain", { x: PoserX, y: PoserY+10 }, 100, { x: 0, y: 0 }, { x: 0, y: 0 }, 0.terrainDepth);
    
                            var coords = { x: PoserX, y: PoserY };
                            var rotation = { x:0, y: 0 };
                            var rotationSpeed = { x: 0, y: 0 };
                            var distance = 100;
                            var scale = 0.terrainDepth;
    
                            var id = generator.AddDecoration(sys_idPlayer, "terrain", coords, distance, rotation, rotationSpeed, scale);
    
    
    
    
    
    
                            arrayOfTerrain[currentPosX * width + currentPosY] = 1;
                        }
                    }
                }
            }*/
        }
        else {




            startOnceDecision = 0;
        }

        if (player_id != null) {
            var isEnteringAnyJumpgates = ship.EnteringJumpgate(player_id);
            if (isEnteringAnyJumpgates) {
                var timerClass = timer.SetTimer(5, "PlayerIsNotInTheSameSystemKeepNpcBrainAlive", { what_to_say: "hello" }, 0);
            }

        }
    }
}





        /*//if (sys_idNPC == sys_idPlayer)
        {
            //console.Print(sys_idPlayer + " _ " + sys_idNPC);

            if (startOnce) {
                playerCoords = ship.GetCoordinates(ships[i]);
                //var obj0 = generator.GetSystemByID(sys_idPlayer);
                //var obj1 = generator.GetSystemByID(sys_idNPC);
                //console.Print(obj0.name);
                //console.Print(obj1.name);


                //var id = generator.AddDecoration(sys_idPlayer, "terrain", { x: PoserX, y: PoserY }, distance, { x: 0, y: 0 }, { x: 0, y: 0 }, scale);
                //var id = generator.AddDecoration(sys_idPlayer, "terrain", coords, distance, rotation, rotationSpeed, scale);

                var posX = Math.floor(playerCoords.x);
                var posY = Math.floor(playerCoords.y);


                for (var xi = -width; xi <= width; xi += 5) {
                    for (var yi = -height; yi <= height; yi += 5) {
                        var PoserX = posX + xi; //posX + 
                        var PoserY = posY + yi; // posY +


                        var coords = { x: PoserX, y: PoserY };
                        var rotation = { X: 0, Y: 0, Z: 0 };
                        var rotationSpeed = { X: 0, Y: 0, Z: 0 };
                        var distance = 0;
                        var scale = 1;

                        /*if (!generator.AreThereAnyObjects(sys_idPlayer, PoserX, PoserY, 2))
                        {
                            //generator.AddDecoration(sys_idPlayer, "broken_asteroid_01", coords, distance, rotation, rotationSpeed, scale);
                            generator.AddAsteroid(sys_idPlayer, PoserX, PoserY, "terrain", 1, 0, 0, { X: 0, Y: 0, Z: 0 });
                            //generator.AddSpaceObject(sys_idPlayer, PoserX, PoserY, "terrain", { class: "terrain" });  
                        }*/

















                        //generator.AddContainer(sys_idPlayer, coords.x, coords.y, "terrain", "droplist_empty");
                        //var id = generator.AddDecoration(sys_idPlayer, "terrain", coords, distance, rotation, rotationSpeed, scale);
                        //var id = generator.AddDecoration(sys_idPlayer, "broken_asteroid_01", { x: coords.x, y: coords.y }, 100, { x: 5, y: 5 }, { x: 5, y: 5 }, 2);
                        //var id = generator.AddDecoration(sys_idPlayer, "broken_asteroid_01", coords, distance, rotation, rotationSpeed, scale);

                        //generator.AddSpaceObject(sys_idPlayer, coords.x, coords.y, "terrain", { class: "terrain" });                 
                        //generator.AddContainer(sys_idPlayer, coords.x, coords.y, "terrain", "droplist_empty");









                        /*if (!generator.AreThereAnyObjects(sys_idPlayer, 50, 50, 10)) 
                        {
                            generator.AddAsteroid(sys_idPlayer, 50, 50, "terrain", 1, 0, { x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 });
                        }

                        //console.Print(PoserX + " _ " + PoserY);

                        //var rotation = { x: 100, y: 100, z: 100 };
                        //var rotationSpeed = { x: 100, y: 100, z: 100 };

                        //console.Print(id);
                        //var id = generator.AddContainer(sys_idPlayer, coords.x, coords.y, "crate_01", "droplist_empty");
                        //arrayOfTerrain[x * width + y] = 0;
                        //console.Print("generate terrain");
                    }
                }
                startOnce = false;
                //}
            }
        }*/