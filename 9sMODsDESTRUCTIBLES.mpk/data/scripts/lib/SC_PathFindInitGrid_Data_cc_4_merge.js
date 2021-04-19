using(console);

include(SC_Utilities.js);

var stationTilesArray = [];
var initialPathfindStartingPos = [];
var initialPathfindTargetPos = [];

var indexOfPlayer;
var globIndex;
var switchForStationTiles = 0;
var stationTilesArray = [];

var stationTiles;
var lastPlayerID;
var indexOfPlayer;
var targetPos;
var initialPos;

var gcosty = 0;
var hcosty = 0;
var fcosty = 0;

var nodeDiameter;

var gridSizeXL;
var gridSizeYB;

var gridSizeXR;
var gridSizeYT;

var parenter;

var grid = [];
var openSet = [];
var closedSet = [];

var xx;
var yy;

var boolWalker;
var arrayOfTurretsAtStation = [];

var roundedX;
var roundedY;
var rounded;
var coords;
var diffX;
var diffY;
var test;
var indexer;

var startNodegcoster;
var startNodehcoster;
var startNodefcoster;
var index;
var lastStationTilesIndex;
var splittedTilesOfStation;


var someDataToReturn;

var dontCountStationUnwalkableTiles = -1;



var swtch0 = 0;
var swtch1 = 0;
var swtch2 = 0;

var finalswtch = 0;

var PathfindDebugMovementSigns = 1;
//-1 don't add movement signs on the floor to visually see the pathfind.
//1 add movement signs on the floor to visually see the pathfind.
//0 and etc - unassigned



var SC_PathFindInitGrid_Data_cc_4_merge =
{
    npcGetGridDataMerged: function (gridWorldSize, nodeRadius, indexOfGrid, cData, noder, isStationTile) {

        grid = [];
        if (dontCountStationUnwalkableTiles == -1) {



            if (stationTilesArray[cData.iop] == null || cData.sSwtc == 1) {
                var current_base = cData.objt.bid;//ship.GetCurrentBase(cData.objt.bid);

                stationTilesArray[cData.iop] = storage.GetGlobal("station_tiles" + current_base);

                if (stationTilesArray[cData.iop] != null) {
                    //---console.PrintError("station tiles != null 0");
                    cData.stRot = stationTilesArray[cData.iop].rot;
                    cData.stCoord = stationTilesArray[cData.iop].coord;
                    cData.sSwtc = 2;
                }
                else // station tiles == null so the pathfind is for something different than trying to get to the station entrance.
                {
                    //---console.PrintError("station tiles == null 0");
                    cData.sSwtc = 3;
                }
            }
            else {
                if (stationTilesArray[cData.iop] != null) {
                    //---console.PrintError("station tiles != null 1");
                    cData.stRot = stationTilesArray[cData.iop].rot;
                    cData.stCoord = stationTilesArray[cData.iop].coord;
                }
                else // station tiles == null so the pathfind is for something different than trying to get to the station entrance.
                {
                    //---console.PrintError("station tiles == null 1");
                    cData.sSwtc = 3;
                }
            }
        }
        else {
            cData.sSwtc = 3;
        }











        /*if (stationTilesArray[cData.iop] != null) {
            //---console.PrintError("station tiles exist so npcBuildGrid");
            someDataToReturn = SC_PathFindInitGrid_cc_4.npcBuildGrid(gridWorldSize, nodeRadius, indexOfGrid, cData, noder, stationTilesArray[cData.iop]);
        }
        else // station tiles == null so the pathfind is for something different than trying to get to the station entrance.
        {
            someDataToReturn = SC_PathFindInitGrid_cc_4.npcBuildGrid(gridWorldSize, nodeRadius, indexOfGrid, cData, noder, null);
        }*/










        if (stationTilesArray[cData.iop] != null) {
            if (stationTilesArray[cData.iop].grid != null) {
                if (stationTilesArray[cData.iop].grid.length > 0) {
                    ////---console.PrintError("station TILES EXIST");
                    switchForStationTiles = 1;
                }
                else {
                    switchForStationTiles = -1;
                }
            }
            else {
                switchForStationTiles = -1;
            }
        }
        else {
            switchForStationTiles = -1;
        }

        if (cData.objt.formation == 1) {
            globIndex = storage.GetGlobal("maxDroneIndex0");
        }
        else if (cData.objt.formation == 2) {
            globIndex = storage.GetGlobal("maxDroneIndex1");
        }
        else if (cData.objt.formation == 3) {
            globIndex = storage.GetGlobal("maxDroneIndex2");
        }
        else if (cData.objt.formation == 4) {
            globIndex = storage.GetGlobal("maxDroneIndex3");
        }
        else if (cData.objt.formation == 5) {
            globIndex = storage.GetGlobal("maxDroneIndex4");
        }

        if (initialPathfindStartingPos[globIndex] == null || cData.sSwtc == 2) {
            //---console.PrintError("sSwtc == 2");
            var initialPosX = Math.round(cData.nData.nCoord.x);
            var initialPosY = Math.round(cData.nData.nCoord.y);
            var initialPos = { x: initialPosX, y: initialPosY };

            var remnant = 360 - cData.stRot;

            if (cData == null || cData.nData == null || cData.nData.nCoord == null) {
                //---console.PrintError("null0 SC_PathFindInitGrid_Data_cc_4_merge.js");
            }

            //initialPathfindStartingPos[globIndex] = initialPos;

            if (stationTilesArray[cData.iop] != null) {
                initialPathfindStartingPos[globIndex] = SC_Utilities.RotatePoint(initialPos, stationTilesArray[cData.iop].coord, remnant);
            }
            else // station tiles == null so the pathfind is for something different than trying to get to the station entrance.
            {
                initialPathfindStartingPos[globIndex] = SC_Utilities.RotatePoint(initialPos, cData.cFWP, remnant);
            }



            initialPathfindStartingPos[globIndex].x = Math.round(initialPathfindStartingPos[globIndex].x);
            initialPathfindStartingPos[globIndex].y = Math.round(initialPathfindStartingPos[globIndex].y);

            cData.lip = initialPathfindStartingPos[globIndex];
            cData.glip = initialPathfindStartingPos[globIndex];
            cData.lsp = initialPathfindStartingPos[globIndex];

            cData.sSwtc = 3;
        }













        if (initialPathfindTargetPos[globIndex] == null || cData.sSwtc == 3) {

            if (cData.sSwtc == 3) {
                //---console.PrintError("sSwtc == 3  SC_PathFindInitGrid_Data_cc_4_merge.js");

                var targetX = Math.round(cData.cFWP.x);
                var targetY = Math.round(cData.cFWP.y);
                var target = { x: targetX, y: targetY };

                var remnant = 360 - cData.stRot;

                if (cData == null || cData.nData == null || cData.nData.nCoord == null) {
                    //---console.PrintError("null1 SC_PathFindInitGrid_Data_cc_4.js");
                }

                if (stationTilesArray[cData.iop] != null) {
                    initialPathfindTargetPos[globIndex] = SC_Utilities.RotatePoint(target, stationTilesArray[cData.iop].coord, remnant);
                }
                else // station tiles == null so the pathfind is for something different than trying to get to the station entrance.
                {
                    initialPathfindTargetPos[globIndex] = SC_Utilities.RotatePoint(target, cData.cFWP, remnant);
                }

                //initialPathfindStartingPos[globIndex] = initialPos;

                initialPathfindTargetPos[globIndex].x = Math.round(initialPathfindTargetPos[globIndex].x);
                initialPathfindTargetPos[globIndex].y = Math.round(initialPathfindTargetPos[globIndex].y);

                cData.ltp = { x: initialPathfindTargetPos[globIndex].x, y: initialPathfindTargetPos[globIndex].y };
                cData.sSwtc = 0;
            }
            else {

                var targetX = Math.round(cData.cFWP.x);
                var targetY = Math.round(cData.cFWP.y);
                var target = { x: targetX, y: targetY };

                var remnant = 360 - cData.stRot;

                if (cData == null || cData.nData == null || cData.nData.nCoord == null) {
                    //---console.PrintError("null1 SC_PathFindInitGrid_Data_cc_4.js");
                }

                if (stationTilesArray[cData.iop] != null) {
                    initialPathfindTargetPos[globIndex] = SC_Utilities.RotatePoint(target, stationTilesArray[cData.iop].coord, remnant);
                }
                else // station tiles == null so the pathfind is for something different than trying to get to the station entrance.
                {
                    initialPathfindTargetPos[globIndex] = SC_Utilities.RotatePoint(target, cData.cFWP, remnant);
                }

                //initialPathfindStartingPos[globIndex] = initialPos;


                initialPathfindTargetPos[globIndex].x = Math.round(initialPathfindTargetPos[globIndex].x);
                initialPathfindTargetPos[globIndex].y = Math.round(initialPathfindTargetPos[globIndex].y);

                cData.ltp = { x: initialPathfindTargetPos[globIndex].x, y: initialPathfindTargetPos[globIndex].y };
                cData.sSwtc = 0;
            }




            /*var targetX = Math.round(cData.cFWP.x);
            var targetY = Math.round(cData.cFWP.y);
            var target = { x: targetX, y: targetY };

            var remnant = 360 - cData.stRot;

            if (cData == null || cData.nData == null || cData.nData.nCoord == null) {
                //---console.PrintError("null1 SC_PathFindInitGrid_Data_cc_4.js");
            }

            if (stationTilesArray[cData.iop] != null) {
                initialPathfindTargetPos[globIndex] = SC_Utilities.RotatePoint(target, stationTilesArray[cData.iop].coord, remnant);
            }
            else // station tiles == null so the pathfind is for something different than trying to get to the station entrance.
            {
                initialPathfindTargetPos[globIndex] = SC_Utilities.RotatePoint(target, cData.cFWP, remnant);
            }

            //initialPathfindStartingPos[globIndex] = initialPos;


            initialPathfindTargetPos[globIndex].x = Math.round(initialPathfindTargetPos[globIndex].x);
            initialPathfindTargetPos[globIndex].y = Math.round(initialPathfindTargetPos[globIndex].y);

            cData.ltp = { x: initialPathfindTargetPos[globIndex].x, y: initialPathfindTargetPos[globIndex].y };
            cData.sSwtc = 0;*/
        }











        nodeDiameter = nodeRadius * 2;

        gridSizeXL = (gridWorldSize.xL * 1);
        gridSizeYB = (gridWorldSize.yB * 1);

        gridSizeXR = (gridWorldSize.xR * 1);
        gridSizeYT = (gridWorldSize.yT * 1);

        parenter = null;

        if (noder != null) {
            parenter = noder.parent;
        }

        grid = [];
        openSet = [];
        closedSet = [];








        //RESETTING SWITCHES THAT DETERMINES IF THE GRID HAS AN OPENSET LENGTH > 0 FOR IT'S DATA VARIABLE REQUIREMENTS. RESETTING THOSE SWITCHES TO ZERO BEFORE THE GRID GCOST/FCOST/HCOST ARE ANALYSED.
        swtch0 = 0;
        swtch1 = 0;
        swtch2 = 0;
        finalswtch = 0;
        //RESETTING SWITCHES THAT DETERMINES IF THE GRID HAS AN OPENSET LENGTH > 0 FOR IT'S DATA VARIABLE REQUIREMENTS. RESETTING THOSE SWITCHES TO ZERO BEFORE THE GRID GCOST/FCOST/HCOST ARE ANALYSED.
        //var isStationTile = -1;


        /*if (isStationTile == 1) {
            isStationTile = 2;
        }
        else
        {
            isStationTile = 0;
        }*/


        var listOfUnwalkableLinksToOpenSet = [];

        for (var x = -gridSizeXL; x <= gridSizeXR; x++) {
            for (var y = -gridSizeYB; y <= gridSizeYT; y++) {

                boolWalker = 1;
                var open = 0;
                var closed = 0;
                /*if (isStationTile == 1)
                {
          
                    //isStationTile = 2;
                    boolWalker = 0;
                }*/

                //grid.push(null);
                xx = x;
                yy = y;

                if (xx < 0) {
                    xx *= -1;
                    xx = (gridSizeXR) + xx;
                }
                if (yy < 0) {
                    yy *= -1;
                    yy = (gridSizeYT) + yy;
                }

     

                roundedX = cData.glip.x + x;
                roundedY = cData.glip.y + y;

                var worldPoint = { x: roundedX, y: roundedY };

                if (switchForStationTiles == 1) {
                    var rotWorldForStation = SC_Utilities.RotatePoint(worldPoint, stationTilesArray[cData.iop].coord, stationTilesArray[cData.iop].rot);
                    var coords = game.GetObjectCoordinates(cData.objt.sid, stationTilesArray[cData.iop].baseID);

                    //var remnant = 360 - stationTilesArray[cData.iop] .rot;
                    //coords = SC_Utilities.RotatePoint(worldPoint, stationTilesArray[cData.iop] .coord, stationTilesArray[cData.iop] .rot);

                    var getSomeIndex = stationTilesArray[cData.iop].xml_id.substring(11, stationTilesArray[cData.iop].xml_id.length); //outpost_01_0
                    var parsedAngle = parseInt(getSomeIndex);
                    coords = SC_Utilities.RotatePoint(coords, stationTilesArray[cData.iop].coord, stationTilesArray[cData.iop].rot);

                    coords.x = Math.round(coords.x);
                    coords.y = Math.round(coords.y);

                    //var testX = coords.x + x;
                    //var testY = coords.y + y;

                    var diffX = Math.round(Math.abs(Math.abs(roundedX) - Math.abs(coords.x)));
                    var diffY = Math.round(Math.abs(Math.abs(roundedY) - Math.abs(coords.y)));

                    //var test = { x: roundedX, y: roundedY };

                    if (worldPoint.x < coords.x) {
                        diffX *= -1;
                    }

                    if (worldPoint.y < coords.y) {
                        diffY *= -1;
                    }

                    if (worldPoint.x >= coords.x - stationTilesArray[cData.iop].widthL && worldPoint.x < coords.x + stationTilesArray[cData.iop].widthR && worldPoint.y >= coords.y - stationTilesArray[cData.iop].heightB && worldPoint.y < coords.y + stationTilesArray[cData.iop].heightT) {
                        if (diffX < 0) {
                            diffX = (stationTilesArray[cData.iop].widthR) + (diffX * -1);
                        }

                        if (diffY < 0) {
                            diffY = (stationTilesArray[cData.iop].heightT) + (diffY * -1);
                        }

                        var indexer = diffX + (stationTilesArray[cData.iop].widthL + stationTilesArray[cData.iop].widthR + 1) * diffY;



                





                        if (stationTilesArray[cData.iop].grid[indexer] == 0) {
                            boolWalker = 1; //walkable
                        }
                        else {
                            if (stationTilesArray[cData.iop].visualTiles[indexer] == 1) {
                                if (PathfindDebugMovementSigns != -1) {
                                    /*var idObj = generator.AddSpecialObject(cData.objt.sid, rotWorldForStation.x, rotWorldForStation.y, "stopSign_00", 0);
                                    //var item_list = "droplist_empty";
                                    //var cont_id = generator.AddContainer(cData.objt.sid, rotWorldForStation.x, rotWorldForStation.y, "crate_02", item_list, { itemlist: item_list});

                                    if (storage.IsSetGlobal("crates_Stop" + cData.objt.sid)) {
                                        var crates = storage.GetGlobal("crates_Stop" + cData.objt.sid);
                                        crates.push({ id: idObj, staID: stationTilesArray[cData.iop].baseID });
                                        storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
                                    }
                                    else {
                                        var crates = [];
                                        crates.push({ id: idObj, staID: stationTilesArray[cData.iop].baseID });
                                        storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
                                    }*/















                                
                                }

                                stationTilesArray[cData.iop].visualTiles[indexer] = 2;
                            }

                            isStationTile = 1;
                            boolWalker = 0;  //unwalkable
                        }

                        if (grid[indexer] != null) {
                            if (grid[indexer].isStationTile == 2) {

                                grid[indexer].boolWalk = 0;
                                boolWalker = 0;
                            }

                            else {

                            }
                        }
                        else {

                        }


                    }
                }


                








                var newRoundedx = Math.round(roundedX);
                var newRoundedy = Math.round(roundedY);

                var currentWaypx = Math.round(cData.cFWP.x);
                var currentWaypy = Math.round(cData.cFWP.y);

                //TESTS
                //TESTS
                //TESTS


                if (cData.lip.x == cData.ltp.x && cData.lip.y == cData.ltp.y && boolWalker == 0) //cData.lip == last initial position
                {
                    //---console.PrintError("the initial position is the same as the target position. this pathfind was started with missing logic prior to this function call, for this scenario to not even arrive to this point maybe... checking.");


                    ////---console.PrintError("00 target waypoint is invalid. the formation of the drone has to be modified. The drone can keep knowledge of the formation point but offsetting the pathfinding backwards to find a walkable tile and using that should be my next step to make this work. Also, when that will be done, i need to make sure that if the drone is 1 tile diagonally or linearly next to a station, the pathfind might not have a valid openset and that might be fixed in the script SC_PathFindInitGrid_cc_4 or inside of SC_PathFind_cc_4.js");
                }

                var InitialPos = { x: cData.ltp.x, y: cData.ltp.y };
                InitialPos.x = Math.round(InitialPos.x);
                InitialPos.y = Math.round(InitialPos.y);

                var lastTargetPos = { x: cData.lip.x, y: cData.lip.y };
                lastTargetPos.x = Math.round(lastTargetPos.x);
                lastTargetPos.y = Math.round(lastTargetPos.y);

                if (InitialPos.x == lastTargetPos.x && InitialPos.y == lastTargetPos.y) //cData.lip == last initial position
                {
                    //---console.PrintError("### the initial position is the same as the target position. this pathfind was started with missing logic prior to this function call, for this scenario to not even arrive to this point maybe... checking. SC_PathFindInitGrid_Data_cc_4_merge.js");

                    ////---console.PrintError("00 target waypoint is invalid. the formation of the drone has to be modified. The drone can keep knowledge of the formation point but offsetting the pathfinding backwards to find a walkable tile and using that should be my next step to make this work. Also, when that will be done, i need to make sure that if the drone is 1 tile diagonally or linearly next to a station, the pathfind might not have a valid openset and that might be fixed in the script SC_PathFindInitGrid_cc_4 or inside of SC_PathFind_cc_4.js");
                }
                //TESTS
                //TESTS
                //TESTS
                worldPoint.x = Math.round(worldPoint.x);
                worldPoint.y = Math.round(worldPoint.y);


                var lastInitWaypx = Math.round(cData.lip.x);
                var lastInitWaypy = Math.round(cData.lip.y);
                var lastInitWaypgx = Math.round(cData.glip.x);
                var lastInitWaypgy = Math.round(cData.glip.y);

                var index = xx * (gridWorldSize.xL + gridWorldSize.xR + 1) + yy;
       
                if (cData.hasUnwalkableTiles == 1) {

                    //arrayOfTurretsAtStation.push({ loc: 7, id: id7, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });
                    //storage.SetGlobal("system_" + sys_id + "_base_" + base_id + "_turrets", arrayOfTurretsAtStation);

                    var current_base = cData.objt.bid;
                    //arrayOfTurretsAtStation[cData.iop] = storage.GetGlobal("system_" + cData.objt.sid + "_base_" + current_base + "_turrets");

                    if (cData.hasUnwalkableTilesArray != null) {

                        //console.PrintError("test0");
                        for (var i = 0; i < cData.hasUnwalkableTilesArray.length; i++) {
                            //console.PrintError("test1");
                            var unwalkabletilesPos = { x: cData.hasUnwalkableTilesArray[i].coords.x, y: cData.hasUnwalkableTilesArray[i].coords.y };
                            unwalkabletilesPos.x = Math.round(unwalkabletilesPos.x);
                            unwalkabletilesPos.y = Math.round(unwalkabletilesPos.y);

                            /*var rotWorldForStation = SC_Utilities.RotatePoint(worldPoint, unwalkabletilesPos, 0);
                            //var coords = game.GetObjectCoordinates(cData.objt.sid, stationTilesArray[cData.iop].baseID);

                            //var remnant = 360 - stationTilesArray[cData.iop] .rot;
                            //coords = SC_Utilities.RotatePoint(worldPoint, stationTilesArray[cData.iop] .coord, stationTilesArray[cData.iop] .rot);

                            var getSomeIndex = stationTilesArray[cData.iop].xml_id.substring(11, stationTilesArray[cData.iop].xml_id.length); //outpost_01_0
                            var parsedAngle = parseInt(getSomeIndex);

                            coords = SC_Utilities.RotatePoint(coords, stationTilesArray[cData.iop].coord, stationTilesArray[cData.iop].rot);

                            coords.x = Math.round(coords.x);
                            coords.y = Math.round(coords.y);*/






       

                            var diffUnwalkX = Math.round(Math.abs(Math.abs(worldPoint.x) - Math.abs(unwalkabletilesPos.x)));
                            var diffUnwalkY = Math.round(Math.abs(Math.abs(worldPoint.y) - Math.abs(unwalkabletilesPos.y)));

                            //if (worldPoint.x < unwalkabletilesPos.x) {
                            //    diffUnwalkX *= -1;
                            //}

                            //if (worldPoint.y < unwalkabletilesPos.y) {
                            //    diffUnwalkY *= -1;
                            //}
                            //var indexer = diffUnwalkX + (gridSizeXL + gridSizeXR + 1) * diffUnwalkY;


                            var xux = diffUnwalkX;
                            var yuy = diffUnwalkY;

                            if (xux < 0) {
                                xux *= -1;
                                xux = (gridSizeXR) + xux;
                            }
                            if (yuy < 0) {
                                yuy *= -1;
                                yuy = (gridSizeYT) + yuy;
                            }




                            var index = xux * (gridWorldSize.xL + gridWorldSize.xR + 1) + yuy;

                            /*if (worldPoint.x < unwalkabletilesPos.x) {
                                diffUnwalkX *= -1;
                            }

                            if (worldPoint.y < unwalkabletilesPos.y) {
                                diffUnwalkY *= -1;
                            }*/


                            if (diffUnwalkX < 0) {

                                diffUnwalkX = (gridSizeXR) + (diffUnwalkX * -1);
                            }

                            if (diffUnwalkY < 0) {
                                diffUnwalkY = (gridSizeYT) + (diffUnwalkY * -1);
                            }

                            var indexer = xux + (gridSizeXL + gridSizeXR + 1) * yuy;
   


                
                            if (worldPoint.x === unwalkabletilesPos.x && worldPoint.y === unwalkabletilesPos.y)
                            {
                                listOfUnwalkableLinksToOpenSet.push(indexer);
                                console.PrintError("unwalkable tile discovered " + " gindex " + indexer); //ship/npc/etc 
                                //grid[indexer] = { boolWalk: boolWalker, worldPosition: unwalkabletilesPos, gcost: 0, hcost: 0, fcost: 0, gridTileX: x, gridTileY: y, index: indexer, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0, isStationTile: isStationTile };
                                boolWalker = 0;
                                isStationTile = 2;
                                open = 0;
                                closed = 1;
                            }



                            /*
                            //if (worldPoint.x >= unwalkabletilesPos.x - gridSizeXL && worldPoint.x < unwalkabletilesPos.x + gridSizeXR && worldPoint.y >= unwalkabletilesPos.y - gridSizeYB && worldPoint.y < unwalkabletilesPos.y + gridSizeYT) {
                            //console.PrintError("unwalkable tile discovered " + " gindex " + index); //ship/npc/etc 
                            if (indexer < (gridSizeXL + gridSizeXR + 1) * (gridSizeXL + gridSizeXR + 1)) {

                                boolWalker = 0;
                                isStationTile = 2;

                                gcosty = SC_Utilities.GetDistance(cData.lsp, { x: roundedX, y: roundedY });
                                hcosty = SC_Utilities.GetDistance(cData.ltp, { x: roundedX, y: roundedY });
                                fcosty = gcosty + hcosty;

                                //console.PrintError("unwalkable tile discovered " + " gindex " + indexer); //ship/npc/etc 
                                //grid[indexer] = { boolWalk: boolWalker, worldPosition: unwalkabletilesPos, gcost: 0, hcost: 0, fcost: 0, gridTileX: x, gridTileY: y, index: indexer, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0, isStationTile: isStationTile };


                                //openSet[indexer] = grid[indexer];


                                var idObj = generator.AddSpecialObject(cData.objt.sid, unwalkabletilesPos.x, unwalkabletilesPos.y, "stopSign_00", 0);
                                //var item_list = "droplist_empty";
                                //var cont_id = generator.AddContainer(cData.objt.sid, rotWorldForStation.x, rotWorldForStation.y, "crate_02", item_list, { itemlist: item_list});

                                if (storage.IsSetGlobal("crates_Stop" + cData.objt.sid)) {
                                    var crates = storage.GetGlobal("crates_Stop" + cData.objt.sid);
                                    crates.push({ id: idObj, staID: stationTilesArray[cData.iop].baseID });
                                    storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
                                }
                                else {
                                    var crates = [];
                                    crates.push({ id: idObj, staID: stationTilesArray[cData.iop].baseID });
                                    storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
                                }
                            }*/
                        }
                    }
                    else // if (cData.hasUnwalkableTilesArray == null)
                    {

                    }

                    //cData.hasUnwalkableTiles = -1;
                    //cData.hasUnwalkableTilesArray = [];
                }


                //if (roundedX == lastInitWaypgx && roundedY == lastInitWaypgy )
                if (roundedX === lastInitWaypx && roundedY === lastInitWaypy) {
                    //---console.PrintError("found start Node merge.js");
                    startNodegcoster = 0;
                    startNodehcoster = SC_Utilities.GetDistance({ x: roundedX, y: roundedY }, cData.ltp);
                    startNodefcoster = startNodegcoster + startNodehcoster;

                    grid[index] = { boolWalk: boolWalker, worldPosition: cData.lsp, gcost: startNodegcoster, hcost: startNodehcoster, fcost: startNodefcoster, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: open, closed: closed, isStationTile: isStationTile };
                    openSet.push(grid[index]);
                    swtch0 = 1;
                }
                else if (cData.lsp.x === roundedX && cData.lsp.y === roundedY) {
                    //---console.PrintError("new Grid merge.js");
                    gcosty = SC_Utilities.GetDistance(cData.lsp, { x: roundedX, y: roundedY });
                    hcosty = SC_Utilities.GetDistance({ x: roundedX, y: roundedY }, cData.ltp);
                    fcosty = gcosty + hcosty;

                    grid[index] = { boolWalk: boolWalker, worldPosition: cData.lsp, gcost: gcosty, hcost: hcosty, fcost: fcosty, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: open, closed: closed, isStationTile: isStationTile };
                    openSet.push(grid[index]);
                    swtch1 = 1;
                }
                else {
                    gcosty = SC_Utilities.GetDistance(cData.lsp, { x: roundedX, y: roundedY });
                    hcosty = SC_Utilities.GetDistance(cData.ltp, { x: roundedX, y: roundedY });
                    fcosty = gcosty + hcosty;

                    grid[index] = { boolWalk: boolWalker, worldPosition: { x: roundedX, y: roundedY }, gcost: gcosty, hcost: hcosty, fcost: fcosty, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open:open, closed: closed, isStationTile: isStationTile };
                    swtch2 = 1;
                    //NOTHING IS ADDED IN OPENSET. NORMALLY, 
                }







                /*
                if (roundedX == cData.lip.x && roundedY == cData.lip.y)
                {
                    //---console.PrintError("found start Node merge.js");
                    startNodegcoster = 0;
                    startNodehcoster = SC_Utilities.GetDistance({ x: roundedX, y: roundedY }, cData.ltp);
                    startNodefcoster = startNodegcoster + startNodehcoster;

                    grid[index] = { boolWalk: boolWalker, worldPosition: cData.lsp, gcost: startNodegcoster, hcost: startNodehcoster, fcost: startNodefcoster, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };;
                    openSet.push(grid[index]);
                    swtch0 = 1;
                }
                else if (cData.lsp.x == roundedX && cData.lsp.y == roundedY)
                {
                    //---console.PrintError("new Grid merge.js");
                    gcosty = SC_Utilities.GetDistance(cData.lsp, { x: roundedX, y: roundedY });
                    hcosty = SC_Utilities.GetDistance({ x: roundedX, y: roundedY }, cData.ltp);
                    fcosty = gcosty + hcosty;

                    grid[index] = { boolWalk: boolWalker, worldPosition: cData.lsp, gcost: gcosty, hcost: hcosty, fcost: fcosty, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };;
                    openSet.push(grid[index]);
                    swtch1 = 1;
                }
                else
                {
                    gcosty = SC_Utilities.GetDistance(cData.lsp, { x: roundedX, y: roundedY });
                    hcosty = SC_Utilities.GetDistance(cData.ltp, { x: roundedX, y: roundedY });
                    fcosty = gcosty + hcosty;

                    grid[index] = { boolWalk: boolWalker, worldPosition: { x: roundedX, y: roundedY }, gcost: gcosty, hcost: hcosty, fcost: fcosty, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };
                    swtch2 = 1;
                    //NOTHING IS ADDED IN OPENSET. NORMALLY, 
                }*/



                var worldPointRoundedx = Math.round(worldPoint.x);
                var worldPointRoundedy = Math.round(worldPoint.y);


                var targetRoundedx = Math.round(cData.ltp.x);
                var targetRoundedy = Math.round(cData.ltp.y);

                if (worldPointRoundedx == targetRoundedx && worldPointRoundedy == targetRoundedy && boolWalker == 0 ||
                    roundedX == targetRoundedx && roundedY == targetRoundedy && boolWalker == 0) //cData.lip == last initial position
                {
                    ////---console.PrintError("target waypoint is invalid. the formation of the drone has to be modified. The drone can keep knowledge of the formation point but offsetting the pathfinding backwards to find a walkable tile and using that should be my next step to make this work. Also, when that will be done, i need to make sure that if the drone is 1 tile diagonally or linearly next to a station, the pathfind might not have a valid openset and that might be fixed in the script SC_PathFindInitGrid_cc_4 or inside of SC_PathFind_cc_4.js");
                    //---console.PrintError("**********found unwalkable target Node**********");

                    //finalswtch = -2;
                    finalswtch = 0;


                    //the problem is that the path cannot be retraced as the target is unwalkable and can never be reached.



                    //dontCountStationUnwalkableTiles = 1; //
                    //cData.mSwtc = 1;




                    //start an alternative pathfind where unwalkable station tiles aren't tagged unwalkable so that the path is calculated anyway and then making the difference between both pathfind paths and shrink the first pathfind to the path length of the second pathfind or something of that nature.
                    //TO READD
                    //TO READD
                    //TO READD
                    //cData.pfc = 2;
                    //cData.mSwtc = 1;
                    //TO READD
                    //TO READD
                    //TO READD


                    /*var startNodegcoster = 0;
                    var startNodehcoster = SC_Utilities.npcCheckDistance(worldPoint, cData.ltp);
                    var startNodefcoster = startNodegcoster + startNodehcoster;

                    grid[index] = { boolWalk: boolWalker4StationTiles, worldPosition: cData.lsp, gcost: startNodegcoster, hcost: startNodehcoster, fcost: startNodefcoster, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };;
                    openSet.push(grid[index]);*/


                    /*var idObj = generator.AddSpecialObject(cData.objt.sid, worldPoint.x, worldPoint.y, "stopSign_00", 0);

                    if (storage.IsSetGlobal("crates_Stop" + cData.objt.sid)) {
                        var crates = storage.GetGlobal("crates_Stop" + cData.objt.sid);
                        crates.push({ id: idObj, staID: stationTiles.baseID });
                        storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
                    }
                    else {
                        var crates = [];
                        crates.push({ id: idObj, staID: stationTiles.baseID });
                        storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
                    }*/

                }
                /*if (newRoundedx == currentWaypx && newRoundedy == currentWaypy && boolWalker == 0)
                 {
                     //---console.PrintError("**********found unwalkable target Node**********");
                     //startNodegcoster = 0;
                     //startNodehcoster = SC_Utilities.GetDistance({ x: roundedX, y: roundedY }, cData.ltp);
                     //startNodefcoster = startNodegcoster + startNodehcoster;

                     //grid[index] = { boolWalk: boolWalker, worldPosition: cData.lsp, gcost: startNodegcoster, hcost: startNodehcoster, fcost: startNodefcoster, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };;
                     //openSet.push(grid[index]);

                     finalswtch = -2;
                 }*/


            }
        }
        //var indexer = xux + (gridSizeXL + gridSizeXR + 1) * yuy;

                            //console.PrintError(index);

                            //grid[index].boolWalk = 0;

                            //worldPoint.x = unwalkabletilesPos.x;
                            //worldPoint.y = unwalkabletilesPos.y;
                            //if (diffUnwalkX == x && diffUnwalkY == y)
                            //{
                            //    //console.PrintError("0setting to unwalkable");
                            //    grid[index] = { boolWalk: boolWalker, worldPosition: { x: unwalkabletilesPos.x, y: unwalkabletilesPos.y }, gcost: gcosty, hcost: hcosty, fcost: fcosty, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };

                            //}

                            //if (unwalkabletilesPos.x == newRoundedx && unwalkabletilesPos.y == newRoundedy) //cData.lip == last initial position
                            //{
                            //   }

                            /*var idObj = generator.AddSpecialObject(cData.objt.sid, unwalkabletilesPos.x, unwalkabletilesPos.y, "stopSign_00", 0);
                            //var item_list = "droplist_empty";
                            //var cont_id = generator.AddContainer(cData.objt.sid, rotWorldForStation.x, rotWorldForStation.y, "crate_02", item_list, { itemlist: item_list});

                            if (storage.IsSetGlobal("crates_Stop" + cData.objt.sid)) {
                                var crates = storage.GetGlobal("crates_Stop" + cData.objt.sid);
                                crates.push({ id: idObj, staID: stationTilesArray[cData.iop].baseID });
                                storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
                            }
                            else {
                                var crates = [];
                                crates.push({ id: idObj, staID: stationTilesArray[cData.iop].baseID });
                                storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
                            }*/
                            //console.PrintError("1setting to unwalkable");



                            //console.PrintError("test2");
                            /*var idObj = generator.AddSpecialObject(cData.objt.sid, unwalkabletilesPos.x, unwalkabletilesPos.y, "stopSign_00", 0);
                            //var item_list = "droplist_empty";
                            //var cont_id = generator.AddContainer(cData.objt.sid, rotWorldForStation.x, rotWorldForStation.y, "crate_02", item_list, { itemlist: item_list});

                            if (storage.IsSetGlobal("crates_Stop" + cData.objt.sid)) {
                                var crates = storage.GetGlobal("crates_Stop" + cData.objt.sid);
                                crates.push({ id: idObj, staID: current_base });
                                storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
                            }
                            else {
                                var crates = [];
                                crates.push({ id: idObj, staID: current_base });
                                storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
                            }*/
                            //grid = [];
                            //if (worldPoint.x >= unwalkabletilesPos.x - gridSizeXL && worldPoint.x < unwalkabletilesPos.x + gridSizeXR && worldPoint.y >= unwalkabletilesPos.y - gridSizeYB && worldPoint.y < unwalkabletilesPos.y + gridSizeYT)
                            //if (diffUnwalkX >= -gridSizeXL && diffUnwalkX < gridSizeXR && diffUnwalkY >= -gridSizeYB && diffUnwalkY < gridSizeYT)
                            //{
                            //    boolWalker = 0;
                            //gcosty = SC_Utilities.GetDistance(cData.lsp, { x: unwalkabletilesPos.x, y: unwalkabletilesPos.y });
                            //hcosty = SC_Utilities.GetDistance(cData.ltp, { x: unwalkabletilesPos.x, y: unwalkabletilesPos.y });
                            //fcosty = gcosty + hcosty;
                            //    grid[index] = { boolWalk: boolWalker, worldPosition: unwalkabletilesPos, gcost: 0, hcost: 0, fcost: 0, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: -1, closed: 1 };

                            //console.PrintError("test2");
                            /*gcosty = SC_Utilities.GetDistance(cData.lsp, { x: unwalkabletilesPos.x, y: unwalkabletilesPos.y });
                            hcosty = SC_Utilities.GetDistance(cData.ltp, { x: unwalkabletilesPos.x, y: unwalkabletilesPos.y });
                            fcosty = gcosty + hcosty;
                            grid[index] = { boolWalk: boolWalker, worldPosition: unwalkabletilesPos, gcost: gcosty, hcost: hcosty, fcost: fcosty, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };
                            */

                            //console.PrintError("test2");
                            /*var idObj = generator.AddSpecialObject(cData.objt.sid, unwalkabletilesPos.x, unwalkabletilesPos.y, "stopSign_00", 0);
                            //var item_list = "droplist_empty";
                            //var cont_id = generator.AddContainer(cData.objt.sid, rotWorldForStation.x, rotWorldForStation.y, "crate_02", item_list, { itemlist: item_list});

                            if (storage.IsSetGlobal("crates_Stop" + cData.objt.sid)) {
                                var crates = storage.GetGlobal("crates_Stop" + cData.objt.sid);
                                crates.push({ id: idObj, staID: current_base });
                                storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
                            }
                            else {
                                var crates = [];
                                crates.push({ id: idObj, staID: current_base });
                                storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
                            }*/
                            //console.PrintError("tiles added to unwalkables 1 " + " id " + cData.objt.nid);


                            /*var lastInitWaypx = Math.round(cData.lip.x);
                            var lastInitWaypy = Math.round(cData.lip.y);

                            if (worldPoint.x == lastInitWaypx && worldPoint.y == lastInitWaypy) {
                                //console.PrintError("1found start Node merge.js");
                                startNodegcoster = 0;
                                startNodehcoster = SC_Utilities.GetDistance({ x: worldPoint.x, y: worldPoint.y }, cData.ltp);
                                startNodefcoster = startNodegcoster + startNodehcoster;

                                grid[index] = { boolWalk: boolWalker, worldPosition: cData.lsp, gcost: startNodegcoster, hcost: startNodehcoster, fcost: startNodefcoster, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };;
                                //openSet.push(grid[index]);
                                //swtch0 = 1;
                            }
                            else if (cData.lsp.x == worldPoint.x && cData.lsp.y == worldPoint.y) {
                                //console.PrintError("1new Grid merge.js");
                                gcosty = SC_Utilities.GetDistance(cData.lsp, { x: worldPoint.x, y: worldPoint.y });
                                hcosty = SC_Utilities.GetDistance({ x: worldPoint.x, y: worldPoint.y }, cData.ltp);
                                fcosty = gcosty + hcosty;

                                grid[index] = { boolWalk: boolWalker, worldPosition: cData.lsp, gcost: gcosty, hcost: hcosty, fcost: fcosty, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };;
                                //openSet.push(grid[index]);
                                //swtch1 = 1;
                            }
                            else {
                                //console.PrintError("NOTHING IS ADDED IN OPENSET");
                                gcosty = SC_Utilities.GetDistance(cData.lsp, { x: worldPoint.x, y: worldPoint.y });
                                hcosty = SC_Utilities.GetDistance(cData.ltp, { x: worldPoint.x, y: worldPoint.y });
                                fcosty = gcosty + hcosty;

                                grid[index] = { boolWalk: boolWalker, worldPosition: { x: worldPoint.x, y: worldPoint.y }, gcost: gcosty, hcost: hcosty, fcost: fcosty, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };
                                //swtch2 = 1;
                                //NOTHING IS ADDED IN OPENSET. NORMALLY, 
                            }*/
        /*
        if (unwalkabletilesPos.x == somecoordsX && unwalkabletilesPos.y == somecoordsY) //cData.lip == last initial position
        {
            

            /*
            //gridSizeXL// gridSizeXR
            //gridSizeYB// gridSizeYT

            for (var x = -1; x <= 1; x++) {
                for (var y = -1; y <= 1; y++) {
                    //console.PrintError("turret");
                    var newCoordsUnwalkables = unwalkabletilesPos;
                    newCoordsUnwalkables.x += x;
                    newCoordsUnwalkables.y += y;


                    var diffUnwalkX = Math.round(Math.abs(Math.abs(roundedX) - Math.abs(newCoordsUnwalkables.x)));
                    var diffUnwalkY = Math.round(Math.abs(Math.abs(roundedY) - Math.abs(newCoordsUnwalkables.y)));

                    //var test = { x: roundedX, y: roundedY };

                    if (worldPoint.x < newCoordsUnwalkables.x) {
                        diffUnwalkX *= -1;
                    }

                    if (worldPoint.y < newCoordsUnwalkables.y) {
                        diffUnwalkY *= -1;
                    }

                    if (worldPoint.x >= newCoordsUnwalkables.x - gridSizeXL && worldPoint.x < newCoordsUnwalkables.x + gridSizeXR && worldPoint.y >= newCoordsUnwalkables.y - gridSizeYB && worldPoint.y < newCoordsUnwalkables.y + gridSizeYT) {
                        if (diffUnwalkX < 0) {
                            diffUnwalkX = (gridSizeXR) + (diffUnwalkX * -1);
                        }

                        if (diffUnwalkY < 0) {
                            diffUnwalkY = (gridSizeYT) + (diffUnwalkY * -1);
                        }

                        var indexer = diffUnwalkX + (gridSizeXL + gridSizeXR + 1) * diffUnwalkY;





                        //console.PrintError("test2");
                        var idObj = generator.AddSpecialObject(cData.objt.sid, newCoordsUnwalkables.x, newCoordsUnwalkables.y, "stopSign_00", 0);
                        //var item_list = "droplist_empty";
                        //var cont_id = generator.AddContainer(cData.objt.sid, rotWorldForStation.x, rotWorldForStation.y, "crate_02", item_list, { itemlist: item_list});

                        if (storage.IsSetGlobal("crates_Stop" + cData.objt.sid)) {
                            var crates = storage.GetGlobal("crates_Stop" + cData.objt.sid);
                            crates.push({ id: idObj, staID: stationTilesArray[cData.iop].baseID });
                            storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
                        }
                        else {
                            var crates = [];
                            crates.push({ id: idObj, staID: stationTilesArray[cData.iop].baseID });
                            storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
                        }
                        //console.PrintError("tiles added to unwalkables 1 " + " id " + cData.objt.nid);


                        boolWalker = 0;
                        var lastInitWaypx = Math.round(cData.lip.x);
                        var lastInitWaypy = Math.round(cData.lip.y);

                        var index = xx * (gridWorldSize.xL + gridWorldSize.xR + 1) + yy;


                        if (newCoordsUnwalkables.x == lastInitWaypx && newCoordsUnwalkables.y == lastInitWaypy) {
                            //---console.PrintError("found start Node merge.js");
                            startNodegcoster = 0;
                            startNodehcoster = SC_Utilities.GetDistance({ x: newCoordsUnwalkables.x, y: newCoordsUnwalkables.y  }, cData.ltp);
                            startNodefcoster = startNodegcoster + startNodehcoster;

                            grid[index] = { boolWalk: boolWalker, worldPosition: cData.lsp, gcost: startNodegcoster, hcost: startNodehcoster, fcost: startNodefcoster, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };;
                            openSet.push(grid[index]);
                            swtch0 = 1;
                        }
                        else if (cData.lsp.x == newCoordsUnwalkables.x && cData.lsp.y == newCoordsUnwalkables.y ) {
                            //---console.PrintError("new Grid merge.js");
                            gcosty = SC_Utilities.GetDistance(cData.lsp, { x: newCoordsUnwalkables.x, y: newCoordsUnwalkables.y  });
                            hcosty = SC_Utilities.GetDistance({ x: newCoordsUnwalkables.x, y: newCoordsUnwalkables.y  }, cData.ltp);
                            fcosty = gcosty + hcosty;

                            grid[index] = { boolWalk: boolWalker, worldPosition: cData.lsp, gcost: gcosty, hcost: hcosty, fcost: fcosty, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };;
                            openSet.push(grid[index]);
                            swtch1 = 1;
                        }
                        else {
                            gcosty = SC_Utilities.GetDistance(cData.lsp, { x: newCoordsUnwalkables.x, y: newCoordsUnwalkables.y  });
                            hcosty = SC_Utilities.GetDistance(cData.ltp, { x: newCoordsUnwalkables.x, y: newCoordsUnwalkables.y  });
                            fcosty = gcosty + hcosty;

                            grid[index] = { boolWalk: boolWalker, worldPosition: { x: newCoordsUnwalkables.x, y: newCoordsUnwalkables.y  }, gcost: gcosty, hcost: hcosty, fcost: fcosty, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };
                            swtch2 = 1;
                            //NOTHING IS ADDED IN OPENSET. NORMALLY, 
                        }
 
                    }
                }
            }*/







        /*//console.PrintError("test2");
        var idObj = generator.AddSpecialObject(cData.objt.sid, unwalkabletilesPos.x, unwalkabletilesPos.y, "stopSign_00", 0);
        //var item_list = "droplist_empty";
        //var cont_id = generator.AddContainer(cData.objt.sid, rotWorldForStation.x, rotWorldForStation.y, "crate_02", item_list, { itemlist: item_list});

        if (storage.IsSetGlobal("crates_Stop" + cData.objt.sid)) {
            var crates = storage.GetGlobal("crates_Stop" + cData.objt.sid);
            crates.push({ id: idObj, staID: stationTilesArray[cData.iop].baseID });
            storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
        }
        else {
            var crates = [];
            crates.push({ id: idObj, staID: stationTilesArray[cData.iop].baseID });
            storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
        }
        //console.PrintError("tiles added to unwalkables 1 " + " id " + cData.objt.nid);
        boolWalker = 0;

    }*/


        /*for (var j = 0; j < openSet.length; j++) {
            //var opensetPos = { x: openSet[j].worldPosition.x, y: openSet[j].worldPosition.y };
            //opensetPos.x = Math.round(opensetPos.x);
            //opensetPos.y = Math.round(opensetPos.y);

            if (unwalkabletilesPos.x == opensetPos.x && unwalkabletilesPos.y == opensetPos.y) //cData.lip == last initial position
            {
                console.PrintError("tiles added to unwalkables");
                openSet[j].boolWalker = 0;

            }
        }*/



        /*if (cData.hasUnwalkableTiles == 1) {


            //arrayOfTurretsAtStation.push({ loc: 7, id: id7, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });
            //storage.SetGlobal("system_" + sys_id + "_base_" + base_id + "_turrets", arrayOfTurretsAtStation);

            var current_base = cData.objt.bid;
            arrayOfTurretsAtStation[cData.iop] = storage.GetGlobal("system_" + cData.objt.sid + "_base_" + current_base + "_turrets");

            if (arrayOfTurretsAtStation[cData.iop] != null) {

                for (var i = 0; i < arrayOfTurretsAtStation.length; i++) {

                    var unwalkabletilesPos = { x: arrayOfTurretsAtStation[i].coordsX, y: arrayOfTurretsAtStation[i].coordsY };
                    unwalkabletilesPos.x = Math.round(unwalkabletilesPos.x);
                    unwalkabletilesPos.y = Math.round(unwalkabletilesPos.y);

                    for (var j = 0; j < openSet.length; j++) {
                        var opensetPos = { x: openSet[j].worldPosition.x, y: openSet[j].worldPosition.y };
                        opensetPos.x = Math.round(opensetPos.x);
                        opensetPos.y = Math.round(opensetPos.y);

                        if (unwalkabletilesPos.x == opensetPos.x && unwalkabletilesPos.y == opensetPos.y) //cData.lip == last initial position
                        {
                            console.PrintError("tiles added to unwalkables");
                            openSet[j].boolWalker = 0;

                        }
                    }
                }
            }
            else // station tiles == null so the pathfind is for something different than trying to get to the station entrance.
            {

            }

            cData.hasUnwalkableTiles = -1;

        }*/



        if (swtch0 == 0 && swtch1 == 0 && swtch2 == 1) {
            //console.PrintError("***00 FIXTHIS***this grid has no valid openset.***FIXTHIS***");
            finalswtch = 1;
        }


        if (swtch0 == 0 && swtch1 == 1 && swtch2 == 1) {
            //nothing to fix here. the initial position is the position of the drone but why is it null
            //console.PrintError("***11 FIXTHIS***this grid has no valid initial position probably because it's starting position is unwalkable.***FIXTHIS***"); //  
            finalswtch = 2;
        }


        /*for (var j = 0; j < listOfUnwalkableLinksToOpenSet.length; j++)
        {
            for (var i = 0; i < openSet.length; i++) {

                if (openSet[i].index === listOfUnwalkableLinksToOpenSet[j]) {
                    openSet[i].boolWalk = 0;
                    //openSet[i].closed = 1;
                    openSet[i].isStationTile = 2;

                    grid[openSet[i].index].boolWalk = 0;
                    //grid[openSet[i].index].closed = 1;
                    grid[openSet[i].index].isStationTile = 2;
                    finalswtch = -2;
                }
            }
        }*/

        

        //---console.PrintError("switches " + " swtch0 = " + swtch0 + " swtch1 = " + swtch1 + " swtch2 = " + swtch2);


        /*if (stationTilesArray != null) {

            if (stationTilesArray[cData.iop] != null) {
                var current_base = cData.objt.bid;//ship.GetCurrentBase(cData.objt.bid);
                storage.SetGlobal("station_tiles" + current_base, stationTilesArray[cData.iop]);
            }
        }*/

        //cData.hasUnwalkableTilesArray = [];
        cData.hasUnwalkableTiles = -1;

        return { grid: grid, openSet: openSet, fSwtch: finalswtch };
    }
};