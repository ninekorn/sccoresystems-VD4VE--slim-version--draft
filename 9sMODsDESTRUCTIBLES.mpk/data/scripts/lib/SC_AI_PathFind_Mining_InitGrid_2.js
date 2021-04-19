using(console);
include(SC_PathFindUtilities_Mining_2.js);
include(SC_Utilities.js);

var switchForStationTiles = 0;
//var stationTilesArray = [];
//var initialPathfindStartingPos = [];
//var initialPathfindTargetPos = [];

var stationTiles;
var lastPlayerID;
var indexOfPlayer;
var globIndex;
var targetPos;
var initialPos;

var gcosty = 0;
var hcosty = 0;
var fcosty = 0;

var stationTilesArray = [];
var initialPathfindStartingPos = [];
var initialPathfindTargetPos = [];

var areThereAnyStations = -1;

var SC_AI_PathFind_Mining_InitGrid_2 =
{
    //gridWorldSize, nodeRadius, cData.lsp, cData.ltp, cData.stationTiles, cData.objt.sid, cData.lip, cData.glip, 0, cData,null
    npcBuildGrid: function (gridWorldSize, nodeRadius, indexOfGrid, cData, noder, stationTiles) {
        //var indexOfGrid = AIPathFindUtilities.getGridIndex(initialPos, cData.lsp, gridWorldSize, startGridPos);
        var nodeDiameter = nodeRadius * 2;

        var gridSizeXL = Math.round(gridWorldSize.xL * 1);
        var gridSizeYB = Math.round(gridWorldSize.yB * 1);

        var gridSizeXR = Math.round(gridWorldSize.xR * 1);
        var gridSizeYT = Math.round(gridWorldSize.yT * 1);

        var parenter = null;

        if (noder != null) {
            parenter = noder.parent;
        }

        var grid = [];
        var openSet = [];
        var closedSet = [];

        /*if (stationTiles != null)
        {
            console.PrintError("stationTiles exists");

            if (stationTiles.grid != null)
            {
                console.PrintError("stationTiles.grid exists");
                if (stationTiles.grid.length > 0)
                {
                    console.PrintError("stationTiles.grid.length exists");
                    switchForStationTiles = 1;
                }
                else
                {
                    console.PrintError("stationTiles.grid.length !exists");
                    switchForStationTiles = 0;
                }

            }
            else {
                console.PrintError("stationTiles.grid !exists");
                switchForStationTiles = 0;
            }

        }
        else {
            console.PrintError("stationTiles !exists");
            switchForStationTiles = 0;
        }*/



        if (storage.IsSetGlobal("GlobalIndex_Player_" + cData.objt.pName)) {
            //cData.iop = storage.GetGlobal("GlobalIndex_Player_" + cData.objt.pName);

            console.PrintError("player index: " + cData.iop);
            //console.PrintError(cData.sSwtc);

            if (storage.IsSetGlobal("station_tiles" + cData.SelectedStation))
            {
                console.PrintError("station tiles for base " + cData.SelectedStation + " -is- set." + " SC_AI_PathFind_Mining_InitGrid_Data_Mining_2.js");

                if (stationTilesArray[cData.iop] == null && cData.sSwtc == 1) {
                    stationTilesArray[cData.iop] = storage.GetGlobal("station_tiles" + cData.SelectedStation);

                    if (stationTilesArray[cData.iop] == null) {
                        console.PrintError("station tiles null00");
                    }
                    else {
                        console.PrintError("station tiles !null01");

                        //stationTiles = stationTilesArray[cData.iop];


                        cData.stRot = stationTilesArray[cData.iop].rot;
                        cData.stCoord = stationTilesArray[cData.iop].coord;
                        //cData.sSwtc = 2;
                        areThereAnyStations = 1;
                        switchForStationTiles = 1;
                    }
                }
                else if (stationTilesArray[cData.iop] != null && cData.sSwtc == 1) {
                    cData.stRot = stationTilesArray[cData.iop].rot;
                    cData.stCoord = stationTilesArray[cData.iop].coord;
                    //cData.sSwtc = 2;
                }
            }
            else if (!storage.IsSetGlobal("station_tiles" + cData.SelectedStation) || cData.SelectedStation == null || isNaN(cData.SelectedStation)) {
                // cData.SelectedStation
                if (!storage.IsSetGlobal("station_tiles" + cData.SelectedStation))
                {

                    switchForStationTiles = -1;
                    areThereAnyStations = -1;
                    console.PrintError("station tiles for base " + cData.SelectedStation + " -is not- set global." + " SC_AI_PathFind_Mining_InitGrid_Data_Mining_2.js");
                }
                else {

                    areThereAnyStations = 1;
                    switchForStationTiles = 1;
                    if (cData.SelectedStation == null || isNaN(cData.SelectedStation)) {
                        console.PrintError("station tiles for base " + cData.SelectedStation + " -is not- set." + " SC_AI_PathFind_Mining_InitGrid_Data_Mining_2.js");
                    }
                    else {
                        console.PrintError("station tiles for base " + cData.SelectedStation + " -is set global" + " SC_AI_PathFind_Mining_InitGrid_Data_Mining_2.js");//
                    }
                }

                //console.PrintError("station tiles for base " + cData.SelectedStation + " -is not- set.");

                //storage.SetGlobal("station_tiles" + cData.SelectedStation);
                //var dataOfStation = SC_Station_Tiles_Outpost_01.buildTiles(theBase, cData.SelectedStation);
                //storage.SetGlobal("station_tiles" + cData.SelectedStation, dataOfStation);	
            }
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


        /*
        if (initialPathfindStartingPos[globIndex] == null || cData.sSwtc == 2) {
            var initialPosX = Math.round(cData.nData.nCoord.x);
            var initialPosY = Math.round(cData.nData.nCoord.y);
            var initialPos = { x: initialPosX, y: initialPosY };

            var remnant = 360 - cData.stRot;

            initialPathfindStartingPos[globIndex] = SC_Utilities.RotatePoint(initialPos, stationTilesArray[cData.iop].coord, remnant);

            initialPathfindStartingPos[globIndex].x = Math.round(initialPathfindStartingPos[globIndex].x);
            initialPathfindStartingPos[globIndex].y = Math.round(initialPathfindStartingPos[globIndex].y);

            cData.lip = initialPathfindStartingPos[globIndex];
            cData.glip = initialPathfindStartingPos[globIndex];
            cData.lsp = initialPathfindStartingPos[globIndex];

            cData.sSwtc = 3;
        }
        else if (initialPathfindTargetPos[globIndex] == null || cData.sSwtc == 3) {
            var targetX = Math.round(cData.cFWP.x);
            var targetY = Math.round(cData.cFWP.y);
            var target = { x: targetX, y: targetY };

            var remnant = 360 - cData.stRot;
            initialPathfindTargetPos[globIndex] = SC_Utilities.RotatePoint(target, stationTilesArray[cData.iop].coord, remnant);

            initialPathfindTargetPos[globIndex].x = Math.round(initialPathfindTargetPos[globIndex].x);
            initialPathfindTargetPos[globIndex].y = Math.round(initialPathfindTargetPos[globIndex].y);

            cData.ltp = { x: initialPathfindTargetPos[globIndex].x, y: initialPathfindTargetPos[globIndex].y };
            cData.sSwtc = 0;
        }*/












        var xxx = 0;
        var yyy = 0;

        for (var x = -gridSizeXL; x <= gridSizeXR; x++) {
            for (var y = -gridSizeYB; y <= gridSizeYT; y++) {
                //grid.push(null);
                var xx = x;
                var yy = y;

                if (xx < 0) {
                    xx *= -1;
                    xx = (gridSizeXR) + xx;
                }
                if (yy < 0) {
                    yy *= -1;
                    yy = (gridSizeYT) + yy;
                }

                var boolWalker = 1;

                var roundedX = (Math.round(cData.glip.x)) + x;
                var roundedY = (Math.round(cData.glip.y)) + y;
                var rounded = { x: roundedX, y: roundedY };

                if (switchForStationTiles == 1)
                {
                    var coords = game.GetObjectCoordinates(cData.objt.sid, stationTilesArray[cData.iop].baseID);

                    //var remnant = 360 - cData.stationTilesArray[cData.iop].rot;
                    //coords= SC_Utilities.RotatePoint(target, cData.stationTilesArray[cData.iop].coord, remnant);

                    coords.x = Math.round(coords.x);
                    coords.y = Math.round(coords.y);

                    //var testX = coords.x + x;
                    //var testY = coords.y + y;

                    var diffX = Math.round(Math.abs(Math.abs(roundedX) - Math.abs(coords.x)));
                    var diffY = Math.round(Math.abs(Math.abs(roundedY) - Math.abs(coords.y)));

                    var test = { x: roundedX, y: roundedY };

                    if (test.x < coords.x) {
                        diffX *= -1;
                    }

                    if (test.y < coords.y) {
                        diffY *= -1;
                    }

                    if (test.x >= coords.x - stationTilesArray[cData.iop].widthL && test.x < coords.x + stationTilesArray[cData.iop].widthR && test.y >= coords.y - stationTilesArray[cData.iop].heightB && test.y < coords.y + stationTilesArray[cData.iop].heightT) {
                        if (diffX < 0) {
                            diffX = (stationTilesArray[cData.iop].widthR) + (diffX * -1);
                        }

                        if (diffY < 0) {
                            diffY = (stationTilesArray[cData.iop].heightT) + (diffY * -1);
                        }

                        var indexer = diffX + (stationTilesArray[cData.iop].widthL + stationTilesArray[cData.iop].widthR + 1) * diffY;

                        if (stationTilesArray[cData.iop].grid[indexer] == 0) {
                            boolWalker = 1;
                        }
                        else
                        {
                            if (stationTilesArray[cData.iop].visualTiles[indexer] == 1)
                            {
                                /*var idObj = generator.AddSpecialObject(cData.objt.sid, roundedX, roundedY, "stopSign_02", 0);

                                if (storage.IsSetGlobal("crates_Stop" + cData.objt.sid))
                                {
                                    var crates = storage.GetGlobal("crates_Stop" + cData.objt.sid);
                                    crates.push({ id: idObj, staID: stationTilesArray[cData.iop].baseID });
                                    storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
                                }
                                else
                                {
                                    var crates = [];
                                    crates.push({ id: idObj, staID: stationTilesArray[cData.iop].baseID });
                                    storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
                                }*/
                                stationTilesArray[cData.iop].visualTiles[indexer] = 2;
                            }

                            boolWalker = 0;
                        }
                    }
                }

                var worldPoint = { x: roundedX, y: roundedY };

                var index = xx * (gridWorldSize.xL + gridWorldSize.xR + 1) + yy;

                if (worldPoint.x === cData.lip.x && worldPoint.y === cData.lip.y) {
                    //console.PrintError("found start Node");
                    var startNodegcoster = 0;
                    var startNodehcoster = SC_PathFindUtilities_Mining_2.npcCheckDistance(worldPoint, cData.ltp);
                    var startNodefcoster = startNodegcoster + startNodehcoster;

                    grid[index] = { boolWalk: boolWalker, worldPosition: cData.lsp, gcost: startNodegcoster, hcost: startNodehcoster, fcost: startNodefcoster, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };;
                    openSet.push(grid[index]);
                }
                else if (cData.lsp.x === worldPoint.x && cData.lsp.y === worldPoint.y) {
                    //console.PrintError("new Grid");
                    gcosty = SC_PathFindUtilities_Mining_2.npcCheckDistance(cData.lsp, worldPoint);
                    hcosty = SC_PathFindUtilities_Mining_2.npcCheckDistance(worldPoint, cData.ltp);
                    fcosty = gcosty + hcosty;

                    grid[index] = { boolWalk: boolWalker, worldPosition: cData.lsp, gcost: gcosty, hcost: hcosty, fcost: fcosty, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };;
                    openSet.push(grid[index]);
                }
                else {
                    gcosty = SC_PathFindUtilities_Mining_2.npcCheckDistance(cData.lsp, worldPoint);
                    hcosty = SC_PathFindUtilities_Mining_2.npcCheckDistance(cData.ltp, worldPoint);
                    fcosty = gcosty + hcosty;

                    grid[index] = { boolWalk: boolWalker, worldPosition: worldPoint, gcost: gcosty, hcost: hcosty, fcost: fcosty, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };
                }


                /*if (worldPoint.x === initialPathfindStartingPos[globIndex].x && worldPoint.y === initialPathfindStartingPos[globIndex].y) {
                    console.PrintError("found start Node");
                    var startNodegcoster = 0;
                    var startNodehcoster = SC_PathFindUtilities_Mining_2.npcCheckDistance(worldPoint, initialPathfindTargetPos[globIndex]);
                    var startNodefcoster = startNodegcoster + startNodehcoster;
                    var startNode = { boolWalk: boolWalker, worldPosition: cData.lsp, gcost: startNodegcoster, hcost: startNodehcoster, fcost: startNodefcoster, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };

                    grid[index] = startNode;
                    openSet.push(startNode);
                }
                else if (cData.lsp.x === worldPoint.x && cData.lsp.y === worldPoint.y) {
                    console.PrintError("new Grid");
                    gcosty = SC_PathFindUtilities_Mining_2.npcCheckDistance(cData.lsp, worldPoint);
                    hcosty = SC_PathFindUtilities_Mining_2.npcCheckDistance(worldPoint, initialPathfindTargetPos[globIndex]);
                    fcosty = gcosty + hcosty;

                    var startNode = { boolWalk: boolWalker, worldPosition: cData.lsp, gcost: gcosty, hcost: hcosty, fcost: fcosty, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };

                    grid[index] = startNode;
                    openSet.push(startNode);
                }
                else {
                    gcosty = SC_PathFindUtilities_Mining_2.npcCheckDistance(cData.lsp, worldPoint);
                    hcosty = SC_PathFindUtilities_Mining_2.npcCheckDistance(initialPathfindTargetPos[globIndex], worldPoint);
                    fcosty = gcosty + hcosty;

                    grid[index] = { boolWalk: boolWalker, worldPosition: worldPoint, gcost: gcosty, hcost: hcosty, fcost: fcosty, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };
                }*/




                //index++;
                yyy++;
            }
            xxx++;
        }

        //cData.dog.grid = grid;
        //cData.dog.openSet = openSet;
        var data = { grid: grid, openSet: openSet };
        return data;
    }
};