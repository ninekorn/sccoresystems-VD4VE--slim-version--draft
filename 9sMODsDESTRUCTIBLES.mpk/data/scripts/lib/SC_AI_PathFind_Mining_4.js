using(console);
using(storage);

include(SC_AI_PathFind_Mining_NodeDistance_4.js);
include(SC_AI_PathFind_Mining_CheckAllSides_4.js);
include(SC_Utilities.js);
include(SC_AI_PathFind_Mining_Heap_4.js);
include(SC_AI_PathFind_Mining_InitGrid_4.js);



var lastDist;
var gridSizeX;
var gridSizeY;



//the main cData.log script where the .log array is accessed the most is in this script when sorting the neighboor tiles etc. maybe i forgot a "d" for last data of grid... i'm gonna loop back to renaming my variables one by one at least when all of the pathfind for those stations work. cData.dog is the 

var SC_AI_PathFind_Mining_4 =
{
    npcPathFind: function (gridWorldSize, nodeRadius, cData) //
    {
        // openSet, closedSet, targetPos, seekerPos, 
        var someExtratiles = [];

        if (cData.dog.openSet.length > 0)
		{
            gridSizeX = (gridWorldSize.x);
            gridSizeY = (gridWorldSize.y);

            cData.dog.openSet = SC_AI_PathFind_Mining_Heap_4.heapSort(cData.dog.openSet, cData.dog.openSet.length - 1);

            var node = cData.dog.openSet[0];
            cData.dog.openSet.splice(0, 1);
            cData.dog.node = node;

            cData.log[node.gridIndex].grid[node.index].closed = 1;

            var targetX = (cData.ltp.x);
            var targetY = (cData.ltp.y);

            var nodeX = (node.worldPosition.x);
            var nodeY = (node.worldPosition.y);

            var noderPos = { x: nodeX, y: nodeY };
            //var currentPoint = SC_Utilities.RotatePoint(noderPos, cData.stCoord, cData.stRot);
            //currentPoint.x = (currentPoint.x);
            //currentPoint.y = (currentPoint.y);

            if ((nodeX) == (targetX) && (nodeY) == (targetY)) //nodeX == targetX && nodeY == targetY // || (cData.lsp.x) == (targetX) && (cData.lsp.y) == (targetY)
            {
                var path = RetracePath(cData.lip, cData.ltp, node, cData.log, cData.DistanceToAsteroid);
                path.splice(path.length - 1, 1);
                path.splice(0, 1);
              
                /*for (var i = 0; i < path.length; i++) {
                    //path[i].worldPosition = SC_Utilities.RotatePoint(path[i].worldPosition, cData.stCoord, cData.stRot);  
                    var idObj = generator.AddSpecialObject(cData.objt.sid, path[i].worldPosition.x, path[i].worldPosition.y, "goSign_02", 0); //scrap_metal_00

                    if (storage.IsSetGlobal("signs_Go" + cData.objt.sid))
                    {
                        var crates = storage.GetGlobal("signs_Go" + cData.objt.sid);
                        crates.push(idObj);
                        storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                    }
                    else
                    {
                        var crates = [];
                        crates.push(idObj);
                        storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                    }
                }*/

                /*for (var i = 0; i < path.length - 1; i++)
                {
                    //path[i].worldPosition = SC_Utilities.RotatePoint(path[i].worldPosition, cData.stCoord, cData.stRot);  
                    var idObj = generator.AddSpecialObject(cData.objt.sid, path[i].worldPosition.x, path[i].worldPosition.y, "goSign_02", 0); //scrap_metal_00

                    if (storage.IsSetGlobal("signs_Go" + cData.objt.sid))
                    {
                        var crates = storage.GetGlobal("signs_Go" + cData.objt.sid);
                        crates.push(idObj);
                        storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                    }
                    else
                    {
                        var crates = [];
                        crates.push(idObj);
                        storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                    }
                }*/

                /*for (var i = path.length - 1; i < path.length; i++)
                {
                    if (storage.IsSetGlobal("signs_station_docking_points" + cData.objt.sid))
                    {
                        var crates = storage.GetGlobal("signs_station_docking_points" + cData.objt.sid);
                        crates.push(idObj);
                        storage.SetGlobal("signs_station_docking_points" + cData.objt.sid, crates);
                    }
                    else
                    {
                        var crates = [];
                        crates.push(idObj);
                        storage.SetGlobal("signs_station_docking_points" + cData.objt.sid, crates);
                    }
                }*/

                //console.PrintError("found Waypoint");

                var data = { openSet: cData.dog.openSet, currentCommand: 10, grid: cData.log[node.gridIndex].grid, path: path, node: node, iot: null };
                return data;
            }

            //------------------------------------
            //------------------------------------
            var someData = SC_AI_PathFind_Mining_CheckAllSides_4.checkAllSides(node, gridWorldSize, cData, cData.lip);
            //------------------------------------
            //------------------------------------

            if (someData.extraTiles != null) 
			{
                if (someData.extraTiles.length > 0) 
				{
                    someExtratiles = someData.extraTiles;
                    var sometester = [];
                    for (var i = 0; i < someExtratiles.length; i++)
                    {
                        cData.lsp = { x: (someExtratiles[i].sgp.x), y: (someExtratiles[i].sgp.y) };
                        cData.glip = { x: someExtratiles[i].docg.gridData.x, y: someExtratiles[i].docg.gridData.y };
   
                        var gridIndex = someExtratiles[i].docg.index;

                        if (cData.log[gridIndex] == null) {

                            //(gridWorldSize, nodeRadius, stationTiles, indexOfGrid, cData, noder)
                            //var pathData = SC_AI_PathFind_Mining_InitGrid_Data_4.npcGetGridData(gridWorldSize, nodeRadius, gridIndex, cData, node);
                            //var pathData = SC_AI_PathFind_Mining_InitGrid_4.npcGetGridData(gridWorldSize, nodeRadius, gridIndex, cData, node);
                            var pathData = SC_AI_PathFind_Mining_InitGrid_4.npcBuildGrid(gridWorldSize, nodeRadius, gridIndex, cData, node);



                            var data = pathData.openSet;

                            cData.dog.grid = pathData.grid;

                            var someDataOfGrid = { grid: cData.dog.grid };
                            cData.log[gridIndex] = someDataOfGrid;
                            sometester.push(data[0]);

                            continue;
                        }
                        else 
                        {
                            sometester.push(cData.log[gridIndex].grid[someExtratiles[i].iot]);
                        }
                    }
                    for (var i = 0; i < sometester.length; i++) {
                        var testerr = { swtc: 0, node: sometester[i], sgp: null, iot: null, iog: null };
                        someData.neighboors.push(testerr);
                    }
                }
            }

            /*var idObj = generator.AddSpecialObject(cData.objt.sid, node.worldPosition.x, node.worldPosition.y, "waypoint_00", 0); //scrap_metal_00
 
            if (storage.IsSetGlobal("crates_" + cData.objt.sid)) {
                var crates = storage.GetGlobal("crates_" + cData.objt.sid);
                crates.push(idObj);
                storage.SetGlobal("crates_" + cData.objt.sid, crates);
            }
            else {
                var crates = [];
                crates.push(idObj);
                storage.SetGlobal("crates_" + cData.objt.sid, crates);
            }*/

            if (someData.neighboors.length > 0) {

                for (var j = 0; j < someData.neighboors.length; j++)
                {
                    if (someData.neighboors[j].node != null)
                    {
                        if (cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].closed == 1 || someData.neighboors[j].node.walkable == 0) 
                        {

                            if (cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].closed == 1)
                            {
                                /*var idObj = generator.AddSpecialObject(cData.objt.sid, cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].worldPosition.x, cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].worldPosition.y, "waypoint_00", 0); //scrap_metal_00

                                if (storage.IsSetGlobal("crates_" + cData.objt.sid)) {
                                    var crates = storage.GetGlobal("crates_" + cData.objt.sid);
                                    crates.push(idObj);
                                    storage.SetGlobal("crates_" + cData.objt.sid, crates);
                                }
                                else {
                                    var crates = [];
                                    crates.push(idObj);
                                    storage.SetGlobal("crates_" + cData.objt.sid, crates);
                                }*/
                            }
                           
                            continue;
                        }

                        var gcost = node.gcost + SC_AI_PathFind_Mining_NodeDistance_4.checkNodeDistance(node.worldPosition, someData.neighboors[j].node.worldPosition);

                        if (gcost < someData.neighboors[j].node.gcost || cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].open == 0) 
                        {
                            someData.neighboors[j].node.gcost = gcost;
                            someData.neighboors[j].node.hcost = SC_AI_PathFind_Mining_NodeDistance_4.checkNodeDistance(someData.neighboors[j].node.worldPosition, cData.ltp);

                            someData.neighboors[j].node.fcost = someData.neighboors[j].node.gcost + someData.neighboors[j].node.hcost;

                            cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].parent = { iog: node.gridIndex, iot: node.index };

                            if (cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].open == 0 || cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].boolWalk == 0) 
                            { 
                                /*if (cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].open == 0)
                                {
                                    cData.dog.openSet.push(someData.neighboors[j].node);
                                    cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].open = 1;
                                }*/

                                if (cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].open == 0 && cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].boolWalk == 1)
                                {
                                    /*var idObj = generator.AddSpecialObject(cData.objt.sid, node.worldPosition.x, node.worldPosition.y, "waypoint_00", 0); //scrap_metal_00

                                    if (storage.IsSetGlobal("crates_" + cData.objt.sid)) {
                                        var crates = storage.GetGlobal("crates_" + cData.objt.sid);
                                        crates.push(idObj);
                                        storage.SetGlobal("crates_" + cData.objt.sid, crates);
                                    }
                                    else {
                                        var crates = [];
                                        crates.push(idObj);
                                        storage.SetGlobal("crates_" + cData.objt.sid, crates);
                                    }*/
                                    cData.dog.openSet.push(someData.neighboors[j].node);
                                    cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].open = 1;
                                }

                                if (cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].boolWalk == 0)
                                {
                                    /*var idObj = generator.AddSpecialObject(cData.objt.sid, node.worldPosition.x, node.worldPosition.y, "stopSign_02", 0); //scrap_metal_00

                                    if (storage.IsSetGlobal("crates_Stop" + cData.objt.sid)) {
                                        var crates = storage.GetGlobal("crates_Stop" + cData.objt.sid);
                                        crates.push(idObj);
                                        storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
                                    }
                                    else {
                                        var crates = [];
                                        crates.push(idObj);
                                        storage.SetGlobal("crates_Stop" + cData.objt.sid, crates);
                                    }*/



                                    /*var idObj = generator.AddSpecialObject(cData.objt.sid, roundedX, roundedY, "stopSign_02", 0);

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
                            }
                        }
                    }
                }
            }

            var data = { openSet: cData.dog.openSet, currentCommand: 0, grid: cData.log[node.gridIndex].grid, path: null, node: node, iot: null };
            return data;
        }
        else
        {
            console.PrintError("PathFind initiated without a grid or without something else");

            //var data = { openSet: cData.dog.openSet, currentCommand: 9, grid: null, path: null, node: null, iot: null };
            //return data;

            var data = { openSet: cData.dog.openSet, currentCommand: 0, grid: null, path: null, node: null, iot: null };
            return data;
        }
    }
};


var counting = 0;


function RetracePath(initialPos, targetPos, node, listOfGrids, DistanceToAsteroid) {
    var currentNode = node.parent;
    counting = 0;
    var path = [];

    var currentX = targetPos.x;
    var currentY = targetPos.y;

    var startX = (initialPos.x);
    var startY = (initialPos.y);

    var mainSwitch = 1;

    var lastNodeGridIndex;

    while (mainSwitch == 1) {
        path.unshift(node);

        currentX = (node.worldPosition.x);
        currentY = (node.worldPosition.y);

        if (node.parent != null) {
            var gridIndex = node.parent.iog;
            var nodeIndex = node.parent.iot;

            currentNode = listOfGrids[gridIndex].grid[nodeIndex];
            node = currentNode;
        }
        else {
            //console.PrintError("node.parent is NULL");
        }

        if (currentX == startX && currentY == startY) {
            mainSwitch = 0;
            break;
        }

        if (counting > 1500) //ORIGINAL 1500
        {
            console.PrintError("retrace path is returning a path that is very far. the array is very big. fix TODO");
            mainSwitch = 0;
            break;
        }

        counting++;
    }
    return path;

}