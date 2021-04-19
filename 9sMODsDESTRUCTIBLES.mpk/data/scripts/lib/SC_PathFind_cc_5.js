using(console);
using(storage);

include(SC_PathFindUtilities_cc_5.js);
include(SC_PathFindCheckAllSides_cc_5.js);
include(SC_Utilities.js);
include(SC_Pathfind_Heap_cc_5.js);
include(SC_PathFindInitGrid_Data_cc_5_merge.js);

var lastDist;
var gridSizeX;
var gridSizeY;

var unwalkableTargetTile = -1;
var node;

var PathfindDebugMovementSigns = 1; //discovery tiles
//-1 don't add movement signs on the floor to visually see the pathfind.
//1 add movement signs on the floor to visually see the pathfind.
//0 and etc - unassigned

var PathfindDebugMovementPathFoundSigns = 1;  //found path green light
//-1 don't add movement signs on the floor to visually see the pathfind.
//1 add movement signs on the floor to visually see the pathfind.
//0 and etc - unassigned

var maxRetracePath = 150;
var maxOpensetLengthWhenThePathfindLoops = 25;

//var frameCounterArrayResetVisualTiles = 0;
var maxFrameCounterArrayResetVisualTiles = 25;

var pathfindCounter = 0;
var pathfindCounterMax = 25;

var initpathfindCounterSwtch = 1;

var SC_PathFind_cc_5 =
{
    npcPathFind: function (gridWorldSize, nodeRadius, cData, forceUseRetracePathSwtc, pathfindCounterSwtch) //
    {

        var globIndex = parseInt(npc.GetTag(cData.objt.nid, "droneIndex"));
        /*if (frameCounterArrayResetVisualTiles > maxFrameCounterArrayResetVisualTiles) {


            frameCounterArrayResetVisualTiles = 0;
        }*/

        if (pathfindCounterSwtch == 0) {
            if (initpathfindCounterSwtch == 1) {
                initpathfindCounterSwtch = 0;
            }
        }




        unwalkableTargetTile = -1;


     

        // openSet, closedSet, targetPos, seekerPos, 

        //console.PrintError("FramePathfindIteratedCounterMax " + FramePathfindIteratedCounterMax);
        if (cData.dog.openSet.length > 0) {
            if (cData.dog.openSet.length > maxOpensetLengthWhenThePathfindLoops*FramePathfindIteratedCounterMax) // the pathfind openSet of new tiles is starting to get bigger and bigger and the target tile isn't being found. stop the pathfind for now, to fix later.
            {
                //to readd for tests
                console.PrintError("((((((frame is coming here in pathfind script.))))) " + cData.objt.formation);
                //to readd for tests

                gridSizeX = (gridWorldSize.x);
                gridSizeY = (gridWorldSize.y);

                cData.dog.openSet = SC_Pathfind_Heap_cc_5.heapSort(cData.dog.openSet, cData.dog.openSet.length - 1, "fcost"); //original hcost

                node = cData.dog.openSet[0];
                cData.dog.openSet.splice(0, 1);
                cData.dog.node = node;

                cData.log[node.gridIndex].grid[node.index].closed = 1;

                var targetX = (cData.ltp.x);
                var targetY = (cData.ltp.y);

                var nodeX = (node.worldPosition.x);
                var nodeY = (node.worldPosition.y);

                var noderPos = { x: nodeX, y: nodeY };



                //if (isNaN(noderPos) || isNaN(cData.stCoord) || isNaN(cData.stRot)) {
                //    //console.PrintError("null" + " " +  isNaN(noderPos) + " " + isNaN(cData.stCoord) + " " +  isNaN(cData.stRot));
                //}



                var remnants = 360 - cData.stRot;
                var currentPoint = { x: 0, y: 0 };


                var current_base = cData.objt.bid;
                if (current_base != null) {
                    currentPoint = cData.cFWP;//SC_Utilities.RotatePoint(noderPos, cData.stCoord, cData.stRot); //
                }
                else {
                    currentPoint = SC_Utilities.RotatePoint(noderPos, cData.stCoord, cData.stRot); //
                }
                //var currentPoint = SC_Utilities.RotatePoint(noderPos, cData.stCoord, cData.stRot); //

                //currentPoint.x = (currentPoint.x);
                //currentPoint.y = (currentPoint.y);
                forceUseRetracePathSwtc = 1;

                //console.PrintError("forceUseRetracePathSwtc " + forceUseRetracePathSwtc + " " + cData.objt.formation);
                //if ((nodeX) == (targetX) && (nodeY) == (targetY) || forceUseRetracePathSwtc == 1) //nodeX == targetX && nodeY == targetY // || (cData.lsp.x) == (targetX) && (cData.lsp.y) == (targetY)
                //{
                var path = RetracePath(cData.lip, cData.ltp, node, cData.log);

                if (path.path == null || path.path.length < 1) {
                    console.PrintError("&&&&&&&&&&&NULLLLLLLL&&&&&&&&&&&&&");

                    var data = { openSet: null, currentCommand: -2, grid: cData.log[node.gridIndex].grid, path: path.path, node: node, iot: null, PathfindCounter: pathfindCounter, PathfindCounterSwtch: pathfindCounterSwtch };
                    return data;
                }
                else {


                    console.PrintError("found path " + path.path.length + " cData.dog.openSet.length > maxOpensetLengthWhenThePathfindLoops");
                    /*if (path.length != 1) {
                        path.splice(0, 1); // splicing index 0 initial position so that the index 1 is the first point the drone tries to reach.

                    } */            
                    for (var i = 0; i < path.path.length; i++) {

                        if (!isNaN(cData.stCoord) && !isNaN(cData.stRot)) {

                            path.path[i].worldPosition = SC_Utilities.RotatePoint(path.path[i].worldPosition, cData.stCoord, -cData.stRot);
                        }

                        //TOREADD
                        //TOREADD
                        //TOREADD
                        //TOREADD
                        //TOREADD
                        //TOREADD
                        /*var idObj = generator.AddSpecialObject(cData.objt.sid, path[i].worldPosition.x, path[i].worldPosition.y, "goSign_00", 0); //scrap_metal_00
                        //var item_list = "droplist_empty";
                        //var idObj = generator.AddContainer(cData.objt.sid, path[i].worldPosition.x, path[i].worldPosition.y, "DiscoveryTile", item_list, { itemlist: item_list });
 
                        //console.PrintError("globIndex: " + globIndex + " sys_id: " + sys_id);
                        if (storage.IsSetGlobal("index_" + globIndex + "_sid_" + sys_id + "_frame_" + "signs_Go")) {
                            var crates = storage.GetGlobal("index_" + globIndex + "_sid_" + sys_id + "_frame_" + "signs_Go");
                            //crates.push(idObj);
                            crates.push({ id: idObj, globIndex: globIndex, sys_id: sys_id });
                            storage.SetGlobal("index_" + globIndex + "_sid_" + sys_id + "_frame_" + "signs_Go", crates);
                        }
                        else {
                            var crates = [];
                            crates.push({ id: idObj, globIndex: globIndex, sys_id: sys_id });
                            storage.SetGlobal("index_" + globIndex + "_sid_" + sys_id + "_frame_" + "signs_Go", crates);
                        }*/
                        //TOREADD
                        //TOREADD
                        //TOREADD

                    }
                    var coms = 0;
                    if (path.isMaxed == 2) {
                        coms = 2;
                    }
                    else {
                        coms = 10;
                    }

                    cData.lnode = node;

                    pathfindCounter = 0;
                    ////console.PrintError("33cData.cFWPD: " + cData.cFWPD + " cData.mSwtc " + cData.mSwtc + " cData.pfc: " + cData.pfc + " SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.js ");
                    var data = { openSet: cData.dog.openSet, currentCommand: coms, grid: cData.log[node.gridIndex].grid, path: path.path, node: node, iot: null, PathfindCounter: pathfindCounter, PathfindCounterSwtch: pathfindCounterSwtch };
                    return data;
                }
                    //TO READD
                    //TO READD
                    //to readd for picking objects up or levers maybe or asteroids maybe.
                    /*if (!isNaN(cData.stCoord) && !isNaN(cData.stRot))
                    {
                        path[path.length - 1].worldPosition = SC_Utilities.RotatePoint(path[path.length - 1].worldPosition, cData.stCoord, cData.stRot);
                    }
                    path.splice(path.length - 1, 1); // splicing the last node but i might readd it later if it helps.*/
                    //TO READD
                    //TO READD


                //}









                /*
                 //console.PrintError("44cData.cFWPD: " + cData.cFWPD + " cData.mSwtc " + cData.mSwtc + " cData.pfc: " + cData.pfc + " SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.js ");
          
                 var data = { openSet: cData.dog.openSet, currentCommand: 5, grid: cData.log[node.gridIndex].grid, path: path, node: node, iot: null, PathfindCounter: pathfindCounter, PathfindCounterSwtch: pathfindCounterSwtch };
                 pathfindCounter++;
                 return data;*/
            }
            else //if (cData.dog.openSet.length > maxOpensetLengthWhenThePathfindLoops*FramePathfindIteratedCounterMax)
            {

               


                gridSizeX = (gridWorldSize.x);
                gridSizeY = (gridWorldSize.y);

                cData.dog.openSet = SC_Pathfind_Heap_cc_5.heapSort(cData.dog.openSet, cData.dog.openSet.length - 1, "fcost"); //original hcost


                /*if (forceUseRetracePathSwtc != 2) {
                    node = cData.dog.openSet[0];
                    cData.dog.openSet.splice(0, 1);
                    cData.dog.node = node;

                }
                else {
                    cData.dog.node = cData.dog.openSet[0];
                }*/

                node = cData.dog.openSet[0];
                cData.dog.openSet.splice(0, 1);
                cData.dog.node = node;

              
                cData.log[node.gridIndex].grid[node.index].closed = 1;

                var targetX = (cData.ltp.x);
                var targetY = (cData.ltp.y);

                var nodeX = (node.worldPosition.x);
                var nodeY = (node.worldPosition.y);

                var noderPos = { x: nodeX, y: nodeY };



                //if (isNaN(noderPos) || isNaN(cData.stCoord) || isNaN(cData.stRot)) {
                //    //console.PrintError("null" + " " +  isNaN(noderPos) + " " + isNaN(cData.stCoord) + " " +  isNaN(cData.stRot));
                //}



                var remnants = 360 - cData.stRot;
                var currentPoint = { x: 0, y: 0 };


                var current_base = cData.objt.bid;
                if (current_base != null) {
                    currentPoint = cData.cFWP;//SC_Utilities.RotatePoint(noderPos, cData.stCoord, cData.stRot); //
                }
                else {
                    currentPoint = SC_Utilities.RotatePoint(noderPos, cData.stCoord, cData.stRot); //
                }
                //var currentPoint = SC_Utilities.RotatePoint(noderPos, cData.stCoord, cData.stRot); //

                //currentPoint.x = (currentPoint.x);
                //currentPoint.y = (currentPoint.y);

                if ((nodeX) == (targetX) && (nodeY) == (targetY) || forceUseRetracePathSwtc == 1) //nodeX == targetX && nodeY == targetY // || (cData.lsp.x) == (targetX) && (cData.lsp.y) == (targetY)
                {

                    var path = RetracePath(cData.lip, cData.ltp, node, cData.log);


                    console.PrintError("found path " + path.path.length);
                    /*if (path.length > 0) {
                        path[path.length - 1].worldPosition = SC_Utilities.RotatePoint(path[path.length - 1].worldPosition, cData.stCoord, cData.stRot);
    
    
    
                        var idObj = generator.AddSpecialObject(cData.objt.sid, path[path.length - 1].worldPosition.x, path[path.length - 1].worldPosition.y, "DiscoveryTile", 0); //scrap_metal_00
    
                        if (storage.IsSetGlobal("signs_Go" + cData.objt.sid)) {
                            var crates = storage.GetGlobal("signs_Go" + cData.objt.sid);
                            crates.push(idObj);
                            storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                        }
                        else {
                            var crates = [];
                            crates.push(idObj);
                            storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                        }
                        path.splice(path.length - 1, 1); // splicing the last node but i might readd it later if it helps.
    
                        if (path.length > 0) {
                            path[0].worldPosition = SC_Utilities.RotatePoint(path[0].worldPosition, cData.stCoord, cData.stRot);
                            var idObj = generator.AddSpecialObject(cData.objt.sid, path[0].worldPosition.x, path[0].worldPosition.y, "DiscoveryTile", 0); //scrap_metal_00
    
                            if (storage.IsSetGlobal("signs_Go" + cData.objt.sid)) {
                                var crates = storage.GetGlobal("signs_Go" + cData.objt.sid);
                                crates.push(idObj);
                                storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                            }
                            else {
                                var crates = [];
                                crates.push(idObj);
                                storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                            }
                            path.splice(0, 1); // splicing index 0 initial position so that the index 1 is the first point the drone tries to reach.
                        }*/

                    /*if (path.length > 0) {
                        for (var i = 0; i < path.length; i++) {
 
                            path[i].worldPosition = SC_Utilities.RotatePoint(path[i].worldPosition, cData.stCoord, cData.stRot);
 
                            var idObj = generator.AddSpecialObject(cData.objt.sid, path[i].worldPosition.x, path[i].worldPosition.y, "DiscoveryTile", 0); //scrap_metal_00
 
                            if (storage.IsSetGlobal("signs_Go" + cData.objt.sid)) {
                                var crates = storage.GetGlobal("signs_Go" + cData.objt.sid);
                                crates.push(idObj);
                                storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                            }
                            else {
                                var crates = [];
                                crates.push(idObj);
                                storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                            }
                        }
                    }
 
 
                    var data = { openSet: cData.dog.openSet, currentCommand: 10, grid: cData.log[node.gridIndex].grid, path: path, node: node, iot: null };
                    return data;*/
                    /*}
                    else
                    {
                        //---////console.PrintError("the path is null. SC_PathFind_cc_3.js");
                    }*/
                    ////---////console.PrintError("found Waypoint sc_pathfind_cc_3.js"); 

                    if (path.path == null || path.path.length < 1) {
                        //console.PrintError("&&&&&&&&&&&NULLLLLLLL&&&&&&&&&&&&&");
                    }

                    //TO READD
                    //TO READD
                    //to readd for picking objects up or levers maybe or asteroids maybe.
                    /*if (!isNaN(cData.stCoord) && !isNaN(cData.stRot))
                    {
                        path[path.length - 1].worldPosition = SC_Utilities.RotatePoint(path[path.length - 1].worldPosition, cData.stCoord, cData.stRot);
                    }
                    path.splice(path.length - 1, 1); // splicing the last node but i might readd it later if it helps.*/
                    //TO READD
                    //TO READD





                    /*var idObj = generator.AddSpecialObject(cData.objt.sid, path[path.length - 1].worldPosition.x, path[path.length - 1].worldPosition.y, "DiscoveryTile", 0); //scrap_metal_00
    
                    if (storage.IsSetGlobal("signs_Go" + cData.objt.sid)) {
                        var crates = storage.GetGlobal("signs_Go" + cData.objt.sid);
                        crates.push(idObj);
                        storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                    }
                    else {
                        var crates = [];
                        crates.push(idObj);
                        storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                    }*/


                    /*if (path.length > 0) {
                        path[0].worldPosition = SC_Utilities.RotatePoint(path[0].worldPosition, cData.stCoord, cData.stRot);
                        var idObj = generator.AddSpecialObject(cData.objt.sid, path[0].worldPosition.x, path[0].worldPosition.y, "DiscoveryTile", 0); //scrap_metal_00
    
                        if (storage.IsSetGlobal("signs_Go" + cData.objt.sid)) {
                            var crates = storage.GetGlobal("signs_Go" + cData.objt.sid);
                            crates.push(idObj);
                            storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                        }
                        else {
                            var crates = [];
                            crates.push(idObj);
                            storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                        }
                        
                    }*/

                    /*if (path.length != 1)
                    {
                        path.splice(0, 1); // splicing index 0 initial position so that the index 1 is the first point the drone tries to reach.

                    }*/


                    //if (path.length > 0) {
                    //    
                    //}

                    if (PathfindDebugMovementPathFoundSigns != -1) {


                        for (var i = 0; i < path.path.length; i++) {

                            if (!isNaN(cData.stCoord) && !isNaN(cData.stRot)) {

                                path.path[i].worldPosition = SC_Utilities.RotatePoint(path.path[i].worldPosition, cData.stCoord, -cData.stRot);

                            }



                            if (forceUseRetracePathSwtc != 2)
                            {
                                //TOREADD
                                //TOREADD
                                //TOREADD
                                /*if (storage.IsSetGlobal("crates_Go" + cData.objt.sid)) {
                                    var idObj = generator.AddSpecialObject(cData.objt.sid, path.path[i].worldPosition.x, path.path[i].worldPosition.y, "dinosaurShellVEDiscoveryWaypoints", 0);
                                    //var item_list = "droplist_empty";
                                    //var cont_id = generator.AddContainer(cData.objt.sid, rotWorldForStation.x, rotWorldForStation.y, "crate_02", item_list, { itemlist: item_list});
                                    var crates = storage.GetGlobal("crates_Go" + cData.objt.sid);
                                    crates.push({ id: idObj, staID: stationTilesArray[cData.iop].baseID });
                                    storage.SetGlobal("crates_Go" + cData.objt.sid, crates);
                                }
                                else {

                                    var idObj = generator.AddSpecialObject(cData.objt.sid, path.path[i].worldPosition.x, path.path[i].worldPosition.y, "dinosaurShellVEDiscoveryWaypoints", 0); //scrap_metal_00
                                    var crates = [];
                                    crates.push(idObj);
                                    storage.SetGlobal("crates_Go" + cData.objt.sid, crates);
                                }*/
                                //TOREADD
                                //TOREADD
                                //TOREADD
                                //node = cData.dog.openSet[0];
                                //cData.dog.openSet.splice(0, 1);
                                //cData.dog.node = node;

                            }
                            else {

                                //TOREADD
                                //TOREADD
                                //TOREADD
                                /*if (storage.IsSetGlobal("crates_Go" + cData.objt.sid)) {
                                    var idObj = generator.AddSpecialObject(cData.objt.sid, path.path[i].worldPosition.x, path.path[i].worldPosition.y, "stopSign_01", 0);
                                    //var item_list = "droplist_empty";
                                    //var cont_id = generator.AddContainer(cData.objt.sid, rotWorldForStation.x, rotWorldForStation.y, "crate_02", item_list, { itemlist: item_list});
                                    var crates = storage.GetGlobal("crates_Go" + cData.objt.sid);
                                    crates.push({ id: idObj, staID: stationTilesArray[cData.iop].baseID });
                                    storage.SetGlobal("crates_Go" + cData.objt.sid, crates);
                                }
                                else {

                                    var idObj = generator.AddSpecialObject(cData.objt.sid, path.path[i].worldPosition.x, path.path[i].worldPosition.y, "stopSign_01", 0); //scrap_metal_00
                                    var crates = [];
                                    crates.push(idObj);
                                    storage.SetGlobal("crates_Go" + cData.objt.sid, crates);
                                }*/
                                //TOREADD
                                //TOREADD
                                //TOREADD
                                //cData.dog.node = cData.dog.openSet[0];
                            }


                            




                            //var idObj = generator.AddDecoration(cData.objt.sid, "goSign_00", { X: path[i].worldPosition.x, Y: path[i].worldPosition.y }, 0, { X: 0, Y: 0, Z: 0 }, { X: 0, Y: 0, Z: 0 }, 1); //scrap_metal_00
                            //var item_list = "droplist_empty";
                            //var idObj = generator.AddContainer(cData.objt.sid, path[i].worldPosition.x, path[i].worldPosition.y, "goSign_00", item_list, { itemlist: item_list });

                            /*globIndex = parseInt(npc.GetTag(cData.objt.nid, "droneIndex"));
                            var idObj = generator.AddSpecialObject(cData.objt.sid, path[i].worldPosition.x, path[i].worldPosition.y, "goSign_00", 0); //scrap_metal_00

                            if (storage.IsSetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_" + "signs_Go")) {
                                var crates = storage.GetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_" + "signs_Go");
                                crates.push(idObj);
                                ////console.PrintError("sc_pathfind.js == sid: " + cData.objt.sid);
                                storage.SetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_" + "signs_Go", crates);
                            }
                            else {
                                var crates = [];
                                crates.push(idObj);
                                storage.SetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_" + "signs_Go", crates);
                            }*/
                            //TOREADD
                            //TOREADD
                            //TOREADD
                            //TOREADD
                            //TOREADD
                            //TOREADD
                            /*var idObj = generator.AddSpecialObject(cData.objt.sid, path[i].worldPosition.x, path[i].worldPosition.y, "goSign_00", 0); //scrap_metal_00
                            //var item_list = "droplist_empty";
                            //var idObj = generator.AddContainer(cData.objt.sid, path[i].worldPosition.x, path[i].worldPosition.y, "DiscoveryTile", item_list, { itemlist: item_list });

                            //console.PrintError("globIndex: " + globIndex + " sys_id: " + sys_id);
                            if (storage.IsSetGlobal("index_" + globIndex + "_sid_" + sys_id + "_frame_" + "signs_Go")) {
                                var crates = storage.GetGlobal("index_" + globIndex + "_sid_" + sys_id + "_frame_" + "signs_Go");
                                //crates.push(idObj);
                                crates.push({ id: idObj, globIndex: globIndex, sys_id: sys_id });
                                storage.SetGlobal("index_" + globIndex + "_sid_" + sys_id + "_frame_" + "signs_Go", crates);
                            }
                            else {
                                var crates = [];
                                crates.push({ id: idObj, globIndex: globIndex, sys_id: sys_id });
                                storage.SetGlobal("index_" + globIndex + "_sid_" + sys_id + "_frame_" + "signs_Go", crates);
                            }*/
                            //TOREADD
                            //TOREADD
                            //TOREADD


                            //TOREADD
                            //TOREADD
                            //TOREADD







                            /*type: "satellite_01_rust",
                            count_from: 1,
                            count_to: 2,
                            rotation: this.GenSatelliteRotation(),
                            rotationSpeed: this.GenSatelliteRotationSpeed(),
                            distance_from: 45,
                            distance_to: 80,
                            scale_from: 0.7,
                            scale_to: 0.7,*/
                            /*GenSatelliteRotation: function() {
                                return { X: (MathExt.RandDouble()) * 90, Y: (MathExt.RandDouble()) * 90, Z: 0.0 };
                            },
                            GenSatelliteRotationSpeed: function() {
                                return { X: (MathExt.RandDouble() * 0.5 - 1.0) * 0.025 * 90, Y: 0.0, Z: 0.0 };
                            },*/







                            //var idObj = generator.AddDecoration(cData.objt.sid, "goSign_00", { X: path[i].worldPosition.x, Y: path[i].worldPosition.y }, 0, { X: 0, Y: 0, Z: 0 }, { X: 0, Y: 0, Z: 0 }, 1); //scrap_metal_00
                            //var item_list = "droplist_empty";
                            //var idObj = generator.AddContainer(cData.objt.sid, path[i].worldPosition.x, path[i].worldPosition.y, "goSign_00", item_list, { itemlist: item_list });

                            /*globIndex = parseInt(npc.GetTag(cData.objt.nid, "droneIndex"));
                            var idObj = generator.AddSpecialObject(cData.objt.sid, path[i].worldPosition.x, path[i].worldPosition.y, "goSign_00", 0); //scrap_metal_00

                            if (storage.IsSetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_" + "signs_Go")) {
                                var crates = storage.GetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_" + "signs_Go");
                                crates.push(idObj);
                                ////console.PrintError("sc_pathfind.js == sid: " + cData.objt.sid);
                                storage.SetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_" + "signs_Go", crates);
                            }
                            else {
                                var crates = [];
                                crates.push(idObj);
                                storage.SetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_" + "signs_Go", crates);
                            }*/









                        }
                    }




                    /*if (forceUseRetracePathSwtc == 1)
                    {
                        currentCom = 5;
                    }
                    else
                    {
                        currentCom = 10;
                    }*/

                    cData.lnode = node;
                    var coms = 0;
                    
                    if (forceUseRetracePathSwtc != 2) {
                        if (path.isMaxed == 2) {
                            coms = 2;
                        }
                        else {
                            coms = 10;
                        }
                    }
                    else
                    {
                        coms = 9;
                    }

                    pathfindCounter = 0;
                    ////console.PrintError("33cData.cFWPD: " + cData.cFWPD + " cData.mSwtc " + cData.mSwtc + " cData.pfc: " + cData.pfc + " SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.js ");
                    var data = { openSet: cData.dog.openSet, currentCommand: coms, grid: cData.log[node.gridIndex].grid, path: path.path, node: node, iot: null, PathfindCounter: pathfindCounter, PathfindCounterSwtch: pathfindCounterSwtch };
                    return data;
                }



                if (forceUseRetracePathSwtc != 1) {
                    //------------------------------------
                    //------------------------------------
                    var someData = SC_PathFindCheckAllSides_cc_5.checkAllSidesGridIndex(node, gridWorldSize, cData, cData.lip);
                    //------------------------------------
                    //------------------------------------


                    /*if (node != null) {
                        //---////console.PrintError("node != null sc_pathfind_cc_3.js");
                    }
                    else {
                        //---////console.PrintError("node == null sc_pathfind_cc_3.js");
                    }*/

                    var someExtratiles = [];

                    if (someData.extraTiles != null) {
                        if (someData.extraTiles.length > 0) {
                            someExtratiles = someData.extraTiles;
                            var someTempNeighboorArray = [];
                            for (var i = 0; i < someExtratiles.length; i++) {
                                cData.lsp = { x: (someExtratiles[i].sgp.x), y: (someExtratiles[i].sgp.y) };
                                cData.glip = { x: someExtratiles[i].docg.gridData.x, y: someExtratiles[i].docg.gridData.y };

                                var gridIndex = someExtratiles[i].docg.index;

                                if (cData.log[gridIndex] == null) {

                                    //(gridWorldSize, nodeRadius, stationTiles, indexOfGrid, cData, noder)


                                    var pathData = SC_PathFindInitGrid_Data_cc_5_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, gridIndex, cData, node,0);

                                    //var pathData = SC_PathFindInitGrid_Data_cc_5_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, gridIndex, cData, node);


                                    if (pathData.fSwtch == 1) {
                                        //---////console.PrintError("@@@--@@@");
                                        //---////console.PrintError("00fSwtch == " + pathData.fSwtch + " // SC_PathFind_cc_3.js");
                                        //---////console.PrintError("@@@--@@@");

                                        /*var data = { openSet: null, currentCommand: 1, grid: null, path: null, node: null, iot: null };
                                        //cData.pfc == 1;
                                        //cData.mSwtc = 1;
                                        return data;*/

                                        /*var path = RetracePath(cData.lip, cData.ltp, node, cData.log);
                                        path.splice(path.length - 1, 1); // splicing the last node but i might readd it later if it helps.
                                        path.splice(0, 1); // splicing index 0 initial position so that the index 1 is the first point the drone tries to reach.
        
                                        for (var i = 0; i < path.length; i++) {
        
                                            path[i].worldPosition = SC_Utilities.RotatePoint(path[i].worldPosition, cData.stCoord, cData.stRot);
        
                                            var idObj = generator.AddSpecialObject(cData.objt.sid, path[i].worldPosition.x, path[i].worldPosition.y, "DiscoveryTile", 0); //scrap_metal_00
        
                                            if (storage.IsSetGlobal("signs_Go" + cData.objt.sid)) {
                                                var crates = storage.GetGlobal("signs_Go" + cData.objt.sid);
                                                crates.push(idObj);
                                                storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                                            }
                                            else {
                                                var crates = [];
                                                crates.push(idObj);
                                                storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                                            }
                                        }*/

                                        ////---////console.PrintError("found Waypoint sc_pathfind_cc_3.js"); 

                                        //var data = { openSet: cData.dog.openSet, currentCommand: 1, grid: cData.log[node.gridIndex].grid, path: path, node: node, iot: null };
                                        //return data;

                                        //var data = { openSet: null, currentCommand: 4, grid: null, path: null, node: null, iot: null };
                                        //return data;
                                    }
                                    else if (pathData.fSwtch == 2) {

                                        //console.PrintError("@@@--@@@");
                                        //unwalkableTargetTile = 1;
                                        //console.PrintError("22fSwtch == " + pathData.fSwtch + " // SC_PathFind_cc_3.js");

                                        ////console.PrintError("22cData.cFWPD: " + cData.cFWPD + " cData.mSwtc " + cData.mSwtc + " cData.pfc: " + cData.pfc + " SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.js ");

                                        //var data = { openSet: cData.dog.openSet, currentCommand: currentCom, grid: cData.log[node.gridIndex].grid, path: path, node: node, iot: null, PathfindCounter: pathfindCounter, PathfindCounterSwtch: pathfindCounterSwtch };
                                        //return data;

                                        //---////console.PrintError("@@@--@@@");
                                        //---////console.PrintError("11fSwtch == " + pathData.fSwtch + " // SC_PathFind_cc_3.js");
                                        //---////console.PrintError("@@@--@@@");

                                        /*var data = { openSet: null, currentCommand: 1, grid: null, path: null, node: null, iot: null };
                                        //cData.pfc == 1;
                                        //cData.mSwtc = 1;
                                        return data;*/

                                        /*var path = RetracePath(cData.lip, cData.ltp, node, cData.log);
                                        path.splice(path.length - 1, 1); // splicing the last node but i might readd it later if it helps.
                                        path.splice(0, 1); // splicing index 0 initial position so that the index 1 is the first point the drone tries to reach.
        
                                        for (var i = 0; i < path.length; i++) {
        
                                            path[i].worldPosition = SC_Utilities.RotatePoint(path[i].worldPosition, cData.stCoord, cData.stRot);
        
                                            var idObj = generator.AddSpecialObject(cData.objt.sid, path[i].worldPosition.x, path[i].worldPosition.y, "DiscoveryTile", 0); //scrap_metal_00
        
                                            if (storage.IsSetGlobal("signs_Go" + cData.objt.sid)) {
                                                var crates = storage.GetGlobal("signs_Go" + cData.objt.sid);
                                                crates.push(idObj);
                                                storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                                            }
                                            else {
                                                var crates = [];
                                                crates.push(idObj);
                                                storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                                            }
                                        }
        
                                        ////---////console.PrintError("found Waypoint sc_pathfind_cc_3.js"); 
        
                                        var data = { openSet: cData.dog.openSet, currentCommand: 10, grid: cData.log[node.gridIndex].grid, path: path, node: node, iot: null };
                                        return data;*/

                                        //var data = { openSet: null, currentCommand: 4, grid: null, path: null, node: null, iot: null };
                                        //return data;
                                    }
                                    else if (pathData.fSwtch == -2) {
                                        unwalkableTargetTile = 1;
                                        //console.PrintError("22fSwtch == " + pathData.fSwtch + " // SC_PathFind_cc_3.js");

                                        ////console.PrintError("22cData.cFWPD: " + cData.cFWPD + " cData.mSwtc " + cData.mSwtc + " cData.pfc: " + cData.pfc + " SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.js ");

                                        var data = { openSet: cData.dog.openSet, currentCommand: 3, grid: cData.log[node.gridIndex].grid, path: path, node: node, iot: null, PathfindCounter: pathfindCounter, PathfindCounterSwtch: pathfindCounterSwtch  };
                                        pathfindCounter++;
                                        return data;
                                    }
                                    else {
                                        //---////console.PrintError("33fSwtch == " + pathData.fSwtch + " // SC_PathFind_cc_3.js");
                                    }


                                    var data = pathData.openSet;

                                    cData.dog.grid = pathData.grid;

                                    var someDataOfGrid = { grid: cData.dog.grid };
                                    cData.log[gridIndex] = someDataOfGrid;
                                    someTempNeighboorArray.push(data[0]);


                                    continue;
                                }
                                else {
                                    someTempNeighboorArray.push(cData.log[gridIndex].grid[someExtratiles[i].iot]);
                                }
                            }
                            for (var i = 0; i < someTempNeighboorArray.length; i++) {
                                var neighboorTile = { swtc: 0, node: someTempNeighboorArray[i], sgp: null, iot: null, iog: null };
                                someData.neighboors.push(neighboorTile);
                            }
                        }
                    }







                    if (node.isStationTile == 1 || node.isStationTile == 2) // currently unassigned
                    {
                        if (node.isStationTile == 2)
                        {
                            //------------------------------------
                            //------------------------------------
                            var someNewStationTilesUnwalkableNeighboorsData = SC_PathFindCheckAllSides_cc_5.checkAllSidesGridIndex(node, gridWorldSize, cData, cData.lip);
                            //------------------------------------
                            //------------------------------------

                            if (someNewStationTilesUnwalkableNeighboorsData.extraTiles != null) {
                                if (someNewStationTilesUnwalkableNeighboorsData.extraTiles.length > 0) {
                                    someExtratiles = someNewStationTilesUnwalkableNeighboorsData.extraTiles;
                                    var someTempNeighboorArray = [];
                                    for (var i = 0; i < someExtratiles.length; i++) {
                                        cData.lsp = { x: (someExtratiles[i].sgp.x), y: (someExtratiles[i].sgp.y) };
                                        cData.glip = { x: someExtratiles[i].docg.gridData.x, y: someExtratiles[i].docg.gridData.y };

                                        var gridIndex = someExtratiles[i].docg.index;

                                        if (cData.log[gridIndex] == null) {

                                            //(gridWorldSize, nodeRadius, stationTiles, indexOfGrid, cData, noder)


                                            var pathData = SC_PathFindInitGrid_Data_cc_5_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, gridIndex, cData, node, 1 );

                                            //var pathData = SC_PathFindInitGrid_Data_cc_5_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, gridIndex, cData, node);

                                            var data = pathData.openSet;

                                            cData.dog.grid = pathData.grid;

                                            var someNewStationTilesUnwalkableNeighboorsDataOfGrid = { grid: cData.dog.grid };
                                            cData.log[gridIndex] = someNewStationTilesUnwalkableNeighboorsDataOfGrid;
                                            someTempNeighboorArray.push(data[0]);


                                            continue;
                                        }
                                        else {
                                            someTempNeighboorArray.push(cData.log[gridIndex].grid[someExtratiles[i].iot]);
                                        }
                                    }

                                    for (var i = 0; i < someNewStationTilesUnwalkableNeighboorsData.neighboors.length; i++) {
                                        someNewStationTilesUnwalkableNeighboorsData.neighboors[i].node.boolWalk = 0;
                                        someNewStationTilesUnwalkableNeighboorsData.neighboors[i].node.closed = 1;
                                        someNewStationTilesUnwalkableNeighboorsData.neighboors[i].node.isStationTile = 1;
                                    }

                                    for (var i = 0; i < someTempNeighboorArray.length; i++) {

                                        var neighboorTile = { swtc: 0, node: someTempNeighboorArray[i], sgp: null, iot: null, iog: null };
                                        neighboorTile.node.boolWalk = 0;
                                        neighboorTile.node.closed = 1;
                                        neighboorTile.node.isStationTile = 1;
                                        //someTempNeighboorArray[i].boolWalk = 0;
                                        //someTempNeighboorArray[i].closed = 1;

                                        someNewStationTilesUnwalkableNeighboorsData.neighboors.push(neighboorTile);
                                    }
                                }
                            }

                            for (var j = 0; j < someNewStationTilesUnwalkableNeighboorsData.length; j++) {
                                someData.neighboors.push(someNewStationTilesUnwalkableNeighboorsData[j]);
                            }
                            node.isStationTile = 3;
                        }



                       
                    }






                    //VISUAL DISCOVERY TILES CREATOR SECTION
                    //VISUAL DISCOVERY TILES CREATOR SECTION
                    //VISUAL DISCOVERY TILES CREATOR SECTION
                    if (PathfindDebugMovementSigns != -1) {







                        //maxDroneIndex is an array and starts at 0 hence why the minus sign although i could have the formations start at 0 also and i wouldn't be writting this.
                        /*if (cData.objt.formation == 1) {
                            globIndex = storage.GetGlobal("maxDroneIndex" + cData.objt.formation - 1);
                        }
                        else if (cData.objt.formation == 2) {
                            globIndex = storage.GetGlobal("maxDroneIndex" + cData.objt.formation - 1);
                        }
                        else if (cData.objt.formation == 3) {
                            globIndex = storage.GetGlobal("maxDroneIndex" + cData.objt.formation - 1);
                        }
                        else if (cData.objt.formation == 4) {
                            globIndex = storage.GetGlobal("maxDroneIndex" + cData.objt.formation - 1);
                        }
                        else if (cData.objt.formation == 5) {
                            globIndex = storage.GetGlobal("maxDroneIndex" + cData.objt.formation - 1);
                        }*/

                        if (cData.objt.frameCounterArrayResetVisualTiles > maxFrameCounterArrayResetVisualTiles) {
                            cData.objt.frameCounterArrayResetVisualTiles = 0;
                        }

                        //TOREADD
                        //TOREADD
                        //TOREADD
                        /*if (storage.IsSetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_" + "_dinosaurShellVEDiscoveryWaypoints_"))
                        {
                            var idObj = generator.AddSpecialObject(cData.objt.sid, node.worldPosition.x, node.worldPosition.y, "dinosaurShellVEDiscoveryWaypoints", 0); //scrap_metal_00
                            var crates = storage.GetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_"  + "_dinosaurShellVEDiscoveryWaypoints_");
                            crates.push(idObj);
                            storage.SetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_"  + "_dinosaurShellVEDiscoveryWaypoints_", crates);
                        }
                        else
                        {
                            var crates = [];
                            crates.push(idObj);
                            storage.SetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_"  + "_dinosaurShellVEDiscoveryWaypoints_", crates);
                        }*/
                        //TOREADD
                        //TOREADD
                        //TOREADD


                        /*globIndex = parseInt(npc.GetTag(cData.objt.nid, "droneIndex"));
                        var idObj = generator.AddSpecialObject(cData.objt.sid, node.worldPosition.x, node.worldPosition.y, "goSign_00", 0); //scrap_metal_00

                        if (storage.IsSetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_" + "signs_Go")) {
                            var crates = storage.GetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_" + "signs_Go");
                            crates.push(idObj);
                            ////console.PrintError("sc_pathfind.js == sid: " + cData.objt.sid);
                            storage.SetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_" + "signs_Go", crates);
                        }
                        else {
                            var crates = [];
                            crates.push(idObj);
                            storage.SetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_" + "signs_Go", crates);
                        }*/

                        /*var idObj = generator.AddSpecialObject(cData.objt.sid, node.worldPosition.x, node.worldPosition.y, "dinosaurShellVEDiscoveryWaypoints", 0); //scrap_metal_00

                        if (storage.IsSetGlobal("crates_" + cData.objt.sid))
                        {
                            var crates = storage.GetGlobal("crates_" + cData.objt.sid);
                            crates.push(idObj);
                            storage.SetGlobal("crates_" + cData.objt.sid, crates);
                        }
                        else
                        {
                            var crates = [];
                            crates.push(idObj);
                            storage.SetGlobal("crates_" + cData.objt.sid, crates);
                        }*/

                    }
                    //frameCounterArrayResetVisualTiles++;
                    //VISUAL DISCOVERY TILES CREATOR SECTION
                    //VISUAL DISCOVERY TILES CREATOR SECTION
                    //VISUAL DISCOVERY TILES CREATOR SECTION










                    var globIndex = parseInt(npc.GetTag(cData.objt.nid, "droneIndex"));

                    if (someData.neighboors.length > 0) {

                        for (var j = 0; j < someData.neighboors.length; j++) {
                            if (someData.neighboors[j].node != null) {


                                if (cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].closed == 1 ||
                                    someData.neighboors[j].node.boolWalk == 0 ||
                                    someData.neighboors[j].node.closed == 1 ||
                                    cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].boolWalk == 0 ||
                                    someData.neighboors[j].node.isStationTile == 1 ||
                                    someData.neighboors[j].node.isStationTile == 2 ||
                                    cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].isStationTile == 1 ||
                                    cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].isStationTile == 2) {


                                 

                                        //I PREFER THESE VISUAL TILES CREATED AS THEY ARE NICELY DISTRIBUTED AS PER HOW THE NEIGHBOOR TILES ARRIVE IN THIS LOOP BUT IT CURRENTLY LAGS SPAWNING SPECIAL OBJECTS IN HERE.
                                        //VISUAL DISCOVERY TILES CREATOR SECTION
                                        //VISUAL DISCOVERY TILES CREATOR SECTION
                                        //VISUAL DISCOVERY TILES CREATOR SECTION
                                        if (PathfindDebugMovementSigns != -1) {

                                            //if (frameCounterArrayResetVisualTiles > maxFrameCounterArrayResetVisualTiles) {
                                            //    frameCounterArrayResetVisualTiles = 0;
                                            //}


                                            /*if (storage.IsSetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_"  + "_dinosaurShellVEDiscoveryWaypoints_")) {
                                                //var idObj = generator.AddSpecialObject(cData.objt.sid, node.worldPosition.x, node.worldPosition.y, "dinosaurShellVEDiscoveryWaypoints", 0); //scrap_metal_00
                                                var idObj = generator.AddSpecialObject(cData.objt.sid, cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].worldPosition.x, cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].worldPosition.y, "dinosaurShellVEDiscoveryWaypoints", 0); //scrap_metal_00
                                                //var idObj = generator.AddContainer(cData.objt.sid, cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].worldPosition.x, cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].worldPosition.y, "_dinosaurShellVEDiscoveryWaypoints_", item_list, { itemlist: item_list });

                                                var crates = storage.GetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_"  + "_dinosaurShellVEDiscoveryWaypoints_");
                                                crates.push(idObj);
                                                storage.SetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_" + "_dinosaurShellVEDiscoveryWaypoints_", crates);
                                            }
                                            else {
                                                var crates = [];
                                                crates.push(idObj);
                                                storage.SetGlobal("index_" + globIndex + "_sid_" + cData.objt.sid + "_frame_" +  "_dinosaurShellVEDiscoveryWaypoints_", crates);
                                            }*/

                                        }
                                        //frameCounterArrayResetVisualTiles++;


                                        //VISUAL DISCOVERY TILES CREATOR SECTION
                                        //VISUAL DISCOVERY TILES CREATOR SECTION
                                        //VISUAL DISCOVERY TILES CREATOR SECTION
                                    
                                    /*if (someData.neighboors[j].node.boolWalk == 0)
                                    {
                                        if (cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].closed == 1 || cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].closed == 0)
                                        {
                                            if (cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].closed == 1)
                                            {
                                                //---////console.PrintError("boolWalk == 0 // closed == 1");
                                            }
                                            else if (cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].closed == 0)
                                            {
                                                //---////console.PrintError("boolWalk == 0 // closed == 0");
                                            }
                                        }
                                    }*/

                                    continue;
                                }






                                //var containsOrNot = AIPathFindUtilities.doesContain(openSet, someData.neighboors[j].node);

                                var gcost = node.gcost + SC_PathFindUtilities_cc_5.npcCheckDistance(node.worldPosition, someData.neighboors[j].node.worldPosition);

                                if (gcost < someData.neighboors[j].node.gcost || cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].open == 0) {
                                    someData.neighboors[j].node.gcost = gcost;
                                    someData.neighboors[j].node.hcost = SC_PathFindUtilities_cc_5.npcCheckDistance(someData.neighboors[j].node.worldPosition, cData.ltp);

                                    someData.neighboors[j].node.fcost = someData.neighboors[j].node.gcost + someData.neighboors[j].node.hcost;

                                    cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].parent = { iog: node.gridIndex, iot: node.index };

                                    if (cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].open == 0) {

                                        //TOREADD
                                        //TOREADD
                                        //TOREADD
                                        /*if (storage.IsSetGlobal("crates_Go" + cData.objt.sid)) {
                                            var idObj = generator.AddSpecialObject(cData.objt.sid, cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].worldPosition.x, cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].worldPosition.y, "dinosaurShellVEDiscoveryWaypoints", 0);
                                            //var item_list = "droplist_empty";
                                            //var cont_id = generator.AddContainer(cData.objt.sid, rotWorldForStation.x, rotWorldForStation.y, "crate_02", item_list, { itemlist: item_list});
                                            var crates = storage.GetGlobal("crates_Go" + cData.objt.sid);
                                            crates.push({ id: idObj, staID: stationTilesArray[cData.iop].baseID });
                                            storage.SetGlobal("crates_Go" + cData.objt.sid, crates);
                                        }
                                        else {

                                            var idObj = generator.AddSpecialObject(cData.objt.sid, cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].worldPosition.x, cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].worldPosition.y, "dinosaurShellVEDiscoveryWaypoints", 0); //scrap_metal_00
                                            var crates = [];
                                            crates.push(idObj);
                                            storage.SetGlobal("crates_Go" + cData.objt.sid, crates);
                                        }*/
                                        //TOREADD
                                        //TOREADD
                                        //TOREADD
                                        

                                        cData.dog.openSet.push(someData.neighboors[j].node);
                                        cData.log[someData.neighboors[j].node.gridIndex].grid[someData.neighboors[j].node.index].open = 1;
                                    }
                                }
                            }
                        }
                    }
                    else
                    {

                    }
                    //frameCounterArrayResetVisualTiles++;
                    //console.PrintError("normal pathfind currentCommand: 0")







                    ////console.PrintError("00cData.cFWPD: " + cData.cFWPD + " cData.mSwtc " + cData.mSwtc + " cData.pfc: " + cData.pfc + " SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.js ");
                    //pathfindCounter = 0;
                    var data = { openSet: cData.dog.openSet, currentCommand: 3, grid: cData.log[node.gridIndex].grid, path: null, node: node, iot: null, PathfindCounter: pathfindCounter, PathfindCounterSwtch: pathfindCounterSwtch };
                    pathfindCounter++;
                    return data;
                }
                else {

                    ////console.PrintError("11cData.cFWPD: " + cData.cFWPD + " cData.mSwtc " + cData.mSwtc + " cData.pfc: " + cData.pfc + " SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.js ");
                    console.PrintError("forced retracePath pathfind")

                    pathfindCounter = 0;
                    var data = { openSet: cData.dog.openSet, currentCommand: 5, grid: cData.log[node.gridIndex].grid, path: null, node: node, iot: null, PathfindCounter: pathfindCounter, PathfindCounterSwtch: pathfindCounterSwtch };
                    return data;


                }

            }


            if (pathfindCounter > pathfindCounterMax) {

                //console.PrintError("force pathfind move to path even though the pathfind didn't reach the target tile.")
                console.PrintError("@@@@@@@@@pathfindCounter > pathfindCounterMax");
                /*cData.dog.openSet = null;
                cData.mSwtc = 1;
                cData.pfc = -2;
                pathfindCounter = 0;
                currentCom = cData.pfc;*/

                pathfindCounter = 0;
                var data = { openSet: cData.dog.openSet, currentCommand: 3, grid: cData.log[node.gridIndex].grid, path: null, node: node, iot: null, PathfindCounter: pathfindCounter, PathfindCounterSwtch: pathfindCounterSwtch };
                return data;
            }

            var data = { openSet: cData.dog.openSet, currentCommand: 3, grid: cData.log[node.gridIndex].grid, path: null, node: node, iot: null, PathfindCounter: pathfindCounter, PathfindCounterSwtch: pathfindCounterSwtch };
            pathfindCounter++;
            return data;
        }
        else {


            /*var initialPosX = (cData.nData.nCoord.x);
            var initialPosY = (cData.nData.nCoord.y);

            var initialPos = { x: initialPosX, y: initialPosY };

            initialPos.x = Math.round(initialPos.x);
            initialPos.y = Math.round(initialPos.y);

            //cData.lip
            if (roundedX === cData.lip.x && roundedY === cData.lip.y)
            {
                //---////console.PrintError("found start Node merge.js");
                startNodegcoster = 0;
                startNodehcoster = SC_Utilities.GetDistance({ x: roundedX, y: roundedY }, cData.ltp);
                startNodefcoster = startNodegcoster + startNodehcoster;

                grid[index] = { boolWalk: boolWalker, worldPosition: cData.lsp, gcost: startNodegcoster, hcost: startNodehcoster, fcost: startNodefcoster, gridTileX: x, gridTileY: y, index: index, parent: parenter, gridIndex: indexOfGrid, gridPos: cData.glip, open: 0, closed: 0 };;
                openSet.push(grid[index]);
            }*/

            /*var InitialPos = { x: cData.ltp.x, y: cData.ltp.y };
            InitialPos.x = Math.round(InitialPos.x);
            InitialPos.y = Math.round(InitialPos.y);

            var lastTargetPos = { x: cData.lip.x, y: cData.lip.y };
            lastTargetPos.x = Math.round(lastTargetPos.x);
            lastTargetPos.y = Math.round(lastTargetPos.y);



            if (InitialPos.x === lastTargetPos.x && InitialPos.y === lastTargetPos.y) //cData.lip == last initial position
            {
                //---////console.PrintError("### the initial position is the same as the target position. this pathfind was started with missing logic prior to this function call, for this scenario to not even arrive to this point maybe... checking. SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.js");


                ////---////console.PrintError("00 target waypoint is invalid. the formation of the drone has to be modified. The drone can keep knowledge of the formation point but offsetting the pathfinding backwards to find a walkable tile and using that should be my next step to make this work. Also, when that will be done, i need to make sure that if the drone is 1 tile diagonally or linearly next to a station, the pathfind might not have a valid openset and that might be fixed in the script SC_PathFindInitGrid_cc_3 or inside of SC_PathFind_cc_3.js");
            }*/

            /*var currentWaypoint = { x: initialPos.x, y: initialPos.y };
            currentWaypoint.x = Math.round(currentWaypoint.x);
            currentWaypoint.y = Math.round(currentWaypoint.y);

            var lastCurrentWaypoint = { x: cData.ltp.x, y: cData.ltp.y };
            lastCurrentWaypoint.x = Math.round(lastCurrentWaypoint.x);
            lastCurrentWaypoint.y = Math.round(lastCurrentWaypoint.y);


            if (currentWaypoint.x === lastCurrentWaypoint.x && currentWaypoint.y === lastCurrentWaypoint.y && cData.cFWPD < 1.45) {
                //---////console.PrintError("the pathfind restarted because it has a different path to the same waypoint OR");
                //---////console.PrintError("the pathfind restarted because the drone is right next to the waypoint and the start node is the same as the target node.");

                cData.mSwtc = 1;
                return cData;
            }
            else {
                ////---////console.PrintError("npcGetGridData");

                var pathData = SC_PathFindInitGrid_Data_cc_5_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, 0, cData, null);
                //var pathData = SC_PathFindInitGrid_Data_cc_5_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, 0, cData, null);
                //cData.dog.gridOffset = pathData.gridOffset;
                cData.dog.grid = pathData.grid;
                cData.dog.openSet = pathData.openSet;
                cData.dog.closedSet = pathData.closedSet;
                //cData.dog.path = pathData.path;

                var someDataOfGrid = { grid: cData.dog.grid }; //, index: cData.log.length 
                cData.log.push(someDataOfGrid);
                cData.mSwtc = 3;
            }*/






            if (forceUseRetracePathSwtc == 1) {
                if (node != null) {

                    var path = RetracePath(cData.lip, cData.ltp, node, cData.log);

                    if (path.path.length > 0) {




                        if (path.path[path.path.length - 1] == null) {
                            //console.PrintError("00");
                        }
                        if (path.path[path.path.length - 1].worldPosition == null) {
                            //console.PrintError("01");
                        }

                        if (cData.stCoord != null && cData.stRot != null) {
                            //console.PrintError("02");
                            path.path[path.path.length - 1].worldPosition = SC_Utilities.RotatePoint(path.path[path.path.length - 1].worldPosition, cData.stCoord, cData.stRot);
                        }

                        /*if (cData.stRot == null) {
                            //console.PrintError("03");
                        }*/





                        //path.splice(path.length - 1, 1); // splicing the last node but i might readd it later if it helps.
                        //path.splice(0, 1);
                        //console.PrintError("@@@21-PathFind initiated without a grid or without something else " + " restarting search for waypoint with pathfind. for instance, initial position == target position fixed or not. more tests and debugging/commenting out more-@@@");

                        if (PathfindDebugMovementPathFoundSigns != -1) {


                            for (var i = 0; i < path.path.length; i++) {



                                if (cData.stCoord != null && cData.stRot != null) {
                                    //console.PrintError("02");
                                    path.path[i].worldPosition = SC_Utilities.RotatePoint(path.path[i].worldPosition, cData.stCoord, cData.stRot);
                                }

                                //TOREADD
                                //TOREADD
                                //TOREADD
                                /*if (storage.IsSetGlobal("crates_Go" + cData.objt.sid)) {
                                    var idObj = generator.AddSpecialObject(cData.objt.sid, path.path[i].worldPosition.x, path.path[i].worldPosition.y, "dinosaurShellVEDiscoveryWaypoints", 0);
                                    //var item_list = "droplist_empty";
                                    //var cont_id = generator.AddContainer(cData.objt.sid, rotWorldForStation.x, rotWorldForStation.y, "crate_02", item_list, { itemlist: item_list});
                                    var crates = storage.GetGlobal("crates_Go" + cData.objt.sid);
                                    crates.push({ id: idObj, staID: stationTilesArray[cData.iop].baseID });
                                    storage.SetGlobal("crates_Go" + cData.objt.sid, crates);
                                }
                                else {

                                    var idObj = generator.AddSpecialObject(cData.objt.sid, path.path[i].worldPosition.x, path.path[i].worldPosition.y, "dinosaurShellVEDiscoveryWaypoints", 0); //scrap_metal_00
                                    var crates = [];
                                    crates.push(idObj);
                                    storage.SetGlobal("crates_Go" + cData.objt.sid, crates);
                                }*/
                            //TOREADD
                            //TOREADD
                            //TOREADD


                            }
                        }




                        var coms = 0;
                        if (path.isMaxed == 2) {
                            coms = 2;
                        }
                        else {
                            coms = 10;
                        }

                        /*if (forceUseRetracePathSwtc == 1)
                        {
                            currentCom = 5;
                        }
                        else
                        {
                            currentCom = 10;
                        }*/

                        if (path.path.length > 0) {
                            //---////console.PrintError("path !empty");
                        }
                        else {
                            //---////console.PrintError("path empty");
                        }

                        pathfindCounter = 0;
                        var data = { openSet: cData.dog.openSet, currentCommand: coms, grid: cData.log[node.gridIndex].grid, path: path.path, node: node, iot: null, PathfindCounter: pathfindCounter, PathfindCounterSwtch: pathfindCounterSwtch };

                        return data;
                    }
                    else {

                        //console.PrintError("@@@22-PathFind initiated without a grid or without something else " + " restarting search for waypoint with pathfind. for instance, initial position == target position fixed or not. more tests and debugging/commenting out more-@@@");

                        pathfindCounter = 0;
                        var data = { openSet: null, currentCommand: -2, grid: null, path: null, node: null, iot: null, PathfindCounter: pathfindCounter, PathfindCounterSwtch: pathfindCounterSwtch };

                        //cData.pfc == 1;
                        //cData.mSwtc = 1;

                        return data;
                    }
                }
                else {

                    node = cData.lnode;
                    if (node != null) {
                        var path = RetracePath(cData.lip, cData.ltp, node, cData.log);

                        if (path.path.length > 0) {

                            path.path[path.path.length - 1].worldPosition = SC_Utilities.RotatePoint(path.path[path.path.length - 1].worldPosition, cData.stCoord, cData.stRot);

                            path.path.splice(path.path.length - 1, 1); // splicing the last node but i might readd it later if it helps.
                            path.path.splice(0, 1);

                            if (PathfindDebugMovementPathFoundSigns != -1) {


                                for (var i = 0; i < path.path.length; i++) {

                                    path.path[i].worldPosition = SC_Utilities.RotatePoint(path.path[i].worldPosition, cData.stCoord, cData.stRot);

                                    //TOREADD
                                    //TOREADD
                                    //TOREADD
                                    /*if (storage.IsSetGlobal("crates_Go" + cData.objt.sid)) {
                                        var idObj = generator.AddSpecialObject(cData.objt.sid, path.path[i].worldPosition.x, path.path[i].worldPosition.y, "dinosaurShellVEDiscoveryWaypoints", 0);
                                        //var item_list = "droplist_empty";
                                        //var cont_id = generator.AddContainer(cData.objt.sid, rotWorldForStation.x, rotWorldForStation.y, "crate_02", item_list, { itemlist: item_list});
                                        var crates = storage.GetGlobal("crates_Go" + cData.objt.sid);
                                        crates.push({ id: idObj, staID: stationTilesArray[cData.iop].baseID });
                                        storage.SetGlobal("crates_Go" + cData.objt.sid, crates);
                                    }
                                    else {

                                        var idObj = generator.AddSpecialObject(cData.objt.sid, path.path[i].worldPosition.x, path.path[i].worldPosition.y, "dinosaurShellVEDiscoveryWaypoints", 0); //scrap_metal_00
                                        var crates = [];
                                        crates.push(idObj);
                                        storage.SetGlobal("crates_Go" + cData.objt.sid, crates);
                                    }*/
                                    //TOREADD
                                    //TOREADD
                                    //TOREADD
                                }
                            }




                            var coms = 0;
                            if (path.isMaxed == 2) {
                                coms = 2;
                            }
                            else {
                                coms = 10;
                            }

                            /*if (forceUseRetracePathSwtc == 1)
                            {
                                currentCom = 5;
                            }
                            else
                            {
                                currentCom = 10;
                            }*/
                            pathfindCounter = 0;

                            var data = { openSet: cData.dog.openSet, currentCommand: coms, grid: cData.log[node.gridIndex].grid, path: path.path, node: node, iot: null, PathfindCounter: pathfindCounter, PathfindCounterSwtch: pathfindCounterSwtch };
     
                            return data;
                        }
                        else {
                            //console.PrintError("@@@33-PathFind initiated without a grid or without something else " + " restarting search for waypoint with pathfind. for instance, initial position == target position fixed or not. more tests and debugging/commenting out more-@@@");
                            pathfindCounter = 0;
                            var data = { openSet: null, currentCommand: -2, grid: null, path: null, node: null, iot: null, PathfindCounter: pathfindCounter, PathfindCounterSwtch: pathfindCounterSwtch };

                            //cData.pfc == 1;
                            //cData.mSwtc = 1;
                            return data;
                        }
                    }
                    else {
                        //console.PrintError("@@@44-PathFind initiated without a grid or without something else " + " restarting search for waypoint with pathfind. for instance, initial position == target position fixed or not. more tests and debugging/commenting out more-@@@");
                        pathfindCounter = 0;
                        var data = { openSet: null, currentCommand: -2, grid: null, path: null, node: null, iot: null, PathfindCounter: pathfindCounter, PathfindCounterSwtch: pathfindCounterSwtch };

                        //cData.pfc == 1;
                        //cData.mSwtc = 1;
                        return data;
                    }

                }

                /*if (path.length > 0) {
                    path[path.length - 1].worldPosition = SC_Utilities.RotatePoint(path[path.length - 1].worldPosition, cData.stCoord, cData.stRot);



                    var idObj = generator.AddSpecialObject(cData.objt.sid, path[path.length - 1].worldPosition.x, path[path.length - 1].worldPosition.y, "DiscoveryTile", 0); //scrap_metal_00

                    if (storage.IsSetGlobal("signs_Go" + cData.objt.sid)) {
                        var crates = storage.GetGlobal("signs_Go" + cData.objt.sid);
                        crates.push(idObj);
                        storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                    }
                    else {
                        var crates = [];
                        crates.push(idObj);
                        storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                    }
                    path.splice(path.length - 1, 1); // splicing the last node but i might readd it later if it helps.

                    if (path.length > 0) {
                        path[0].worldPosition = SC_Utilities.RotatePoint(path[0].worldPosition, cData.stCoord, cData.stRot);
                        var idObj = generator.AddSpecialObject(cData.objt.sid, path[0].worldPosition.x, path[0].worldPosition.y, "DiscoveryTile", 0); //scrap_metal_00

                        if (storage.IsSetGlobal("signs_Go" + cData.objt.sid)) {
                            var crates = storage.GetGlobal("signs_Go" + cData.objt.sid);
                            crates.push(idObj);
                            storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                        }
                        else {
                            var crates = [];
                            crates.push(idObj);
                            storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                        }
                        path.splice(0, 1); // splicing index 0 initial position so that the index 1 is the first point the drone tries to reach.
                    }

                    if (path.length > 0) {
                        for (var i = 0; i < path.length; i++) {

                            path[i].worldPosition = SC_Utilities.RotatePoint(path[i].worldPosition, cData.stCoord, cData.stRot);

                            var idObj = generator.AddSpecialObject(cData.objt.sid, path[i].worldPosition.x, path[i].worldPosition.y, "DiscoveryTile", 0); //scrap_metal_00

                            if (storage.IsSetGlobal("signs_Go" + cData.objt.sid)) {
                                var crates = storage.GetGlobal("signs_Go" + cData.objt.sid);
                                crates.push(idObj);
                                storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                            }
                            else {
                                var crates = [];
                                crates.push(idObj);
                                storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                            }
                        }
                    }


                    var data = { openSet: cData.dog.openSet, currentCommand: 10, grid: cData.log[node.gridIndex].grid, path: path, node: node, iot: null };
                    return data;
                }
                else
                {
                    //---////console.PrintError("the path is null. SC_PathFind_cc_3.js");
                }*/
                ////---////console.PrintError("found Waypoint sc_pathfind_cc_3.js"); 



                /*var idObj = generator.AddSpecialObject(cData.objt.sid, path[path.length - 1].worldPosition.x, path[path.length - 1].worldPosition.y, "DiscoveryTile", 0); //scrap_metal_00

                if (storage.IsSetGlobal("signs_Go" + cData.objt.sid)) {
                    var crates = storage.GetGlobal("signs_Go" + cData.objt.sid);
                    crates.push(idObj);
                    storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                }
                else {
                    var crates = [];
                    crates.push(idObj);
                    storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                }*/
                /*if (path.length > 0) {
                    path[0].worldPosition = SC_Utilities.RotatePoint(path[0].worldPosition, cData.stCoord, cData.stRot);
                    var idObj = generator.AddSpecialObject(cData.objt.sid, path[0].worldPosition.x, path[0].worldPosition.y, "DiscoveryTile", 0); //scrap_metal_00

                    if (storage.IsSetGlobal("signs_Go" + cData.objt.sid)) {
                        var crates = storage.GetGlobal("signs_Go" + cData.objt.sid);
                        crates.push(idObj);
                        storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                    }
                    else {
                        var crates = [];
                        crates.push(idObj);
                        storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                    }
                    
                }*/





                /*path[path.length - 1].worldPosition = SC_Utilities.RotatePoint(path[path.length - 1].worldPosition, cData.stCoord, cData.stRot);

                path.splice(path.length - 1, 1); // splicing the last node but i might readd it later if it helps.

                path.splice(0, 1); // splicing index 0 initial position so that the index 1 is the first point the drone tries to reach.

                //if (path.length > 0) {
                //    
                //}
                for (var i = 0; i < path.length; i++) {

                    path[i].worldPosition = SC_Utilities.RotatePoint(path[i].worldPosition, cData.stCoord, cData.stRot);

                    var idObj = generator.AddSpecialObject(cData.objt.sid, path[i].worldPosition.x, path[i].worldPosition.y, "DiscoveryTile", 0); //scrap_metal_00

                    if (storage.IsSetGlobal("signs_Go" + cData.objt.sid)) {
                        var crates = storage.GetGlobal("signs_Go" + cData.objt.sid);
                        crates.push(idObj);
                        storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                    }
                    else {
                        var crates = [];
                        crates.push(idObj);
                        storage.SetGlobal("signs_Go" + cData.objt.sid, crates);
                    }
                }

                var currentCom = 10;

                //if (forceUseRetracePathSwtc == 1)
                //{
                 //   currentCom = 5;
                //}
                //else
                //{
                //    currentCom = 10;
                //}

                var data = { openSet: cData.dog.openSet, currentCommand: currentCom, grid: cData.log[node.gridIndex].grid, path: path, node: node, iot: null };
                return data;*/
            }
            else {

                //console.PrintError("@@@55-PathFind initiated without a grid or without something else " + " restarting search for waypoint with pathfind. for instance, initial position == target position fixed or not. more tests and debugging/commenting out more-@@@");
                pathfindCounter = 0;
                var data = { openSet: null, currentCommand: -2, grid: null, path: null, node: null, iot: null, PathfindCounter: pathfindCounter, PathfindCounterSwtch: pathfindCounterSwtch };

                //cData.pfc == 1;
                //cData.mSwtc = 1;

                return data;
            }
        }
    }
};


var counting = 0;
var currentNode;

function RetracePath(initialPos, targetPos, node, listOfGrids) {
    /*if (node.parent == null) {
        var currentNode = node;
    }
    else
    {
        currentNode = node.parent;
    }*/
    currentNode = node.parent;
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
            ////---////console.PrintError("node.parent is NULL");
        }

        if (currentX == startX && currentY == startY) {
            mainSwitch = 0;
            break;
        }

        if (counting > maxRetracePath) //1500 original
        {
            console.PrintError("counting the path gives a total of 1500 total nodes before breaking. This value can be changed. ");


            mainSwitch = 2;
            break;
        }

        counting++;
    }

    var pathData = { path: path, isMaxed: mainSwitch };

  

    return pathData;

}