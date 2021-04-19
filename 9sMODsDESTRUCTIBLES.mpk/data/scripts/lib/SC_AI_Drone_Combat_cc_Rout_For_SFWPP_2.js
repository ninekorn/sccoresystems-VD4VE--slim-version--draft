using(console);
using(player);
using(npc);
using(ship);
using(generator);

include(SC_Utilities.js);
include(SC_PathFindInitGrid_Data_cc_2_merge.js);
include(SC_PathFind_cc_2.js);
include(SC_PathFindUtilities_cc_2.js);

var gridWorldSize = { xL: 1, xR: 0, yB: 1, yT: 0 }; //{ xL: 5, xR: 4, yB: 5, yT: 4};
var nodeRadius = 1;
var stationTiles = null; // only increment if there is a different player, otherwise, the drone index doesnt matter here. only 1 pathfind allowed per player for the moment.
var pathData;
var forceUseRetracePathSwtc = -1;

var minimumDistance = 0.15; // 1 later // 0.175
var pathfindIterations = 10; //original 10 for grid x2 y1

var FramePathfindIteratedCounterMax = 3 * pathfindIterations; //original 5 * 10 for grid x2 y1
var FramePathfindIteratedCounter = 0;

var lastFramerotationDOTVELO = 0; 
var lastFramerotationDOTVELOTWO = 0; 

var someFrameCounterForStopTemp = 0;
var someFrameCounterForStopMax = 0; //3
var someFrameCounterForStopMin = 0;

var someOtherFrameCounterForStopResettingPathfindAtEveryFrame = 0;
var someOtherFrameCounterForStopResettingPathfindAtEveryFrameMax = 0;//use 0 when setdecisionis at 1 //3.123 //15.123 a bit fast at 1 decision per seconds//3//5 //25 too much when the drone finally follows the path fast//13.123454321//5, too fast and the drone restarts the pathfind when its really far. maybe the path.length < 3 is a problem here. //9.123454321

var someOtherFrameCounterForPathPosNotEqualToLastFrame = 0;

var frameDistanceIncreaser = 0;
var additionalDistanceToAdd = 0;
var additionalDistanceToAddMax = 5.25; //3
var frameDistanceIncreaserBase = 0.275;
var minimumStickDistance = 4.35;

var frameDistanceIncreaserTWO = 0;
var additionalDistanceToAddTWO = 0;
var additionalDistanceToAddMaxTWO = 5.25; //3
var frameDistanceIncreaserBaseTWO = 0.275;
var minimumStickDistanceTWO = 4.35;
var decisionsPerSeconds = 1;

var SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2 =
{
    npcPathfindToWaypoint: function (cData) //cData.nData, cData.pData, cData.objt, cData.cFWP,cData.cFWPD
    {
        var waypointDist = Math.round(cData.cFWPD * 10) * 0.1;
        waypointDist *= 10;
        waypointDist = Math.round(waypointDist) * 0.1;
        //console.PrintError("form:" + cData.objt.formation + "/nid:" + cData.objt.nid + "/cFWPD:" + waypointDist + "/mSwtc:" + cData.mSwtc + "/pfc: " + cData.pfc + "/forceRetracePath:" + forceUseRetracePathSwtc); // + " SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2.js "




        if (storage.IsSetGlobal("npcDecisionsPerSeconds" + cData.objt.pid)) {

            decisionsPerSeconds = storage.GetGlobal("npcDecisionsPerSeconds" + cData.objt.pid);
 
            if (decisionsPerSeconds == 1) {
                someOtherFrameCounterForStopResettingPathfindAtEveryFrameMax = 0;
                //frameDistanceIncreaser = 0;
                //additionalDistanceToAdd = 0;
                additionalDistanceToAddMax = 5.25; //3
                frameDistanceIncreaserBase = 0.1;
                minimumStickDistance = 4.35;

                //frameDistanceIncreaserTWO = 0;
                //additionalDistanceToAddTWO = 0;
                additionalDistanceToAddMaxTWO = 5.25; //3
                frameDistanceIncreaserBaseTWO = 0.1;
                minimumStickDistanceTWO = 4.35;
                someFrameCounterForStopMax = 0;


            }
            else if (decisionsPerSeconds == 19) {
                someOtherFrameCounterForStopResettingPathfindAtEveryFrameMax = 46.123;// normally 15.123 but let's try this one now.
                //frameDistanceIncreaser = 0;
                //additionalDistanceToAdd = 0;
                additionalDistanceToAddMax = 5.5; //3
                frameDistanceIncreaserBase = 0.1;
                minimumStickDistance = 4.5;

                //frameDistanceIncreaserTWO = 0;
                //additionalDistanceToAddTWO = 0;
                additionalDistanceToAddMaxTWO = 5.5; //3
                frameDistanceIncreaserBaseTWO = 0.1;
                minimumStickDistanceTWO = 4.5;
                someFrameCounterForStopMax = 3;
            }

            //storage.SetGlobal("npcDecisionsPerSeconds" + player_id, npcDecisionsPerSeconds);
        }


      



        var rotationDotForward = SC_Utilities.Dot(cData.nData.nForward.x, cData.nData.nForward.y, cData.nData.nVelo.x, cData.nData.nVelo.y);
        if (!isNaN(rotationDotForward) || rotationDotForward != 0) //player is not moving
        {
            if (!isNaN(lastFramerotationDOTVELO)) //player is not moving
            {
                if (rotationDotForward < lastFramerotationDOTVELO) {
                    npc.InstantStop(cData.objt.nid);
                }
            }
            else {

                if (!isNaN(rotationDOTVELO)) //player is not moving
                {
                    if (rotationDotForward < rotationDOTVELO) {
                        npc.InstantStop(cData.objt.nid);
                    }
                }
            }
        }


        if (cData.pfc == -2 && cData.mSwtc == 1) {



           
            if (cData.dog.path != null) {

                if (cData.dog.path[1] != null) // && cData.dog.path[0] != null
                {



                    //var offSetDistTwo = additionalDistanceToAddTWO;
                    //var offSetDistTwoTwo = 3.5 + additionalDistanceToAdd;

                    cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
                    cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);

                    var distToNode = SC_Utilities.GetDistance(cData.dog.path[0].worldPosition, cData.nData.nCoord);

                    var dirToWaypointX = ((cData.dog.path[0].worldPosition.x) - (cData.nData.nCoord.x)) / distToNode;
                    var dirToWaypointY = ((cData.dog.path[0].worldPosition.y) - (cData.nData.nCoord.y)) / distToNode;

                    var newOffsetWaypointPosX = (cData.dog.path[0].worldPosition.x + (dirToWaypointX * additionalDistanceToAdd));
                    var newOffsetWaypointPosY = (cData.dog.path[0].worldPosition.y + (dirToWaypointY * additionalDistanceToAdd));

                    var newOffsetWaypointPos = { x: newOffsetWaypointPosX, y: newOffsetWaypointPosY };

                    var rotationDOTRL = SC_Utilities.Dot(dirToWaypointY, -dirToWaypointX, -cData.nData.nForward.x, -cData.nData.nForward.y);
                    var rotationDOTFB = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nForward.x, cData.nData.nForward.y);
                    var rotationDOTVELO = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nVelo.x, cData.nData.nVelo.y);





                    var distToNodeTwo = SC_Utilities.GetDistance(cData.dog.path[1].worldPosition, cData.nData.nCoord);
                    var dirToWaypointXTwo = ((cData.dog.path[1].worldPosition.x) - (cData.nData.nCoord.x)) / distToNodeTwo;
                    var dirToWaypointYTwo = ((cData.dog.path[1].worldPosition.y) - (cData.nData.nCoord.y)) / distToNodeTwo;
                    var newOffsetWaypointPosXTwo = (cData.dog.path[1].worldPosition.x + (dirToWaypointXTwo * (additionalDistanceToAddTWO)));
                    var newOffsetWaypointPosYTwo = (cData.dog.path[1].worldPosition.y + (dirToWaypointYTwo * (additionalDistanceToAddTWO)));
                    var newOffsetWaypointPosTWO = { x: newOffsetWaypointPosXTwo, y: newOffsetWaypointPosYTwo };


                    var rotationDOTRLTWO = SC_Utilities.Dot(dirToWaypointYTwo, -dirToWaypointXTwo, -cData.nData.nForward.x, -cData.nData.nForward.y);
                    var rotationDOTFBTWO = SC_Utilities.Dot(dirToWaypointXTwo, dirToWaypointYTwo, cData.nData.nForward.x, cData.nData.nForward.y);
                    var rotationDOTVELOTWO = SC_Utilities.Dot(dirToWaypointXTwo, dirToWaypointYTwo, cData.nData.nVelo.x, cData.nData.nVelo.y);
                    var distToNodeONEExtended = SC_Utilities.GetDistance(newOffsetWaypointPosTWO, cData.nData.nCoord);


                    if (rotationDOTVELO < 0 && rotationDOTVELOTWO < 0) {
                        npc.InstantStop(cData.objt.nid);
                    }


                    //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);

                    if (distToNodeTwo > minimumDistance) {
                        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                    }
                    else {
                        if (rotationDOTVELO < 0 && rotationDOTVELOTWO < 0) {
                            npc.InstantStop(cData.objt.nid);
                        }


                        cData.dog.path.shift();
                        //if (cData.dog.path.length < 1)
                        //{
                        cData.stopSCM = someFrameCounterForStopTemp;

                        if (cData.stopS == 0) {
                            npc.Unstick(cData.objt.nid);
                            npc.InstantStop(cData.objt.nid);
                            cData.stopS = 0;
                        }
                        someFrameCounterForStopTemp = someFrameCounterForStopMax;
                        //}
                    }

                    if (rotationDOTVELOTWO < 0.99) {
                        cData.stopSCM = someFrameCounterForStopTemp;

                        if (cData.stopS == 0) {
                            npc.Unstick(cData.objt.nid);
                            npc.InstantStop(cData.objt.nid);
                            cData.stopS = 0;
                        }
                        someFrameCounterForStopTemp = someFrameCounterForStopMin;
                    }
                    else {
                        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                    }
                    /*
                    if (distToNodeTwo > minimumDistance) {
                        //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
        
                        if (rotationDOTFBTWO >= 0.99) {
                            if (rotationDOTVELOTWO < 0.99) {
        
                                //npc.InstantStop(cData.objt.nid);
        
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                            }
                            else {
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                            }
                        }
                        else {
                            if (rotationDOTVELOTWO < 0.99) {
        
                                //npc.InstantStop(cData.objt.nid);
        
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                            }
                            else {
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                            }
                        }
                    }
                    else {
                        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                        cData.dog.path.shift();
                        //npc.InstantStop(cData.objt.nid);
                    }*/
                }
                else {

                    if (cData.dog.path[0] != null) {
                        cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
                        cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);

                        var distToNode = SC_Utilities.GetDistance(cData.dog.path[0].worldPosition, cData.nData.nCoord);

                        var dirToWaypointX = ((cData.dog.path[0].worldPosition.x) - (cData.nData.nCoord.x)) / distToNode;
                        var dirToWaypointY = ((cData.dog.path[0].worldPosition.y) - (cData.nData.nCoord.y)) / distToNode;

                        var newOffsetWaypointPosX = (cData.dog.path[0].worldPosition.x + (dirToWaypointX * additionalDistanceToAdd));
                        var newOffsetWaypointPosY = (cData.dog.path[0].worldPosition.y + (dirToWaypointY * additionalDistanceToAdd));

                        var newOffsetWaypointPos = { x: newOffsetWaypointPosX, y: newOffsetWaypointPosY };

                        var rotationDOTRL = SC_Utilities.Dot(dirToWaypointY, -dirToWaypointX, -cData.nData.nForward.x, -cData.nData.nForward.y);
                        var rotationDOTFB = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nForward.x, cData.nData.nForward.y);
                        var rotationDOTVELO = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nVelo.x, cData.nData.nVelo.y);

                        /*var rotationDotForward = SC_Utilities.Dot(cData.nData.nForward.x, cData.nData.nForward.y, cData.nData.nVelo.x, cData.nData.nVelo.y);
                        if (!isNaN(rotationDotForward) || rotationDotForward != 0) //player is not moving
                        {

                        }*/





                        //var distToNodeTwo = SC_Utilities.GetDistance(newOffsetWaypointPos, cData.nData.nCoord);

                        //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);

                        if (distToNode > minimumDistance) {
                            npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                        }
                        else {
                            if (rotationDOTVELO < 0) {
                                npc.InstantStop(cData.objt.nid);
                            }


                            cData.dog.path.shift();
                            //if (cData.dog.path.length < 1) {
                            cData.stopSCM = someFrameCounterForStopTemp;

                            if (cData.stopS == 0) {
                                npc.Unstick(cData.objt.nid);
                                npc.InstantStop(cData.objt.nid);
                                cData.stopS = 0;
                            }
                            someFrameCounterForStopTemp = someFrameCounterForStopMax;
                            //}
                        }

                        if (rotationDOTVELO < 0.99) {
                            cData.stopSCM = someFrameCounterForStopTemp;

                            if (cData.stopS == 0) {
                                npc.Unstick(cData.objt.nid);
                                npc.InstantStop(cData.objt.nid);
                                cData.stopS = 0;
                            }
                            someFrameCounterForStopTemp = someFrameCounterForStopMin;
                        }
                        else {
                            npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                        }

                    }




                    /*if (distToNode > minimumDistance) {
                        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
        
                        if (rotationDOTFB >= 0.99) {
        
                            if (rotationDOTVELO < 0.99) {
                                //npc.InstantStop(cData.objt.nid);
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                            }
                            else {
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                            }
                        }
                        else {
                            if (rotationDOTVELO < 0.99) {
                                //npc.InstantStop(cData.objt.nid);
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                            }
                            else {
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                            }
                        }
                    }
                    else {
                        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                        cData.dog.path.shift();
                        //npc.InstantStop(cData.objt.nid);
                    }*/

                }
                cData.dog.path = [];
            }
            else {


            }
        }
















        //npc.GetObstacleOnTheWay(cData.objt.nid,);
        //ship.GetFirstObstacleOnTheWay(shipId, maxDistance)
        //object.GetObstacleOnRay(ship_id, rota

        //object.GetObstacleOnRay(ship_id, rotationAngle, distance);








        //waypointDist = waypointDist >> 1 << 1;

        //Math.trunc(waypointDist) * 0.1;

        //waypointDist = waypointDist >> 1 << 1;




        var initialPos = { x: cData.nData.nCoord.x, y: cData.nData.nCoord.y };
        initialPos.x = Math.round(initialPos.x);
        initialPos.y = Math.round(initialPos.y);

        var currentWaypoint = { x: cData.cFWP.x, y: cData.cFWP.y };//{ x: initialPos.x, y: initialPos.y };
        currentWaypoint.x = Math.round(currentWaypoint.x);
        currentWaypoint.y = Math.round(currentWaypoint.y);

        var lastCurrentWaypoint = { x: cData.cLFWP.x, y: cData.cLFWP.y };
        lastCurrentWaypoint.x = Math.round(lastCurrentWaypoint.x);
        lastCurrentWaypoint.y = Math.round(lastCurrentWaypoint.y);

        if (initialPos.x === currentWaypoint.x && initialPos.y === currentWaypoint.y || initialPos.x === lastCurrentWaypoint.x && initialPos.y === lastCurrentWaypoint.y)
        {
            //npc.InstantStop(cData.objt.nid);
            cData.mSwtc = 4;
            cData.pfc = 4;
        }


        var distToWaypoint = SC_Utilities.GetDistance(currentWaypoint, cData.nData.nCoord);
        if (distToWaypoint < 5)
        {
            if (decisionsPerSeconds == 1) {
                someOtherFrameCounterForStopResettingPathfindAtEveryFrameMax = 0;
                //frameDistanceIncreaser = 0;
                //additionalDistanceToAdd = 0;
                additionalDistanceToAddMax = 5.25; //3
                frameDistanceIncreaserBase = 0.1;
                minimumStickDistance = 4.35;

                //frameDistanceIncreaserTWO = 0;
                //additionalDistanceToAddTWO = 0;
                additionalDistanceToAddMaxTWO = 5.25; //3
                frameDistanceIncreaserBaseTWO = 0.1;
                minimumStickDistanceTWO = 4.35;

                someFrameCounterForStopMax = 0;


            }
            else if (decisionsPerSeconds == 19) {
                someOtherFrameCounterForStopResettingPathfindAtEveryFrameMax = 46.123;// normally 15.123 but let's try this one now.
                //frameDistanceIncreaser = 0;
                //additionalDistanceToAdd = 0;
                additionalDistanceToAddMax = 5.5; //3
                frameDistanceIncreaserBase = 0.1;
                minimumStickDistance = 4.35;

                //frameDistanceIncreaserTWO = 0;
                //additionalDistanceToAddTWO = 0;
                additionalDistanceToAddMaxTWO = 5.5; //3
                frameDistanceIncreaserBaseTWO = 0.1;
                minimumStickDistanceTWO = 4.35;

                someFrameCounterForStopMax = 3;
            }
        }





        /*else if (initialPos.x === currentWaypoint.x && initialPos.y === currentWaypoint.y) // && currentWaypoint.x === lastCurrentWaypoint.x && currentWaypoint.y === lastCurrentWaypoint.y
        {
            //npc.InstantStop(cData.objt.nid);
            cData.mSwtc = 4;
            cData.pfc = 4;
        }*/


        /*
        if (cData.mSwtc == 3 && cData.pfc == 0) {
            if (cData.dog != null) {
                if (cData.dog.openSet != null) {
                    if (cData.dog.openSet.length <= 0) {
                        ////////console.PrintError("OPENSET EMPTY");
                        cData.mSwtc = 1;
                        cData.pfc = -2;
                        //npc.InstantStop(cData.objt.nid);
                        /*if (cData.swtchForPathfind == 0)
                        {
                            //////npc.InstantStop(cData.objt.nid);
                            //npc.Unstick(cData.objt.nid);
                        }

                        //npc.Unstick(cData.objt.nid);

                        cData.log = [];
                        cData.dog.grid = [];
                        cData.dog.openSet = [];
                        cData.dog.closedSet = [];

                        cData.footStepsCounter = 0;
                    }
                }
                else {
                    ////////console.PrintError("OPENSET EMPTY");
                    cData.mSwtc = 1;
                    cData.pfc = -2;
                    //npc.InstantStop(cData.objt.nid);
                    /*if (cData.swtchForPathfind == 0)
                    {
                        //////npc.InstantStop(cData.objt.nid);
                        //npc.Unstick(cData.objt.nid);
                    }

                    //npc.Unstick(cData.objt.nid);

                    cData.log = [];
                    cData.dog.grid = [];
                    cData.dog.openSet = [];
                    cData.dog.closedSet = [];

                    cData.footStepsCounter = 0;
                }
            }
            else {
                ////////console.PrintError("OPENSET EMPTY");
                cData.mSwtc = 1;
                cData.pfc = -2;
                //npc.InstantStop(cData.objt.nid);
                /*if (cData.swtchForPathfind == 0)
                {
                    //////npc.InstantStop(cData.objt.nid);
                    //npc.Unstick(cData.objt.nid);
                }

                //npc.Unstick(cData.objt.nid);

                cData.log = [];
                cData.dog.grid = [];
                cData.dog.openSet = [];
                cData.dog.closedSet = [];

                cData.footStepsCounter = 0;
            }
        }*/




        if (cData.pfc == -2) {
            cData.hasUnwalkableTiles = -1;
            cData.hasUnwalkableTilesArray = [];


            cData.log = [];
            cData.dog.grid = [];
            cData.dog.openSet = [];
            cData.dog.closedSet = [];
            //cData.dog.path = [];
            //cData.dog = [];
            ////---//////console.PrintError("0npcPathFind cData.pfc == 1");
            cData.pfc = 0;
            cData.mSwtc = 1;
            forceUseRetracePathSwtc = -1;
            FramePathfindIteratedCounter = 0;
            cData.footStepsCounter = 0;
            cData.footStepsCounterMAX = 0;
            additionalDistanceToAdd = 0;
            additionalDistanceToAddTWO = 0;

        }




        /*if (cData.pfc == 1)
        {
            cData.log = [];
            cData.dog.grid = [];
            cData.dog.openSet = [];
            cData.dog.closedSet = [];
            ////---//////console.PrintError("0npcPathFind cData.pfc == 1");
            cData.pfc = -1;
            cData.mSwtc = 1;
        }
        else if (cData.cFWP.x != cData.cLFWP.x || cData.cFWP.y != cData.cLFWP.y)
        {
            if (cData.dog.openSet != null && cData.dog.openSet.length > 0)
            {
                //cData.log = [];
                //cData.dog.grid = [];
                //cData.dog.openSet = [];
                //cData.dog.closedSet = [];
                ////---//////console.PrintError("0 current waypoint x or y is different than the last frame waypoint. restart pathfind for new route.");

                //cData.pfc = -1;
                //cData.mSwtc = 3;
            }
            else
            {
                cData.log = [];
                cData.dog.grid = [];
                cData.dog.openSet = [];
                cData.dog.closedSet = [];
                ////---//////console.PrintError("1 current waypoint x or y is different than the last frame waypoint. restart pathfind for new route.");

                cData.pfc = -1;
                cData.mSwtc = 1;
            }
        }*/





        ////console.PrintError("distanceToAdd " + additionalDistanceToAdd);



        //if (cData.cFWPD > minimumDistance)
        //{
        //if () {}

        if (cData.mSwtc == 1) {



            cData.hasUnwalkableTiles = -1;
            cData.hasUnwalkableTilesArray = [];


            cData.log = [];
            cData.dog.grid = [];
            cData.dog.openSet = [];
            cData.dog.closedSet = [];
            //cData.dog.path = [];
            //cData.dog = [];
            ////---//////console.PrintError("0npcPathFind cData.pfc == 1");
            cData.pfc = 0;
            cData.mSwtc = 1;
            forceUseRetracePathSwtc = -1;
            FramePathfindIteratedCounter = 0;
            cData.footStepsCounter = 0;
            cData.footStepsCounterMAX = 0;
            additionalDistanceToAdd = 0;
            additionalDistanceToAddTWO = 0;

            //////npc.InstantStop(cData.objt.nid);
            ////npc.InstantStopEvasion(cData.objt.nid);

            if (cData.swtchForPathfind == 0) {

                //npc.Unstick(cData.objt.nid);
            }

            var player_name = game.GetShipOwner(cData.objt.pid);

            if (storage.IsSetGlobal("GlobalIndex_Player_" + player_name)) {
                var indexOfPlayer = storage.GetGlobal("GlobalIndex_Player_" + player_name);
                cData.iop = indexOfPlayer;
                //---//////console.PrintError("indexOfPlayer: " + indexOfPlayer);
            }

            cData.sSwtc = 1;

            var initialPosX = (cData.nData.nCoord.x);
            var initialPosY = (cData.nData.nCoord.y);
            var initialPos = { x: initialPosX, y: initialPosY };

            /*var xinitialPos = Math.round(initialPos.x);
            var yinitialPos = Math.round(initialPos.y);

            var lastInitialPos = { x: cData.lip.x, y: cData.lip.y }; 
            lastInitialPos.x = Math.round(lastInitialPos.x);
            lastInitialPos.y = Math.round(lastInitialPos.y);

            if (xinitialPos === lastInitialPos.x && yinitialPos === lastInitialPos.y)
            {
                //---//////console.PrintError("this pathfind start has the same SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2.js");
            }

            if (initialPos.x )
            {

            }*/









            cData.lip = initialPos; //last initial pos
            cData.glip = initialPos;//grid last initial pos
            cData.lsp = initialPos;//last seeker pos
            cData.ltp = cData.cFWP;//last target pos



            var currentWaypoint = { x: cData.cFWP.x, y: cData.cFWP.y };//{ x: initialPos.x, y: initialPos.y };
            currentWaypoint.x = Math.round(currentWaypoint.x);
            currentWaypoint.y = Math.round(currentWaypoint.y);

            var lastCurrentWaypoint = { x: cData.cLFWP.x, y: cData.cLFWP.y };//{ x: cData.ltp.x, y: cData.ltp.y };
            lastCurrentWaypoint.x = Math.round(lastCurrentWaypoint.x);
            lastCurrentWaypoint.y = Math.round(lastCurrentWaypoint.y);

            /*if (cData.lip.x === cData.ltp.x && cData.lip.y === cData.ltp.y && boolWalker == 0) //cData.lip == last initial position
            {
                //---//////console.PrintError("### the initial position is the same as the target position. this pathfind was started with missing logic prior to this function call, for this scenario to not even arrive to this point maybe... checking. SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2.js");
                ////---//////console.PrintError("00 target waypoint is invalid. the formation of the drone has to be modified. The drone can keep knowledge of the formation point but offsetting the pathfinding backwards to find a walkable tile and using that should be my next step to make this work. Also, when that will be done, i need to make sure that if the drone is 1 tile diagonally or linearly next to a station, the pathfind might not have a valid openset and that might be fixed in the script SC_PathFindInitGrid_cc_2 or inside of SC_PathFind_cc_2.js");
            }*/


            //cData.cLFWP = cData.cFWP;
            //cData.cLFWPD = cData.cFWPD;

            /*var pathData = SC_PathFindInitGrid_Data_cc_2_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, 0, cData, null);
            //cData.dog.gridOffset = pathData.gridOffset;
            cData.dog.grid = pathData.grid;
            cData.dog.openSet = pathData.openSet;
            //cData.dog.closedSet = pathData.closedSet; //NOT USED ANYMORE. DOUBLE CHECK BEFORE REMOVING COMPLETELY
            //cData.dog.path = pathData.path;
            var someDataOfGrid = { grid: cData.dog.grid }; //, index: cData.log.length 
            cData.log.push(someDataOfGrid);
            cData.mSwtc = 3;
            cData.pfc = 0;*/

            var pathData = SC_PathFindInitGrid_Data_cc_2_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, 0, cData, null, 0);
            //cData.dog.gridOffset = pathData.gridOffset;
            cData.dog.grid = pathData.grid;
            cData.dog.openSet = pathData.openSet;
            //cData.dog.closedSet = pathData.closedSet; //NOT USED ANYMORE. DOUBLE CHECK BEFORE REMOVING COMPLETELY
            //cData.dog.path = pathData.path;
            var someDataOfGrid = { grid: cData.dog.grid }; //, index: cData.log.length 
            cData.log.push(someDataOfGrid);
            cData.mSwtc = 3;






            /*
            if (currentWaypoint.x === lastCurrentWaypoint.x && currentWaypoint.y === lastCurrentWaypoint.y) //original minimumDistance //but it bugs at some point... why? let's search 
            {
                if (cData.cFWPD >= minimumDistance) {
                    var pathData = SC_PathFindInitGrid_Data_cc_2_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, 0, cData, null);
                    //cData.dog.gridOffset = pathData.gridOffset;
                    cData.dog.grid = pathData.grid;
                    cData.dog.openSet = pathData.openSet;
                    //cData.dog.closedSet = pathData.closedSet; //NOT USED ANYMORE. DOUBLE CHECK BEFORE REMOVING COMPLETELY
                    //cData.dog.path = pathData.path;
                    var someDataOfGrid = { grid: cData.dog.grid }; //, index: cData.log.length 
                    cData.log.push(someDataOfGrid);
                    cData.mSwtc = 3;
                }
                else //cData.cFWPD < minimumDistance
                {
                    //var dirToWaypointX = ((cData.cFWP.x) - (cData.pData.pCoord.x)) / distToNode;
                    //var dirToWaypointY = ((cData.cFWP.y) - (cData.pData.pCoord.y)) / distToNode;

                    //var rotationDOTVELO = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.pData.pVelo.x, cData.pData.pVelo.y);

                    //---//////console.PrintError("0standing next to waypoint.");
                    //if (isNaN(rotationDOTVELO)) //player is not moving
                    //{
                    //
                    //}

                }
            }
            else {
                if (cData.cFWPD >= minimumDistance) {
                    var pathData = SC_PathFindInitGrid_Data_cc_2_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, 0, cData, null);
                    //cData.dog.gridOffset = pathData.gridOffset;
                    cData.dog.grid = pathData.grid;
                    cData.dog.openSet = pathData.openSet;
                    //cData.dog.closedSet = pathData.closedSet; //NOT USED ANYMORE. DOUBLE CHECK BEFORE REMOVING COMPLETELY
                    //cData.dog.path = pathData.path;
                    var someDataOfGrid = { grid: cData.dog.grid }; //, index: cData.log.length 
                    cData.log.push(someDataOfGrid);
                    cData.mSwtc = 3;
                }
                else {
                    //cData.mSwtc = 1;
                    //cData.pfc = -2;
                    //var dirToWaypointX = ((cData.cFWP.x) - (cData.pData.pCoord.x)) / distToNode;
                    //var dirToWaypointY = ((cData.cFWP.y) - (cData.pData.pCoord.y)) / distToNode;

                    //var rotationDOTVELO = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.pData.pVelo.x, cData.pData.pVelo.y);

                    //---//////console.PrintError("1standing next to waypoint.");
                    //if (isNaN(rotationDOTVELO)) //player is not moving
                    //{
                    //
                    //}
                }
            }*/






        }



























        //re-add else if it lags
        if (cData.mSwtc == 3) {

            //pfc arrives at -1 

            //////npc.InstantStop(cData.objt.nid);
            //---//////console.PrintError("npcPathFind");
            //////console.PrintError("cData.pfc == " + cData.pfc);
            if (FramePathfindIteratedCounter > FramePathfindIteratedCounterMax) {
                //cData.mSwtc = 1;
                //cData.pfc = -2;
                //////npc.InstantStop(cData.objt.nid);
                //    ////npc.InstantStop(cData.objt.nid);

                //if (cData.dog.path!= null)
                //{
                //    cData.dog.path.shift();
                //}

                //cData.dog.path = null;
                /*cData.footStepsCounter = 0;
                if (cData.swtchForPathfind == 0) {
                    //npc.Unstick(cData.objt.nid);
                }*/

                console.PrintError("npcPathFind FramePathfindIteratedCounter > FramePathfindIteratedCounterMax");

                forceUseRetracePathSwtc = 1;
                FramePathfindIteratedCounter = 0;
                cData.mSwtc = 3;
            }
            else {

                //to readd for tests
                //console.PrintError("npcPathFind pathfind start");
                //to readd for tests

                var pathData = null;

                for (var i = 0; i < pathfindIterations; i++) {

                    //console.PrintError("pathfindIterations: " + i + " FramePathfindIteratedCounter: " + FramePathfindIteratedCounter);

                    pathData = SC_PathFind_cc_2.npcPathFind(gridWorldSize, nodeRadius, cData, forceUseRetracePathSwtc);

                    /*if (pathfindIterations == 0)
                    {
                        forceUseRetracePathSwtc = 2;
                        pathData = SC_PathFind_cc_2.npcPathFind(gridWorldSize, nodeRadius, cData, forceUseRetracePathSwtc);

                        if (pathData != null) {

                            cData.pfc = pathData.currentCommand;
                            if (cData.pfc == 9) {

                                if (cData.dog.path != null) {


                                    if (cData.dog.path[1] != null) // && cData.dog.path[0] != null
                                    {
                                        //var offSetDistTwo = 3.5 + additionalDistanceToAdd;
                                        //var offSetDistTwoTwo = 3.5 + additionalDistanceToAdd;

                                        cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
                                        cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);

                                        var distToNode = SC_Utilities.GetDistance(cData.dog.path[0].worldPosition, cData.nData.nCoord);

                                        var dirToWaypointX = ((cData.dog.path[0].worldPosition.x) - (cData.nData.nCoord.x)) / distToNode;
                                        var dirToWaypointY = ((cData.dog.path[0].worldPosition.y) - (cData.nData.nCoord.y)) / distToNode;

                                        var newOffsetWaypointPosX = (cData.dog.path[0].worldPosition.x + (dirToWaypointX * additionalDistanceToAdd));
                                        var newOffsetWaypointPosY = (cData.dog.path[0].worldPosition.y + (dirToWaypointY * additionalDistanceToAdd));

                                        var newOffsetWaypointPos = { x: newOffsetWaypointPosX, y: newOffsetWaypointPosY };

                                        var rotationDOTRL = SC_Utilities.Dot(dirToWaypointY, -dirToWaypointX, -cData.nData.nForward.x, -cData.nData.nForward.y);
                                        var rotationDOTFB = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nForward.x, cData.nData.nForward.y);
                                        var rotationDOTVELO = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nVelo.x, cData.nData.nVelo.y);

                                        var distToNodeTwo = SC_Utilities.GetDistance(cData.dog.path[1].worldPosition, cData.nData.nCoord);
                                        var dirToWaypointXTwo = ((cData.dog.path[1].worldPosition.x) - (cData.nData.nCoord.x)) / distToNodeTwo;
                                        var dirToWaypointYTwo = ((cData.dog.path[1].worldPosition.y) - (cData.nData.nCoord.y)) / distToNodeTwo;
                                        var newOffsetWaypointPosXTwo = (cData.dog.path[1].worldPosition.x + (dirToWaypointXTwo * (minimumStickDistanceTWO + additionalDistanceToAddTWO)));
                                        var newOffsetWaypointPosYTwo = (cData.dog.path[1].worldPosition.y + (dirToWaypointYTwo * (minimumStickDistanceTWO +additionalDistanceToAddTWO)));
                                        var newOffsetWaypointPosTWO = { x: newOffsetWaypointPosXTwo, y: newOffsetWaypointPosYTwo };


                                        var rotationDOTRLTWO = SC_Utilities.Dot(dirToWaypointYTwo, -dirToWaypointXTwo, -cData.nData.nForward.x, -cData.nData.nForward.y);
                                        var rotationDOTFBTWO = SC_Utilities.Dot(dirToWaypointXTwo, dirToWaypointYTwo, cData.nData.nForward.x, cData.nData.nForward.y);
                                        var rotationDOTVELOTWO = SC_Utilities.Dot(dirToWaypointXTwo, dirToWaypointYTwo, cData.nData.nVelo.x, cData.nData.nVelo.y);
                                        var distToNodeONEExtended = SC_Utilities.GetDistance(newOffsetWaypointPosTWO, cData.nData.nCoord);


                                        if (rotationDOTVELO < 0.95 && rotationDOTVELOTWO < 0.95) {
                                            npc.InstantStop(cData.objt.nid);
                                        }


                                        //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);

                                        if (distToNodeTwo > minimumDistance) {
                                            npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                                        }
                                        else {
                                            if (rotationDOTVELO < 0.95 && rotationDOTVELOTWO < 0.95) {
                                                npc.InstantStop(cData.objt.nid);
                                            }


                                            if (cData.dog.path.length > 1) {
                                                cData.dog.path.shift();
                                            }
                                            else {
                                                cData.pfc = -2;
                                                cData.mSwtc = 1;
                                            }
                                            //if (cData.dog.path.length < 1)
                                            //{
                                            cData.stopSCM = someFrameCounterForStopTemp;

                                            if (cData.stopS == 0) {
                                                npc.Unstick(cData.objt.nid);
                                                npc.InstantStop(cData.objt.nid);
                                                cData.stopS = 0;
                                            }
                                            someFrameCounterForStopTemp = someFrameCounterForStopMax;
                                            //}
                                        }

                                        if (rotationDOTVELOTWO < 0.99) {
                                            cData.stopSCM = someFrameCounterForStopTemp;

                                            if (cData.stopS == 0) {
                                                npc.Unstick(cData.objt.nid);
                                                npc.InstantStop(cData.objt.nid);
                                                cData.stopS = 0;
                                            }
                                            someFrameCounterForStopTemp = someFrameCounterForStopMin;
                                        }
                                        else {
                                            npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                                        }
                                        /*
                                        if (distToNodeTwo > minimumDistance) {
                                            //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                            
                                            if (rotationDOTFBTWO >= 0.99) {
                                                if (rotationDOTVELOTWO < 0.99) {
                            
                                                    //npc.InstantStop(cData.objt.nid);
                            
                                                    npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                                                }
                                                else {
                                                    npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                                                }
                                            }
                                            else {
                                                if (rotationDOTVELOTWO < 0.99) {
                            
                                                    //npc.InstantStop(cData.objt.nid);
                            
                                                    npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                                                }
                                                else {
                                                    npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                                                }
                                            }
                                        }
                                        else {
                                            npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                                            cData.dog.path.shift();
                                            //npc.InstantStop(cData.objt.nid);
                                        }
                                    }
                                    else {

                                        if (cData.dog.path[0] != null) {
                                            cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
                                            cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);

                                            var distToNode = SC_Utilities.GetDistance(cData.dog.path[0].worldPosition, cData.nData.nCoord);

                                            var dirToWaypointX = ((cData.dog.path[0].worldPosition.x) - (cData.nData.nCoord.x)) / distToNode;
                                            var dirToWaypointY = ((cData.dog.path[0].worldPosition.y) - (cData.nData.nCoord.y)) / distToNode;

                                            var newOffsetWaypointPosX = (cData.dog.path[0].worldPosition.x + (dirToWaypointX * additionalDistanceToAdd));
                                            var newOffsetWaypointPosY = (cData.dog.path[0].worldPosition.y + (dirToWaypointY * additionalDistanceToAdd));

                                            var newOffsetWaypointPos = { x: newOffsetWaypointPosX, y: newOffsetWaypointPosY };

                                            var rotationDOTRL = SC_Utilities.Dot(dirToWaypointY, -dirToWaypointX, -cData.nData.nForward.x, -cData.nData.nForward.y);
                                            var rotationDOTFB = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nForward.x, cData.nData.nForward.y);
                                            var rotationDOTVELO = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nVelo.x, cData.nData.nVelo.y);

                                            //var distToNodeTwo = SC_Utilities.GetDistance(newOffsetWaypointPos, cData.nData.nCoord);

                                            //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);

                                            if (distToNode > minimumDistance) {
                                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                                            }
                                            else {
                                                if (rotationDOTVELO < 0.95) {
                                                    npc.InstantStop(cData.objt.nid);
                                                }


                                                if (cData.dog.path.length > 1) {
                                                    cData.dog.path.shift();
                                                }
                                                else {
                                                    cData.pfc = -2;
                                                    cData.mSwtc = 1;
                                                }
                                                //if (cData.dog.path.length < 1) {
                                                cData.stopSCM = someFrameCounterForStopTemp;

                                                if (cData.stopS == 0) {
                                                    npc.Unstick(cData.objt.nid);
                                                    npc.InstantStop(cData.objt.nid);
                                                    cData.stopS = 0;
                                                }
                                                someFrameCounterForStopTemp = someFrameCounterForStopMax;
                                                //}
                                            }

                                            if (rotationDOTVELO < 0.99) {
                                                cData.stopSCM = someFrameCounterForStopTemp;

                                                if (cData.stopS == 0) {
                                                    npc.Unstick(cData.objt.nid);
                                                    npc.InstantStop(cData.objt.nid);
                                                    cData.stopS = 0;
                                                }
                                                someFrameCounterForStopTemp = someFrameCounterForStopMin;
                                            }
                                            else {
                                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                                            }

                                        }




                                        /*if (distToNode > minimumDistance) {
                                            npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                            
                                            if (rotationDOTFB >= 0.99) {
                            
                                                if (rotationDOTVELO < 0.99) {
                                                    //npc.InstantStop(cData.objt.nid);
                                                    npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                                                }
                                                else {
                                                    npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                                                }
                                            }
                                            else {
                                                if (rotationDOTVELO < 0.99) {
                                                    //npc.InstantStop(cData.objt.nid);
                                                    npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                                                }
                                                else {
                                                    npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                                                }
                                            }
                                        }
                                        else {
                                            npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                                            cData.dog.path.shift();
                                            //npc.InstantStop(cData.objt.nid);
                                        }

                                    }





                                }
                                else {


                                }


                                break;
                            }
                        }
                    }*/







                    if (pathData != null) {


                        cData.pfc = pathData.currentCommand;

                        if (pathData.openSet != null && pathData.openSet.length > 0) {

                            //cData.dog.grid = pathData.grid;
                            cData.xtra = pathData.extra;
                            cData.dog.node = pathData.node;
                            cData.dog.openSet = pathData.openSet;
                            //cData.dog.closedSet = pathData.closedSet;
                            cData.dog.path = pathData.path;
                            cData.lsp = { x: cData.dog.node.worldPosition.x, y: cData.dog.node.worldPosition.y };

                            //cData.glip = initialPos;

                            if (cData.pfc == 10) {
                                //the path to target was found
                                ////---//////console.PrintError("cData.pfc == 10");
                                ////npc.Unstick(cData.objt.nid);
                                cData.mSwtc = 5;
                                cData.pfc = 2;
                                break;
                            }
                            else if (cData.pfc == 5) {
                                //The pathfind went through pfc == 3 first and was forced to activate a retracePath although the target tile is unwalkable.
                                ////---//////console.PrintError("cData.pfc == 5");
                                ////npc.Unstick(cData.objt.nid);

                                var initialPosX = (cData.nData.nCoord.x);
                                var initialPosY = (cData.nData.nCoord.y);
                                var initialPos = { x: initialPosX, y: initialPosY };

                                initialPos.x = Math.round(initialPos.x);
                                initialPos.y = Math.round(initialPos.y);

                                var currentWaypoint = { x: cData.cFWP.x, y: cData.cFWP.y };//{ x: initialPos.x, y: initialPos.y };
                                currentWaypoint.x = Math.round(currentWaypoint.x);
                                currentWaypoint.y = Math.round(currentWaypoint.y);


                                //var currentWaypointSeekerPos = { x: cData.dog.path[0].worldPosition.x, y: cData.dog.path[0].worldPosition.y };//{ x: initialPos.x, y: initialPos.y };
                                //currentWaypointSeekerPos.x = Math.round(currentWaypointSeekerPos.x);
                                //currentWaypointSeekerPos.y = Math.round(currentWaypointSeekerPos.y);


                                //var lastCurrentWaypoint = { x: cData.cLFWP.x, y: cData.cLFWP.y };//{ x: cData.ltp.x, y: cData.ltp.y };
                                //lastCurrentWaypoint.x = Math.round(lastCurrentWaypoint.x);
                                //lastCurrentWaypoint.y = Math.round(lastCurrentWaypoint.y);


                                //var dronePosRounded = { x: cData.nData.nCoord.x, y: cData.nData.nCoord.y };//{ x: cData.ltp.x, y: cData.ltp.y };
                                //dronePosRounded.x = Math.round(dronePosRounded.x);
                                //dronePosRounded.y = Math.round(dronePosRounded.y);


                                if (initialPos.x === currentWaypoint.x && initialPos.y === currentWaypoint.y) {
                                    //console.PrintError("00 arrived at waypoint.");
                                    cData.mSwtc = 4;
                                    cData.pfc = 4;
                                }
                                else {
                                    if (cData.dog.path != null) {
                                        if (cData.dog.path.length > 0) {
                                            ////console.PrintError("test 000");
                                            cData.mSwtc = 5;
                                            cData.pfc = 2;
                                            //cData.mSwtc = 1;
                                            //cData.pfc = -2;

                                        }
                                        else {
                                            //---//////console.PrintError("0 reset whole pathfind arrays");
                                            cData.mSwtc = 1;
                                            cData.pfc = -2;
                                            // cData.dog.path = null;
                                        }
                                    }
                                    else {
                                        //---//////console.PrintError("1 reset whole pathfind arrays");
                                        cData.mSwtc = 1;
                                        cData.pfc = -2;
                                        // cData.dog.path = null;
                                    }
                                }



                                break;
                            }
                            else if (cData.pfc == 4) {
                                ////---//////console.PrintError("cData.pfc == 4");
                                ////npc.Unstick(cData.objt.nid);
                                //console.PrintError("01 arrived at waypoint.");
                                cData.mSwtc = 4;
                                break;
                            }
                            else if (cData.pfc == 6) {

                            }
                            else if (cData.pfc == -2) {
                                //reset pathfind
                                ////---//////console.PrintError("cData.pfc == -2");
                                ////npc.Unstick(cData.objt.nid);
                                cData.mSwtc = 1;
                                cData.pfc = -2;
                                // cData.dog.path = null;
                                //cData.mSwtc = 4;
                                break;
                            }
                            else if (cData.pfc == 3) {

                                //to readd for tests
                                //console.PrintError("pathfind is looping");
                                //to readd for tests


                                /*var currentWaypoint = { x: cData.cFWP.x, y: cData.cFWP.y };
                                currentWaypoint.x = Math.round(currentWaypoint.x);
                                currentWaypoint.y = Math.round(currentWaypoint.y);

                                var lastCurrentWaypoint = { x: cData.cLFWP.x, y: cData.cLFWP.y };
                                lastCurrentWaypoint.x = Math.round(lastCurrentWaypoint.x);
                                lastCurrentWaypoint.y = Math.round(lastCurrentWaypoint.y);

                                if (currentWaypoint.x === lastCurrentWaypoint.x && currentWaypoint.y === lastCurrentWaypoint.y && cData.cFWPD >= minimumDistance) //original minimumDistance //but it bugs at some point... why? let's search
                                {
                                    cData.mSwtc = 4;
                                    cData.pfc = 4;
                                }
                                else {
                                    console.PrintError();
                                }*/






                                /*var distToNode = SC_Utilities.GetDistance(cData.dog.path[0].worldPosition, cData.nData.nCoord);

                                if ()
                                {

                                }*/


                                /////////////////////////////
                                //The target tile is an unwalkable tile and the grid analyzer has reached the target gridNTile. Force retracePath. Salvage the current pathfind tiles and check the distance of where the seekerposition is at, and when a distance is reached by the seeker, then currently (temporarily) conclude that the target is innaccessible
                                //and stop the pathfind.
                                //forceUseRetracePathSwtc = 1;
                                //cData.mSwtc = 3; //cData.pfc = 5;
                                //cData.pfc = 3;
                                /////////////////////////////


                                //var distToNode = SC_Utilities.GetDistance(cData.dog.path[0].worldPosition, cData.nData.nCoord);


                                /*pathData = null;
                                forceUseRetracePathSwtc = 1;
                                pathData = SC_PathFind_cc_2.npcPathFind(gridWorldSize, nodeRadius, cData, forceUseRetracePathSwtc);
                                cData.mSwtc = 3;*/

                                //


                                //////npc.InstantStop(cData.objt.nid);

                                //reset pathfind
                                ////---//////console.PrintError("cData.pfc == 3");
                                ////npc.Unstick(cData.objt.nid);
                                //cData.mSwtc = 4;



                                //break;
                            }
                            else if (cData.pfc == 0) {

                                /*if (cData.dog.path != null) {
                                    if (cData.dog.path.length > 0) {
                                        var currentWaypoint = { x: cData.cFWP.x, y: cData.cFWP.y };//{ x: initialPos.x, y: initialPos.y };
                                        currentWaypoint.x = Math.round(currentWaypoint.x);
                                        currentWaypoint.y = Math.round(currentWaypoint.y);
 
 
                                        var currentWaypointSeekerPos = { x: cData.dog.path[0].worldPosition.x, y: cData.dog.path[0].worldPosition.y };//{ x: initialPos.x, y: initialPos.y };
                                        currentWaypointSeekerPos.x = Math.round(currentWaypointSeekerPos.x);
                                        currentWaypointSeekerPos.y = Math.round(currentWaypointSeekerPos.y);
 
 
                                        var lastCurrentWaypoint = { x: cData.cLFWP.x, y: cData.cLFWP.y };//{ x: cData.ltp.x, y: cData.ltp.y };
                                        lastCurrentWaypoint.x = Math.round(lastCurrentWaypoint.x);
                                        lastCurrentWaypoint.y = Math.round(lastCurrentWaypoint.y);
 
 
                                        var dronePosRounded = { x: cData.nData.nCoord.x, y: cData.nData.nCoord.y };//{ x: cData.ltp.x, y: cData.ltp.y };
                                        dronePosRounded.x = Math.round(dronePosRounded.x);
                                        dronePosRounded.y = Math.round(dronePosRounded.y);
 
 
 
                                        var swtchFastCancel = -1;
 
                                        if (currentWaypointSeekerPos.x === currentWaypoint.x && currentWaypointSeekerPos.y === currentWaypoint.y && dronePosRounded.x === currentWaypoint.x && dronePosRounded.y === currentWaypoint.y ||
                                            currentWaypointSeekerPos.x === currentWaypoint.x && currentWaypointSeekerPos.y === currentWaypoint.y && dronePosRounded.x === currentWaypointSeekerPos.x && dronePosRounded.y === currentWaypointSeekerPos.y) {
                                            cData.mSwtc = 1;
                                            cData.pfc = -2;
                                            //////npc.InstantStop(cData.objt.nid);
                                            //    ////npc.InstantStop(cData.objt.nid);
                                            cData.dog.path.shift();
                                            cData.dog.path = null;
                                            //npc.Unstick(cData.objt.nid);
                                        }
                                    }
                                }*/





                                /*if (forceUseRetracePathSwtc == 1) {

                                    ////console.PrintError("forceUseRetracePathSwtc == 1 " + forceUseRetracePathSwtc);
                                    //cData.dog.openSet.length > 50 //maxOpensetLengthWhenThePathfindLoops = 11 // 25 lags
                                }
                                else {

                                    ////console.PrintError("forceUseRetracePathSwtc == 0 or else " + forceUseRetracePathSwtc);
                                    continue;
                                }*/

                            }
                            else if (cData.pfc == 2) {
                                ////console.PrintError("cData.pfc == 2");

                                /////////////////////////////
                                //The target tile is an unwalkable tile and the grid analyzer has reached the target gridNTile. Force retracePath. Salvage the current pathfind tiles and check the distance of where the seekerposition is at, and when a distance is reached by the seeker, then currently (temporarily) conclude that the target is innaccessible
                                //and stop the pathfind.
                                //forceUseRetracePathSwtc = 1;
                                //cData.pfc = 5;
                                /////////////////////////////



                                ////npc.Unstick(cData.objt.nid);
                                //cData.mSwtc = 4;
                                //break;



                                cData.mSwtc = 5;
                                cData.pfc = 2;

                            }
                            else {
                                ////console.PrintError("cData.pfc == else");
                            }

                            ////console.PrintError("cData.pfc == " + cData.pfc);
                            /*else {
                                //---//////console.PrintError("cData.pfc == else");
 
                                /////////////////////////////
                                //The target tile is an unwalkable tile and the grid analyzer has reached the target gridNTile. Force retracePath. Salvage the current pathfind tiles and check the distance of where the seekerposition is at, and when a distance is reached by the seeker, then currently (temporarily) conclude that the target is innaccessible
                                //and stop the pathfind.
                                //forceUseRetracePathSwtc = 1;
                                //cData.pfc = 5;
                                /////////////////////////////
 
 
 
                                ////npc.Unstick(cData.objt.nid);
                                //cData.mSwtc = 4;
                                //break;
                            }*/


                        }
                        else //pathData.openSet == null
                        {
                            console.PrintError("###NULL OPENSET###"); //nothing was added inside of OPENSET


                            ////---//////console.PrintError("###NULL### " + " cData.pfc == " + cData.pfc);





                            if (cData.log != null) {
                                if (cData.log.length > 0) {
                                    //---//////console.PrintError("@@@ openset null - list of grids !null - try and make a path out of it.");


                                    var currentWaypoint = { x: cData.cFWP.x, y: cData.cFWP.y };
                                    currentWaypoint.x = Math.round(currentWaypoint.x);
                                    currentWaypoint.y = Math.round(currentWaypoint.y);

                                    var lastCurrentWaypoint = { x: cData.cLFWP.x, y: cData.cLFWP.y };
                                    lastCurrentWaypoint.x = Math.round(lastCurrentWaypoint.x);
                                    lastCurrentWaypoint.y = Math.round(lastCurrentWaypoint.y);




                                    /*if (currentWaypoint.x === lastCurrentWaypoint.x && currentWaypoint.y === lastCurrentWaypoint.y && cData.cFWPD >= minimumDistance) //original minimumDistance //but it bugs at some point... why? let's search
                                    {
                                        cData.mSwtc = 1;
                                        cData.pfc = -2;
                                    }
                                    else
                                    {
                                        cData.mSwtc = 1;
                                        cData.pfc = -2;
                                    }*/



                                    cData.mSwtc = 5;
                                    cData.pfc = 2;

                                    //cData.mSwtc = 5;
                                    //cData.pfc = 2;

                                    //cData.mSwtc = 1;
                                    //cData.pfc = -2;


                                    break;
                                }
                                else {
                                    cData.mSwtc = 1;
                                    cData.pfc = -2;
                                    console.PrintError("@0###NULL### ");
                                }
                            }
                            else {
                                cData.mSwtc = 1;
                                cData.pfc = -2;
                                console.PrintError("@1###NULL### ");
                            }
                        }
                    }
                    else {
                        cData.mSwtc = 1;
                        cData.pfc = -2;
                        console.PrintError("@2###NULL### ");
                    }

                }
                console.PrintError("FramePathfindIteratedCounter: " + FramePathfindIteratedCounter);
                FramePathfindIteratedCounter++;
            }

        }









        //to readd
        //to readd
        //to readd
        /*
        if (cData.pfc == 1) //something wrong or not restart pathfind completely.
        {
            //---//////console.PrintError("1npcPathFind cData.pfc == 1");
            cData.mSwtc = 1;
        }
        else if (cData.pfc == 2)
        {
 
 
            //for the pathfind to arrive here, the pathfind got initiated and got a grid but the waypoint is unwalkable and if there is a grid, there is a path and if there is a path then retrieve it and reroll the pathfind path a couple tiles earlier then the unwalkable target tile and check for walkability.
            //for the pathfind to arrive here, the pathfind got initiated and got a grid but the waypoint is unwalkable and if there is a grid, there is a path and if there is a path then retrieve it and reroll the pathfind path a couple tiles earlier then the unwalkable target tile and check for walkability.
            //for the pathfind to arrive here, the pathfind got initiated and got a grid but the waypoint is unwalkable and if there is a grid, there is a path and if there is a path then retrieve it and reroll the pathfind path a couple tiles earlier then the unwalkable target tile and check for walkability.
 
            //to readd
            //to readd
            //to readd
            //cData.mSwtc = 5;
            ////---//////console.PrintError("1npcPathFind cData.pfc == 2");
            //to readd
            //to readd
            //to readd
 
            break;
 
            //cData.mSwtc = 1;
        }*/
        //to readd
        //to readd
        //to readd


        /*else if (cData.mSwtc == 4) //will double check later i am still using this elsewhere.
        {
            //---//////console.PrintError("***END OF PATHFIND***");
            cData.mSwtc = 50;
 
 
            //move together =>
            if (cData.stopS == 1) {
                if (cData.stopSC >= cData.stopSCM) {
                    cData.stopS = 0;
                    cData.stopSC = 0;
                }
                cData.stopSC++;
            }
            cData.cLFWP = cData.cFWP;
 
            return cData;
            //move together <=
        }*/




        /*else if (cData.mSwtc == 5) {
            if (cData.dog.path != null) {
                if (cData.dog.path.length > 0) {
                    cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
                    cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);

                    var distToNode = SC_Utilities.GetDistance(cData.dog.path[0].worldPosition, cData.nData.nCoord);

                    var dirToWaypointX = ((cData.dog.path[0].worldPosition.x) - (cData.nData.nCoord.x)) / distToNode;
                    var dirToWaypointY = ((cData.dog.path[0].worldPosition.y) - (cData.nData.nCoord.y)) / distToNode;

                    var newOffsetWaypointPosX = (cData.dog.path[0].worldPosition.x + (dirToWaypointX * 4.5));
                    var newOffsetWaypointPosY = (cData.dog.path[0].worldPosition.y + (dirToWaypointY * 4.5));

                    var newOffsetWaypointPos = { x: newOffsetWaypointPosX, y: newOffsetWaypointPosY };

                    var rotationDOTRL = SC_Utilities.Dot(dirToWaypointY, -dirToWaypointX, -cData.nData.nForward.x, -cData.nData.nForward.y);
                    var rotationDOTFB = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nForward.x, cData.nData.nForward.y);
                    var rotationDOTVELO = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nVelo.x, cData.nData.nVelo.y);

                    var distToNodeTwo = SC_Utilities.GetDistance(newOffsetWaypointPos, cData.nData.nCoord);

                    if (distToNodeTwo > 0.25) {
                        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);

                        if (rotationDOTFB >= 0.99) {
                            //npc.Unstick(cData.objt.nid);
                            //npc.GoForward(cData.objt.nid);
                            npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                        }
                        else {
                            if (rotationDOTVELO < 0.99) {
                                //npc.InstantStop(cData.objt.nid);
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                            }
                            else {
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                            }
                        }
                    }
                    else {
                        //npc.Unstick(cData.objt.nid);
                        cData.dog.path.shift();
                        //npc.Stop(cData.objt.nid);
                        ////npc.InstantStop(cData.objt.nid);
                        //npc.InstantStop(cData.objt.nid);

                    }
                }
                else {
                    if (cData.cFWPD > 4.5) {
                        cData.mSwtc = 1;
                    }
                    else {
                        //console.PrintError("reached waypoint");
                        cData.mSwtc = 9;
                    }
                }
            }
            else {
                if (cData.cFWPD > 4.5) {
                    cData.mSwtc = 1;
                }
                else {
                    //console.PrintError("reached waypoint");
                    cData.mSwtc = 9;
                }
            }
        }*/








        //re-add else if it lags
        //it doesn't lag but i add else anyway so that there is a relief of work this frame since the pathfind already might have ran a lot and the arrays of data/objects might be big. to continue to test and developp.
        if (cData.mSwtc == 5) {

            //console.PrintError("is it ever arriving here");
            if (frameDistanceIncreaser > additionalDistanceToAddMax) {
                frameDistanceIncreaser = 0;
            }

            additionalDistanceToAdd = (frameDistanceIncreaser * frameDistanceIncreaserBase);


            if (frameDistanceIncreaserTWO > additionalDistanceToAddMaxTWO) {
                frameDistanceIncreaserTWO = 0;
            }

            additionalDistanceToAddTWO = (frameDistanceIncreaserTWO * frameDistanceIncreaserBaseTWO);






            var foundNewTargetWaypoint = -1;

            //COMPARE THE POSITION OF THE PLAYER AND IT'S DIRECTION AND VELOCITY to path[0] and to path[1]. IF THE VELOCITY OF THE DRONE IS PAST THE WAYPOINT BUT THAT THE DRONE POSITION ROUNDED IS PATH[1] and that the velocity is towards path[1], then path.shift to remove path[0] and follow the next nodes path[1] that should now be path[0] and so forth.
            //and just to make it better, i should also make a path[2] check also.

            if (cData.dog.path != null) {
                if (cData.dog.path.length > 0) {



                    if (cData.pfc == 2) {

                        //LOOP THE WHOLE PATH FOR STICK TO POINT FOR SPLINE STICKTOPOINT MOVES WITHOUT STOPS/ DRIFT MOVES.
                        //LOOP THE WHOLE PATH FOR STICK TO POINT FOR SPLINE STICKTOPOINT MOVES WITHOUT STOPS/ DRIFT MOVES.
                        //LOOP THE WHOLE PATH FOR STICK TO POINT FOR SPLINE STICKTOPOINT MOVES WITHOUT STOPS/ DRIFT MOVES.
                        //LOOP THE WHOLE PATH FOR STICK TO POINT FOR SPLINE STICKTOPOINT MOVES WITHOUT STOPS/ DRIFT MOVES.
                        //LOOP THE WHOLE PATH FOR STICK TO POINT FOR SPLINE STICKTOPOINT MOVES WITHOUT STOPS/ DRIFT MOVES.
                        //LOOP THE WHOLE PATH FOR STICK TO POINT FOR SPLINE STICKTOPOINT MOVES WITHOUT STOPS/ DRIFT MOVES.
                        //LOOP THE WHOLE PATH FOR STICK TO POINT FOR SPLINE STICKTOPOINT MOVES WITHOUT STOPS/ DRIFT MOVES.
                        //LOOP THE WHOLE PATH FOR STICK TO POINT FOR SPLINE STICKTOPOINT MOVES WITHOUT STOPS/ DRIFT MOVES.
                        //LOOP THE WHOLE PATH FOR STICK TO POINT FOR SPLINE STICKTOPOINT MOVES WITHOUT STOPS/ DRIFT MOVES.
                        //LOOP THE WHOLE PATH FOR STICK TO POINT FOR SPLINE STICKTOPOINT MOVES WITHOUT STOPS/ DRIFT MOVES.
                        //LOOP THE WHOLE PATH FOR STICK TO POINT FOR SPLINE STICKTOPOINT MOVES WITHOUT STOPS/ DRIFT MOVES.
                        //LOOP THE WHOLE PATH FOR STICK TO POINT FOR SPLINE STICKTOPOINT MOVES WITHOUT STOPS/ DRIFT MOVES.
                        //LOOP THE WHOLE PATH FOR STICK TO POINT FOR SPLINE STICKTOPOINT MOVES WITHOUT STOPS/ DRIFT MOVES.
                        //LOOP THE WHOLE PATH FOR STICK TO POINT FOR SPLINE STICKTOPOINT MOVES WITHOUT STOPS/ DRIFT MOVES.
                        //LOOP THE WHOLE PATH FOR STICK TO POINT FOR SPLINE STICKTOPOINT MOVES WITHOUT STOPS/ DRIFT MOVES.





                        //CURRENT
                        //CURRENT
                        //during the pathfind at mSwtc == 3, openset came out to be null but the cData.log (list of grids) isn't null. Let's try and salvage a path to the (possibly null) target waypoint
                        //---//////console.PrintError("during the pathfind at mSwtc == 3, openset came out to be null but the cData.log (list of grids) isn't null. Let's try and salvage a path to the (possibly null) target waypoint");
                        //CURRENT
                        //CURRENT
                        //since openset was null and that there is a list of grids (cData.log) available, then only the last iteration (grid) of the whole List of Grids has the null openset, which means the path array's last indexes probably have unwalkable tiles. so we loop to remove them and if the path isn't
                        //null after those unwalkable tiles are removed, then the drone will have to move there. more testing to come.

                        for (var i = cData.dog.path.length - 1; i >= 0; i--) {

                            if (cData.dog.path[i].boolWalk == 1) //walkable.
                            {
                                //

                                if (i != cData.dog.path.length - 1) {
                                    cData.dog.path.splice(i + 1, 1);
                                }


                                foundNewTargetWaypoint = 1;
                                break;
                            }
                            else //unwalkable.
                            {
                                cData.dog.path.splice(i, 1);
                            }
                        }
                        ////console.PrintError("cData.dog.path.length: " + cData.dog.path.length);


                        /*
                        if (someOtherFrameCounterForStopResettingPathfindAtEveryFrame > someOtherFrameCounterForStopResettingPathfindAtEveryFrameMax) //cData.dog.path.length < 3 || 
                        {
                            if (cData.footStepsCounter >= cData.footStepsCounterMAX) {

                                var posNPClastframeX = cData.lip.x;
                                var posNPClastframeY = cData.lip.y;
                                var posNPClastframe = { x: posNPClastframeX, y: posNPClastframeY };

                                posNPClastframe.x = Math.round(posNPClastframe.x);
                                posNPClastframe.y = Math.round(posNPClastframe.y);

                                cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
                                cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);

                                if (currentWaypoint.x != lastCurrentWaypoint.x && currentWaypoint.y != lastCurrentWaypoint.y) {
                                    cData.mSwtc = 1; //1
                                    cData.pfc = -2;

                                }
                                someOtherFrameCounterForStopResettingPathfindAtEveryFrame = 0;
                                cData.footStepsCounter = 0;
                            }
                            /*if (cData.dog.path.length < 3) {
                                if (currentWaypoint.x != lastCurrentWaypoint.x && currentWaypoint.y != lastCurrentWaypoint.y) {
                                    //cData.mSwtc = 3; //1
                                    //cData.pfc = 0;
                                    cData.mSwtc = 1; //1
                                    cData.pfc = -2;
                                    someOtherFrameCounterForStopResettingPathfindAtEveryFrame = 0;
                                }
                                cData.footStepsCounter = 0;
                            }
                            else {
                                
                            }

                        }
                        cData.footStepsCounter++;*/

                        if (someOtherFrameCounterForStopResettingPathfindAtEveryFrame > someOtherFrameCounterForStopResettingPathfindAtEveryFrameMax) //cData.dog.path.length < 3 || 
                        {
                            if (cData.footStepsCounter >= cData.footStepsCounterMAX) {

                                var initialPos = { x: cData.nData.nCoord.x, y: cData.nData.nCoord.y };
                                initialPos.x = Math.round(initialPos.x);
                                initialPos.y = Math.round(initialPos.y);

                                var currentWaypoint = { x: cData.cFWP.x, y: cData.cFWP.y };//{ x: initialPos.x, y: initialPos.y };
                                currentWaypoint.x = Math.round(currentWaypoint.x);
                                currentWaypoint.y = Math.round(currentWaypoint.y);

                                var lastCurrentWaypoint = { x: cData.cLFWP.x, y: cData.cLFWP.y };
                                lastCurrentWaypoint.x = Math.round(lastCurrentWaypoint.x);
                                lastCurrentWaypoint.y = Math.round(lastCurrentWaypoint.y);

                                //TO READD
                                if (initialPos.x != currentWaypoint.x && initialPos.y != currentWaypoint.y) // && currentWaypoint.x === lastCurrentWaypoint.x && currentWaypoint.y === lastCurrentWaypoint.y
                                {
                                    cData.mSwtc = 1; //1
                                    cData.pfc = -2;
                                }
                                //TO READD


                                /*var posNPClastframeX = cData.lip.x;
                                var posNPClastframeY = cData.lip.y;
                                var posNPClastframe = { x: posNPClastframeX, y: posNPClastframeY };
                
                                posNPClastframe.x = Math.round(posNPClastframe.x);
                                posNPClastframe.y = Math.round(posNPClastframe.y);
                
                                cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
                                cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);
                
                                if (currentWaypoint.x != lastCurrentWaypoint.x && currentWaypoint.y != lastCurrentWaypoint.y) {
                                    cData.mSwtc = 1; //1
                                    cData.pfc = -2;
                
                                }*/
                                someOtherFrameCounterForStopResettingPathfindAtEveryFrame = 0;
                                cData.footStepsCounter = 0;
                            }
                            /*if (cData.dog.path.length < 3) {
                                if (currentWaypoint.x != lastCurrentWaypoint.x && currentWaypoint.y != lastCurrentWaypoint.y) {
                                    //cData.mSwtc = 3; //1
                                    //cData.pfc = 0;
                                    cData.mSwtc = 1; //1
                                    cData.pfc = -2;
                                    someOtherFrameCounterForStopResettingPathfindAtEveryFrame = 0;
                                }
                                cData.footStepsCounter = 0;
                            }
                            else {
                                
                            }*/

                        }
                        cData.footStepsCounter++;


                        cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
                        cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);

                        var distToNode = SC_Utilities.GetDistance(cData.dog.path[0].worldPosition, cData.nData.nCoord);

                        var dirToWaypointX = ((cData.dog.path[0].worldPosition.x) - (cData.nData.nCoord.x)) / distToNode;
                        var dirToWaypointY = ((cData.dog.path[0].worldPosition.y) - (cData.nData.nCoord.y)) / distToNode;

                        var newOffsetWaypointPosX = (cData.dog.path[0].worldPosition.x + (dirToWaypointX * (minimumStickDistance * additionalDistanceToAdd)));
                        var newOffsetWaypointPosY = (cData.dog.path[0].worldPosition.y + (dirToWaypointY * (minimumStickDistance * additionalDistanceToAdd)));

                        var newOffsetWaypointPos = { x: newOffsetWaypointPosX, y: newOffsetWaypointPosY };

                        var rotationDOTRL = SC_Utilities.Dot(dirToWaypointY, -dirToWaypointX, -cData.nData.nForward.x, -cData.nData.nForward.y);
                        var rotationDOTFB = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nForward.x, cData.nData.nForward.y);
                        var rotationDOTVELO = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nVelo.x, cData.nData.nVelo.y);

                        var distToNodeZeroExtended = SC_Utilities.GetDistance(newOffsetWaypointPos, cData.nData.nCoord);



                        var angle = ship.GetRotation(cData.objt.nid);
                        var radToDeg = angle * (180.0 / Math.PI);

                        radToDeg = SC_Utilities.normalizedegrees(radToDeg);


                        //var hitObject = ship.GetFirstObstacleOnTheWay(cData.objt.nid, 1);
                        /*var hitObject = ship.GetObstacleOnRay(cData.objt.nid, angle, 1);

                        if (hitObject != null) {
                            console.PrintError("hitObject: " + hitObject.type);
                        }
                        else {
                            console.PrintError("hitObject == null ");
                        }*/



                        if (cData.dog.path[1] != null) // && cData.dog.path[0] != null
                        {
                            //var offSetDistTwo = 3.5 + additionalDistanceToAdd;
                            //var offSetDistTwoTwo = 3.5 + additionalDistanceToAdd;

                            cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
                            cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);

                            var distToNode = SC_Utilities.GetDistance(cData.dog.path[0].worldPosition, cData.nData.nCoord);

                            var dirToWaypointX = ((cData.dog.path[0].worldPosition.x) - (cData.nData.nCoord.x)) / distToNode;
                            var dirToWaypointY = ((cData.dog.path[0].worldPosition.y) - (cData.nData.nCoord.y)) / distToNode;

                            var newOffsetWaypointPosX = (cData.dog.path[0].worldPosition.x + (dirToWaypointX * additionalDistanceToAdd));
                            var newOffsetWaypointPosY = (cData.dog.path[0].worldPosition.y + (dirToWaypointY * additionalDistanceToAdd));

                            var newOffsetWaypointPos = { x: newOffsetWaypointPosX, y: newOffsetWaypointPosY };

                            var rotationDOTRL = SC_Utilities.Dot(dirToWaypointY, -dirToWaypointX, -cData.nData.nForward.x, -cData.nData.nForward.y);
                            var rotationDOTFB = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nForward.x, cData.nData.nForward.y);
                            var rotationDOTVELO = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nVelo.x, cData.nData.nVelo.y);


                            var distToNodeTwo = SC_Utilities.GetDistance(cData.dog.path[1].worldPosition, cData.nData.nCoord);
                            var dirToWaypointXTwo = ((cData.dog.path[1].worldPosition.x) - (cData.nData.nCoord.x)) / distToNodeTwo;
                            var dirToWaypointYTwo = ((cData.dog.path[1].worldPosition.y) - (cData.nData.nCoord.y)) / distToNodeTwo;
                            var newOffsetWaypointPosXTwo = (cData.dog.path[1].worldPosition.x + (dirToWaypointXTwo * (minimumStickDistanceTWO +additionalDistanceToAddTWO)));
                            var newOffsetWaypointPosYTwo = (cData.dog.path[1].worldPosition.y + (dirToWaypointYTwo * (minimumStickDistanceTWO +additionalDistanceToAddTWO)));
                            var newOffsetWaypointPosTWO = { x: newOffsetWaypointPosXTwo, y: newOffsetWaypointPosYTwo };


                            var rotationDOTRLTWO = SC_Utilities.Dot(dirToWaypointYTwo, -dirToWaypointXTwo, -cData.nData.nForward.x, -cData.nData.nForward.y);
                            var rotationDOTFBTWO = SC_Utilities.Dot(dirToWaypointXTwo, dirToWaypointYTwo, cData.nData.nForward.x, cData.nData.nForward.y);
                            var rotationDOTVELOTWO = SC_Utilities.Dot(dirToWaypointXTwo, dirToWaypointYTwo, cData.nData.nVelo.x, cData.nData.nVelo.y);
                            var distToNodeONEExtended = SC_Utilities.GetDistance(newOffsetWaypointPosTWO, cData.nData.nCoord);


                            if (rotationDOTVELO < 0 && rotationDOTVELOTWO < 0) {
                                npc.InstantStop(cData.objt.nid);
                            }



                            //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);

                            if (distToNodeTwo > minimumDistance) {
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                            }
                            else {
                                if (rotationDOTVELO < 0 && rotationDOTVELOTWO < 0) {
                                    npc.InstantStop(cData.objt.nid);
                                }

                                if (cData.dog.path.length > 1) {
                                    cData.dog.path.shift();
                                }
                                else {
                                    cData.pfc = -2;
                                    cData.mSwtc = 1;
                                }

                                //if (cData.dog.path.length < 1)
                                //{
                                cData.stopSCM = someFrameCounterForStopTemp;

                                if (cData.stopS == 0) {
                                    npc.Unstick(cData.objt.nid);
                                    npc.InstantStop(cData.objt.nid);
                                    cData.stopS = 0;
                                }
                                someFrameCounterForStopTemp = someFrameCounterForStopMax;
                                //}
                            }

                            if (rotationDOTVELOTWO < 0.99) {
                                cData.stopSCM = someFrameCounterForStopTemp;

                                if (cData.stopS == 0) {
                                    npc.Unstick(cData.objt.nid);
                                    npc.InstantStop(cData.objt.nid);
                                    cData.stopS = 0;
                                }
                                someFrameCounterForStopTemp = someFrameCounterForStopMin;
                            }
                            else {
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                            }
                            /*
                            if (distToNodeTwo > minimumDistance) {
                                //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);

                                if (rotationDOTFBTWO >= 0.99) {
                                    if (rotationDOTVELOTWO < 0.99) {

                                        //npc.InstantStop(cData.objt.nid);

                                        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                                    }
                                    else {
                                        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                                    }
                                }
                                else {
                                    if (rotationDOTVELOTWO < 0.99) {

                                        //npc.InstantStop(cData.objt.nid);

                                        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                                    }
                                    else {
                                        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                                    }
                                }
                            }
                            else {
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosTWO.x, newOffsetWaypointPosTWO.y, 0);
                                cData.dog.path.shift();
                                //npc.InstantStop(cData.objt.nid);
                            }*/
                        }
                        else {

                            npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);

                            if (distToNode > minimumDistance) {
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                            }
                            else {
                                if (rotationDOTVELO < 0) {
                                    npc.InstantStop(cData.objt.nid);
                                }


                                if (cData.dog.path.length > 1) {
                                    cData.dog.path.shift();
                                }
                                else {
                                    cData.pfc = -2;
                                    cData.mSwtc = 1;
                                }
                                //if (cData.dog.path.length < 1) {
                                cData.stopSCM = someFrameCounterForStopTemp;

                                if (cData.stopS == 0) {
                                    npc.Unstick(cData.objt.nid);
                                    npc.InstantStop(cData.objt.nid);
                                    cData.stopS = 0;
                                }
                                someFrameCounterForStopTemp = someFrameCounterForStopMax;
                                //}
                            }

                            if (rotationDOTVELO < 0.99) {
                                cData.stopSCM = someFrameCounterForStopTemp;

                                if (cData.stopS == 0) {
                                    npc.Unstick(cData.objt.nid);
                                    npc.InstantStop(cData.objt.nid);
                                    cData.stopS = 0;
                                }
                                someFrameCounterForStopTemp = someFrameCounterForStopMin;
                            }
                            else {
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                            }






                            /*if (distToNode > minimumDistance) {
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);

                                if (rotationDOTFB >= 0.99) {

                                    if (rotationDOTVELO < 0.99) {
                                        //npc.InstantStop(cData.objt.nid);
                                        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                                    }
                                    else {
                                        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                                    }
                                }
                                else {
                                    if (rotationDOTVELO < 0.99) {
                                        //npc.InstantStop(cData.objt.nid);
                                        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                                    }
                                    else {
                                        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                                    }
                                }
                            }
                            else {
                                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                                cData.dog.path.shift();
                                //npc.InstantStop(cData.objt.nid);
                            }*/

                        }








                    }
                }
                else //(cData.dog.path.length <= 0)
                {

                    ////console.PrintError("@@@TEST@@@" + " " + forceUseRetracePathSwtc);
                    cData.mSwtc = 1;
                    cData.pfc = -2;
                    //cData.dog.path.shift();
                    cData.footStepsCounter = 0;
                    //cData.dog.path = null;

                    //npc.Unstick(cData.objt.nid);
                    ////npc.InstantStop(cData.objt.nid);

                    /*if (someOtherFrameCounterForStopResettingPathfindAtEveryFrame > someOtherFrameCounterForStopResettingPathfindAtEveryFrameMax) {
                        if (currentWaypoint.x != lastCurrentWaypoint.x && currentWaypoint.y != lastCurrentWaypoint.y) {
                            //cData.mSwtc = 3; //1
                            //cData.pfc = 0;
                            cData.mSwtc = 1; //1
                            cData.pfc = -2;
                        }
                        someOtherFrameCounterForStopResettingPathfindAtEveryFrame = 0;
                    }*/


                    /*
                    if (forceUseRetracePathSwtc == 1) {
                        cData.mSwtc = 1;
                        cData.pfc = -2;
                        cData.dog.path.shift();
                        cData.dog.path = null;
    
                        //npc.Unstick(cData.objt.nid);
                        ////npc.InstantStop(cData.objt.nid);
    
                        /*if (cData.swtchForPathfind == 0) {
                            ////npc.InstantStop(cData.objt.nid);
                            ////npc.InstantStop(cData.objt.nid);
                            ////npc.InstantStop(cData.objt.nid);
                            //npc.Unstick(cData.objt.nid);
                        }*/
                    // cData.dog.path = null;

                    /*cData.mSwtc = 3;c
                    cData.pfc = 2;
                    forceUseRetracePathSwtc = 1;
    
                }
                else {
                    cData.mSwtc = 1;
                    cData.pfc = -2;
    
    
                    //cData.mSwtc = 3;
                    //cData.pfc = 2;
                    //forceUseRetracePathSwtc = 1;
                }*/
                }
            }
            else //if (cData.dog.path == null)
            {

                //cData.mSwtc = 3;
                //forceUseRetracePathSwtc = 1;

                if (cData.log != null) {
                    if (cData.log.length > 0) {
                        ////console.PrintError("@@@ 22 openset null - list of grids !null - try and make a path out of it.");

                        if (cData.dog.path != null) {
                            if (cData.dog.path.length > 0) {
                                cData.mSwtc = 5;
                                cData.pfc = 2;
                                forceUseRetracePathSwtc = 1;
                            }
                            else {
                                cData.mSwtc = 1;
                                cData.pfc = -2;


                                /*cData.mSwtc = 3;
                                cData.pfc = 2;
                                forceUseRetracePathSwtc = 1;*/
                            }
                        }
                        else {
                            cData.mSwtc = 1;
                            cData.pfc = -2;


                            /*cData.mSwtc = 3;
                            cData.pfc = 2;
                            forceUseRetracePathSwtc = 1;*/
                        }
                    }
                    else {
                        //---//////console.PrintError("@0###NULL### ");
                    }
                }
            }

            frameDistanceIncreaser++;
            frameDistanceIncreaserTWO++;
        }

        /*}
        else {

            //////console.PrintError("cfwpd < 1");

            cData.mSwtc = 1;
            cData.pfc = -2;
            cData.dog.path = null;
            cData.footStepsCounter = 0;

        }*/

        /*if (someOtherFrameCounterForStopResettingPathfindAtEveryFrame > someOtherFrameCounterForStopResettingPathfindAtEveryFrameMax) {
            if (currentWaypoint.x != lastCurrentWaypoint.x && currentWaypoint.y != lastCurrentWaypoint.y) {
                //cData.mSwtc = 3; //1
                //cData.pfc = 0;
                cData.mSwtc = 1; //1
                cData.pfc = -2;
            }
            someOtherFrameCounterForStopResettingPathfindAtEveryFrame = 0;
        }*/

        someOtherFrameCounterForStopResettingPathfindAtEveryFrame++;














        if (cData.stopS == 1) {
            if (cData.stopSC >= cData.stopSCM) {
                cData.stopS = 0;
                cData.stopSC = 0;
            }
            cData.stopSC++;
        }


        if (!isNaN(rotationDOTVELO)) {
            lastFramerotationDOTVELO = rotationDOTVELO;
        }


        if (!isNaN(rotationDOTVELOTWO)) {
            lastFramerotationDOTVELOTWO = rotationDOTVELOTWO;
        }














        if (cData.mSwtc == 4) //will double check later i am still using this elsewhere.
        {
            npc.Unstick(cData.objt.nid);
            npc.InstantStop(cData.objt.nid);

            var initialPos = { x: cData.nData.nCoord.x, y: cData.nData.nCoord.y };
            initialPos.x = Math.round(initialPos.x);
            initialPos.y = Math.round(initialPos.y);

            var currentWaypoint = { x: cData.cFWP.x, y: cData.cFWP.y };//{ x: initialPos.x, y: initialPos.y };
            currentWaypoint.x = Math.round(currentWaypoint.x);
            currentWaypoint.y = Math.round(currentWaypoint.y);

            var lastCurrentWaypoint = { x: cData.cLFWP.x, y: cData.cLFWP.y };//{ x: cData.ltp.x, y: cData.ltp.y };
            lastCurrentWaypoint.x = Math.round(lastCurrentWaypoint.x);
            lastCurrentWaypoint.y = Math.round(lastCurrentWaypoint.y);



            if (initialPos.x === currentWaypoint.x && initialPos.y === currentWaypoint.y) {
                //console.PrintError("03 arrived at waypoint.");
                cData.mSwtc = 4;
                cData.pfc = 4;
                //npc.InstantStop(cData.objt.nid);
            }
            else {
                cData.mSwtc = 1;
                cData.pfc = -2;
                //cData.dog.path = null;
            }


            /*
            if (cData.dog != null) {
                if (cData.dog.path != null) {
                    if (cData.dog.path[0] != null) {
                        var currentPathZero = { x: cData.dog.path[0].worldPosition.x, y: cData.dog.path[0].worldPosition.y };//{ x: initialPos.x, y: initialPos.y };
                        currentPathZero.x = Math.round(currentPathZero.x);
                        currentPathZero.y = Math.round(currentPathZero.y);

                        if (initialPos.x === currentWaypoint.x && initialPos.y === currentWaypoint.y || initialPos.x === currentPathZero.x && initialPos.y === currentPathZero.y) {
                            //console.PrintError("02 arrived at waypoint.");
                            cData.mSwtc = 4;
                            cData.pfc = 4;
                            //npc.InstantStop(cData.objt.nid);
                        }
                        else {
                            cData.mSwtc = 1;
                            cData.pfc = -2;
                            cData.dog.path = null;
                        }
                    }
                    else {
                        if (initialPos.x === currentWaypoint.x && initialPos.y === currentWaypoint.y) {
                            //console.PrintError("03 arrived at waypoint.");
                            cData.mSwtc = 4;
                            cData.pfc = 4;
                            //npc.InstantStop(cData.objt.nid);
                        }
                        else {
                            cData.mSwtc = 1;
                            cData.pfc = -2;
                            cData.dog.path = null;
                        }
                    }
                }
                else {
                    if (initialPos.x === currentWaypoint.x && initialPos.y === currentWaypoint.y) {
                        //console.PrintError("04 arrived at waypoint.");
                        cData.mSwtc = 4;
                        cData.pfc = 4;
                        //npc.InstantStop(cData.objt.nid);
                    }
                    else {
                        cData.mSwtc = 1;
                        cData.pfc = -2;
                        cData.dog.path = null;
                    }
                }
            

                /*if (cData.dog != null) {
                    if (cData.dog.path != null) {
                        if (cData.dog.path[0] != null) {
                            var currentPathZero = { x: cData.dog.path[0].worldPosition.x, y: cData.dog.path[0].worldPosition.y };//{ x: initialPos.x, y: initialPos.y };
                            currentPathZero.x = Math.round(currentPathZero.x);
                            currentPathZero.y = Math.round(currentPathZero.y);

                            if (initialPos.x === currentWaypoint.x && initialPos.y === currentWaypoint.y || initialPos.x === currentPathZero.x && initialPos.y === currentPathZero.y) {
                                //console.PrintError("02 arrived at waypoint.");
                                cData.mSwtc = 4;
                                cData.pfc = 4;
                            }
                        }
                        else {
                            if (initialPos.x === currentWaypoint.x && initialPos.y === currentWaypoint.y) {
                                //console.PrintError("03 arrived at waypoint.");
                                cData.mSwtc = 4;
                                cData.pfc = 4;
                            }
                        }
                    }
                    else {
                        if (initialPos.x === currentWaypoint.x && initialPos.y === currentWaypoint.y) {
                            //console.PrintError("04 arrived at waypoint.");
                            cData.mSwtc = 4;
                            cData.pfc = 4;
                        }
                    }
                }
                else {
                    if (initialPos.x === currentWaypoint.x && initialPos.y === currentWaypoint.y) {
                        //console.PrintError("05 arrived at waypoint.");
                        cData.mSwtc = 4;
                        cData.pfc = 4;
                    }
                }
            }
            else {
                cData.mSwtc = 1;
                cData.pfc = -2;
                cData.dog.path = null;
            }*/


            /* var initialPosX = (cData.nData.nCoord.x);
             var initialPosY = (cData.nData.nCoord.y);
             var initialPos = { x: initialPosX, y: initialPosY };
 
             initialPos.x = Math.round(initialPos.x);
             initialPos.y = Math.round(initialPos.y);
 
             var currentWaypoint = { x: cData.cFWP.x, y: cData.cFWP.y };//{ x: initialPos.x, y: initialPos.y };
             currentWaypoint.x = Math.round(currentWaypoint.x);
             currentWaypoint.y = Math.round(currentWaypoint.y);
 
             if (initialPos.x === currentWaypoint.x && initialPos.y === currentWaypoint.y) {
                 cData.mSwtc = 4;
                 cData.pfc = 4;
             }
             else {
                 cData.mSwtc = 1;
                 cData.pfc = -2;
             }*/



            /*var currentWaypointSeekerPos = { x: cData.dog.path[0].worldPosition.x, y: cData.dog.path[0].worldPosition.y };//{ x: initialPos.x, y: initialPos.y };
            currentWaypointSeekerPos.x = Math.round(currentWaypointSeekerPos.x);
            currentWaypointSeekerPos.y = Math.round(currentWaypointSeekerPos.y);
       
       
            var lastCurrentWaypoint = { x: cData.cLFWP.x, y: cData.cLFWP.y };//{ x: cData.ltp.x, y: cData.ltp.y };
            lastCurrentWaypoint.x = Math.round(lastCurrentWaypoint.x);
            lastCurrentWaypoint.y = Math.round(lastCurrentWaypoint.y);
       
       
            var dronePosRounded = { x: cData.nData.nCoord.x, y: cData.nData.nCoord.y };//{ x: cData.ltp.x, y: cData.ltp.y };
            dronePosRounded.x = Math.round(dronePosRounded.x);
            dronePosRounded.y = Math.round(dronePosRounded.y);
            */
            /*var currentWaypointSeekerPos = { x: cData.dog.path[0].worldPosition.x, y: cData.dog.path[0].worldPosition.y };//{ x: initialPos.x, y: initialPos.y };
            currentWaypointSeekerPos.x = Math.round(currentWaypointSeekerPos.x);
            currentWaypointSeekerPos.y = Math.round(currentWaypointSeekerPos.y);
        
        
            var lastCurrentWaypoint = { x: cData.cLFWP.x, y: cData.cLFWP.y };//{ x: cData.ltp.x, y: cData.ltp.y };
            lastCurrentWaypoint.x = Math.round(lastCurrentWaypoint.x);
            lastCurrentWaypoint.y = Math.round(lastCurrentWaypoint.y);
        
        
            var dronePosRounded = { x: cData.nData.nCoord.x, y: cData.nData.nCoord.y };//{ x: cData.ltp.x, y: cData.ltp.y };
            dronePosRounded.x = Math.round(dronePosRounded.x);
            dronePosRounded.y = Math.round(dronePosRounded.y);
            */




            //---//////console.PrintError("***END OF PATHFIND***");
            //cData.mSwtc = 50;


            //move together =>
            /*if (cData.stopS == 1) {
                if (cData.stopSC >= cData.stopSCM) {
                    cData.stopS = 0;
                    cData.stopSC = 0;
                }
                cData.stopSC++;
            }
            cData.cLFWP = cData.cFWP;*/


            //move together <=
        }


        return cData;
    }
}







/*var distToNodeTwo = SC_Utilities.GetDistance(cData.dog.path[1].worldPosition, cData.nData.nCoord);
var dirToWaypointXTwo = ((cData.dog.path[1].worldPosition.x) - (cData.nData.nCoord.x)) / distToNodeTwo;
var dirToWaypointYTwo = ((cData.dog.path[1].worldPosition.y) - (cData.nData.nCoord.y)) / distToNodeTwo;
var newOffsetWaypointPosXTwo = (cData.dog.path[1].worldPosition.x + (dirToWaypointXTwo * offSetDistTwo));
var newOffsetWaypointPosYTwo = (cData.dog.path[1].worldPosition.y + (dirToWaypointYTwo * offSetDistTwo));
var newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationXTwo = (cData.dog.path[1].worldPosition.x + (dirToWaypointXTwo * offSetDistTwoTwo));
var newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationYTwo = (cData.dog.path[1].worldPosition.y + (dirToWaypointYTwo * offSetDistTwoTwo));
var newOffsetWaypointPos = { x: newOffsetWaypointPosX, y: newOffsetWaypointPosY };
var rotationDOTRL = SC_Utilities.Dot(dirToWaypointY, -dirToWaypointX, -cData.nData.nForward.x, -cData.nData.nForward.y);
var rotationDOTFB = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nForward.x, cData.nData.nForward.y);
var rotationDOTVELOTWO = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nVelo.x, cData.nData.nVelo.y);

var distToNodeTWO = SC_Utilities.GetDistance(cData.dog.path[1].worldPosition, cData.nData.nCoord);*/





/*var currentWaypoint = { x: cData.cFWP.x, y: cData.cFWP.y };//{ x: initialPos.x, y: initialPos.y };
currentWaypoint.x = Math.round(currentWaypoint.x);
currentWaypoint.y = Math.round(currentWaypoint.y);


var currentWaypointSeekerPos = { x: cData.dog.path[0].worldPosition.x, y: cData.dog.path[0].worldPosition.y };//{ x: initialPos.x, y: initialPos.y };
currentWaypointSeekerPos.x = Math.round(currentWaypointSeekerPos.x);
currentWaypointSeekerPos.y = Math.round(currentWaypointSeekerPos.y);


var lastCurrentWaypoint = { x: cData.cLFWP.x, y: cData.cLFWP.y };//{ x: cData.ltp.x, y: cData.ltp.y };
lastCurrentWaypoint.x = Math.round(lastCurrentWaypoint.x);
lastCurrentWaypoint.y = Math.round(lastCurrentWaypoint.y);


var dronePosRounded = { x: cData.nData.nCoord.x, y: cData.nData.nCoord.y };//{ x: cData.ltp.x, y: cData.ltp.y };
dronePosRounded.x = Math.round(dronePosRounded.x);
dronePosRounded.y = Math.round(dronePosRounded.y);



var swtchFastCancel = -1;

if (currentWaypointSeekerPos.x === currentWaypoint.x && currentWaypointSeekerPos.y === currentWaypoint.y && dronePosRounded.x === currentWaypoint.x && dronePosRounded.y === currentWaypoint.y ||
    currentWaypointSeekerPos.x === currentWaypoint.x && currentWaypointSeekerPos.y === currentWaypoint.y && dronePosRounded.x === currentWaypointSeekerPos.x && dronePosRounded.y === currentWaypointSeekerPos.y)
{
    cData.mSwtc = 1;
    cData.pfc = -2;
    //////npc.InstantStop(cData.objt.nid);
    //    ////npc.InstantStop(cData.objt.nid);
    cData.dog.path.shift();
    cData.dog.path = null;
    //npc.Unstick(cData.objt.nid);
}*/




/*if (cData.lip.x === cData.ltp.x && cData.lip.y === cData.ltp.y && boolWalker == 0) //cData.lip == last initial position
{
    //---//////console.PrintError("### the initial position is the same as the target position. this pathfind was started with missing logic prior to this function call, for this scenario to not even arrive to this point maybe... checking. SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2.js");
    ////---//////console.PrintError("00 target waypoint is invalid. the formation of the drone has to be modified. The drone can keep knowledge of the formation point but offsetting the pathfinding backwards to find a walkable tile and using that should be my next step to make this work. Also, when that will be done, i need to make sure that if the drone is 1 tile diagonally or linearly next to a station, the pathfind might not have a valid openset and that might be fixed in the script SC_PathFindInitGrid_cc_2 or inside of SC_PathFind_cc_2.js");
}*/


                        //cData.cLFWP = cData.cFWP;
                        //cData.cLFWPD = cData.cFWPD;




/*var swtchFastCancel = -1;

if (currentWaypointSeekerPos.x === currentWaypoint.x && currentWaypointSeekerPos.y === currentWaypoint.y ||
    currentWaypointSeekerPos.x === lastCurrentWaypoint.x && currentWaypointSeekerPos.y === lastCurrentWaypoint.y) //original minimumDistance //but it bugs at some point... why? let's search
{
    cData.mSwtc = 1;
    cData.pfc = -2;
    //////npc.InstantStop(cData.objt.nid);
    //    ////npc.InstantStop(cData.objt.nid);
    cData.dog.path.shift();
    cData.dog.path = null;
    //npc.Unstick(cData.objt.nid);
}*/






                        //if (swtchFastCancel != 1) {


/*if (cData.pfc == 2) {
    //CURRENT
    //CURRENT
    //during the pathfind at mSwtc == 3, openset came out to be null but the cData.log (list of grids) isn't null. Let's try and salvage a path to the (possibly null) target waypoint
    //---//////console.PrintError("during the pathfind at mSwtc == 3, openset came out to be null but the cData.log (list of grids) isn't null. Let's try and salvage a path to the (possibly null) target waypoint");
    //CURRENT
    //CURRENT
    //since openset was null and that there is a list of grids (cData.log) available, then only the last iteration (grid) of the whole List of Grids has the null openset, which means the path array's last indexes probably have unwalkable tiles. so we loop to remove them and if the path isn't
    //null after those unwalkable tiles are removed, then the drone will have to move there. more testing to come.

    //this needs a switch i believe.
    for (var i = 0; i < cData.dog.path.length; i++) {
        if (cData.dog.path[i].boolWalk == 1) //walkable.
        {
            foundNewTargetWaypoint = 1;
            break;
        }
        else //unwalkable.
        {
            cData.dog.path.splice(0, 1);
        }
    }
}*/




/*}
else
{
    cData.mSwtc = 1;
    cData.pfc = -2;
   // cData.dog.path = null;
    //////npc.InstantStop(cData.objt.nid);
}*/


/*if (rotationDOTFB >= 0 && rotationDOTFB < 0.99) //looking towards waypoint.
{
    npc.FaceCoord(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY);
    //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
}

else if (rotationDOTFB >= 0.99) //rotationDOTFB < 0 && rotationDOTFB > -0.99
{
    if (npc.IsStickToPoint(cData.objt.nid) || npc.IsStickToObject(cData.objt.nid)) {
        //npc.Unstick(cData.objt.nid);
    }
    npc.GoForward(cData.objt.nid);
}
else if (rotationDOTFB < 0 && rotationDOTFB > -0.99) //rotationDOTFB < 0 && rotationDOTFB > -0.99
{
    npc.FaceCoord(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY);
    //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
}
else if (rotationDOTFB > -0.99) //rotationDOTFB < 0 && rotationDOTFB > -0.99
{
    if (npc.IsStickToPoint(cData.objt.nid) || npc.IsStickToObject(cData.objt.nid)) {
        //npc.Unstick(cData.objt.nid);
    }
    npc.GoForward(cData.objt.nid);
}*/







    //cData.mSwtc = 1;
    //cData.mSwtc = 3;
    //forceUseRetracePathSwtc = 1;

/*if (cData.log != null)
{
    if (cData.log.length > 0)
    {
        //---//////console.PrintError("@@@ 11 openset null - list of grids !null - try and make a path out of it.");


        var currentWaypoint = { x: cData.cFWP.x, y: cData.cFWP.y };
        currentWaypoint.x = Math.round(currentWaypoint.x);
        currentWaypoint.y = Math.round(currentWaypoint.y);

        var lastCurrentWaypoint = { x: cData.cLFWP.x, y: cData.cLFWP.y };
        lastCurrentWaypoint.x = Math.round(lastCurrentWaypoint.x);
        lastCurrentWaypoint.y = Math.round(lastCurrentWaypoint.y);




        /*if (currentWaypoint.x === lastCurrentWaypoint.x && currentWaypoint.y === lastCurrentWaypoint.y && cData.cFWPD >= minimumDistance) //original minimumDistance //but it bugs at some point... why? let's search
        {
            cData.mSwtc = 1;
            cData.pfc = -2;
        }
        else
        {
            cData.mSwtc = 1;
            cData.pfc = -2;
        }



        //cData.mSwtc = 5;
        //cData.pfc = 2;


        forceUseRetracePathSwtc = 1;
        cData.mSwtc = 3;
        cData.pfc = 2;


        //cData.mSwtc = 5;
        //cData.pfc = 2;

        //cData.mSwtc = 1;
        //cData.pfc = -2;



    }
    else {
        //---//////console.PrintError("@010###NULL### ");
    }
}
else {
    //---//////console.PrintError("@011###NULL### ");
}*/


    //cData.mSwtc = 1;
    //cData.pfc = -2;






/*//---//////console.PrintError("sSwtc == 3  SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2.js");


//---//////console.PrintError("cData.dog.path.length > 0");

if (cData.cFWPD > 1) //if current formation waypoint distance > 1
{
    if (currentWaypoint.x != lastCurrentWaypoint.x && currentWaypoint.y != lastCurrentWaypoint.y) // if current last formation waypoint != current formation waypoint
    {
        ////---//////console.PrintError("sfwpp 0");
        cData.mSwtc = 1;
        cData.sSwtc = 3;
        //if (cData.stationTiles == null)
        //{
        //    cData.mSwtc = 1;
        //}
        //else
        //{
        //    cData.mSwtc = 2;
        //}
    }
    else {



        ////---//////console.PrintError("sfwpp 1");
        //---//////console.PrintError("reached waypoint 00");
        //cData.mSwtc = 1;
        //cData.sSwtc = 2;
        //cData.pfc = 1;


        cData.mSwtc = 1;
        cData.pfc = -2;


        //cData.mSwtc = 1;
        //cData.sSwtc = 3;
    }

    //cData.mSwtc = 2;
}
else //if current formation waypoint distance < 1
{
    //---//////console.PrintError("reached waypoint 1");
}*/





















/*if (cData.objt.bidSwtch == 1)
{
    cData.mSwtc = 1;
    //cData.sSwtc = 3;
}*/













/*var currentWaypoint = { x: cData.cFWP.x, y: cData.cFWP.y };//{ x: initialPos.x, y: initialPos.y };
currentWaypoint.x = Math.round(currentWaypoint.x);
currentWaypoint.y = Math.round(currentWaypoint.y);


var currentWaypointSeekerPos = { x: cData.dog.path[0].worldPosition.x, y: cData.dog.path[0].worldPosition.y };//{ x: initialPos.x, y: initialPos.y };
currentWaypointSeekerPos.x = Math.round(currentWaypointSeekerPos.x);
currentWaypointSeekerPos.y = Math.round(currentWaypointSeekerPos.y);


var lastCurrentWaypoint = { x: cData.cLFWP.x, y: cData.cLFWP.y };//{ x: cData.ltp.x, y: cData.ltp.y };
lastCurrentWaypoint.x = Math.round(lastCurrentWaypoint.x);
lastCurrentWaypoint.y = Math.round(lastCurrentWaypoint.y);

/*if (cData.lip.x === cData.ltp.x && cData.lip.y === cData.ltp.y && boolWalker == 0) //cData.lip == last initial position
{
    //---//////console.PrintError("### the initial position is the same as the target position. this pathfind was started with missing logic prior to this function call, for this scenario to not even arrive to this point maybe... checking. SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2.js");
    ////---//////console.PrintError("00 target waypoint is invalid. the formation of the drone has to be modified. The drone can keep knowledge of the formation point but offsetting the pathfinding backwards to find a walkable tile and using that should be my next step to make this work. Also, when that will be done, i need to make sure that if the drone is 1 tile diagonally or linearly next to a station, the pathfind might not have a valid openset and that might be fixed in the script SC_PathFindInitGrid_cc_2 or inside of SC_PathFind_cc_2.js");
}*/


    //cData.cLFWP = cData.cFWP;
    //cData.cLFWPD = cData.cFWPD;

/*var swtchFastCancel = -1;

if (currentWaypointSeekerPos.x === currentWaypoint.x && currentWaypointSeekerPos.y === currentWaypoint.y ||
    currentWaypointSeekerPos.x === lastCurrentWaypoint.x && currentWaypointSeekerPos.y === lastCurrentWaypoint.y) //original minimumDistance //but it bugs at some point... why? let's search
{
    cData.mSwtc = 1;
    cData.pfc = -2;

        ////npc.InstantStop(cData.objt.nid);


    swtchFastCancel = 1;
}*/



/*if (currentWaypoint.x === lastCurrentWaypoint.x && currentWaypoint.y === lastCurrentWaypoint.y && cData.cFWPD >= minimumDistance) //original minimumDistance //but it bugs at some point... why? let's search
{
    cData.mSwtc = 1;
    cData.pfc = -2;
}
else
{
    cData.mSwtc = 1;
    cData.pfc = -2;
}*/








    //var distToNode = SC_Utilities.GetDistance(cData.dog.path[0].worldPosition, cData.nData.nCoord);



/*if (cData.dog.path[1] != null) {
        var offSetDist = minimumStickDistance + additionalDistanceToAdd;
        var offSetDistTwo = minimumStickDistance + additionalDistanceToAdd;

        cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
        cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);

        var distToNode = SC_Utilities.GetDistance(cData.dog.path[1].worldPosition, cData.nData.nCoord);

        var dirToWaypointX = ((cData.dog.path[1].worldPosition.x) - (cData.nData.nCoord.x)) / distToNode;
        var dirToWaypointY = ((cData.dog.path[1].worldPosition.y) - (cData.nData.nCoord.y)) / distToNode;

        var newOffsetWaypointPosX = (cData.dog.path[1].worldPosition.x + (dirToWaypointX * offSetDist));
        var newOffsetWaypointPosY = (cData.dog.path[1].worldPosition.y + (dirToWaypointY * offSetDist));

        var newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX = (cData.dog.path[1].worldPosition.x + (dirToWaypointX * offSetDistTwo));
        var newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY = (cData.dog.path[1].worldPosition.y + (dirToWaypointY * offSetDistTwo));

        var newOffsetWaypointPos = { x: newOffsetWaypointPosX, y: newOffsetWaypointPosY };

        var rotationDOTRL = SC_Utilities.Dot(dirToWaypointY, -dirToWaypointX, -cData.nData.nForward.x, -cData.nData.nForward.y);
        var rotationDOTFB = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nForward.x, cData.nData.nForward.y);
        var rotationDOTVELOTWO = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nVelo.x, cData.nData.nVelo.y);

        //var distToNodeTwo = SC_Utilities.GetDistance(newOffsetWaypointPos, cData.nData.nCoord);
        //var currentSeekerPos = cData.dog.path[1].worldPosition;
}*/
























/*
if (!isNaN(rotationDOTVELO))
{
    //npc.FaceCoord(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY);

    if (distToNode > minimumDistance)
    {
        if (rotationDOTVELO >= 0.99)
        {
            ////npc.Unstick(cData.objt.nid);
            npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
        }
        else
        {
      
            if (cData.dog.path[1] != null)
            {
                var offSetDistTwo = minimumStickDistance + additionalDistanceToAdd;
                var offSetDistTwoTwo = minimumStickDistance + additionalDistanceToAdd;

                //cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
                //cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);

                var distToNodeTwo = SC_Utilities.GetDistance(cData.dog.path[1].worldPosition, cData.nData.nCoord);

                var dirToWaypointXTwo = ((cData.dog.path[1].worldPosition.x) - (cData.nData.nCoord.x)) / distToNodeTwo;
                var dirToWaypointYTwo = ((cData.dog.path[1].worldPosition.y) - (cData.nData.nCoord.y)) / distToNodeTwo;

                var newOffsetWaypointPosXTwo = (cData.dog.path[1].worldPosition.x + (dirToWaypointXTwo * offSetDistTwo));

                var newOffsetWaypointPosYTwo = (cData.dog.path[1].worldPosition.y + (dirToWaypointYTwo * offSetDistTwo));

                var newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationXTwo = (cData.dog.path[1].worldPosition.x + (dirToWaypointXTwo * offSetDistTwoTwo));
                var newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationYTwo = (cData.dog.path[1].worldPosition.y + (dirToWaypointYTwo * offSetDistTwoTwo));

                var newOffsetWaypointPosTwo = { x: newOffsetWaypointPosXTwo, y: newOffsetWaypointPosYTwo };

                var rotationDOTRLTwo = SC_Utilities.Dot(dirToWaypointYTwo, -dirToWaypointXTwo, -cData.nData.nForward.x, -cData.nData.nForward.y);
                var rotationDOTFBTwo = SC_Utilities.Dot(dirToWaypointXTwo, dirToWaypointYTwo, cData.nData.nForward.x, cData.nData.nForward.y);
                var rotationDOTVELOTwo = SC_Utilities.Dot(dirToWaypointXTwo, dirToWaypointYTwo, cData.nData.nVelo.x, cData.nData.nVelo.y);


                if (rotationDOTVELO >= 0.99 && rotationDOTVELOTwo >= 0.99) //before i was using rotationDOTVELOTwo > 0.99
                {
                    //npc.Unstick(cData.objt.nid);
                    npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationXTwo, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationYTwo, 0);
                }
                else {
                    ////npc.InstantStop(cData.objt.nid);
                    npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
                    ////npc.InstantStop(cData.objt.nid);
                }
            }
            else //if cData.dog.path[1] == null , then there is about a 100% chance that cData.dog.path.length == 1 since a check for cData.dog.path.length > 0 or cData.dog.path != null was already checked.
            {
                ////npc.InstantStop(cData.objt.nid);
                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
                ////npc.InstantStop(cData.objt.nid);

            }







        }
    }
    else //if (!isNaN(rotationDOTVELO)) //if (distToNode <= minimumDistance)
    {





        /*
        var hasChosenPathIndexOne = -1;
        //checking if path[1] is != null and choosing to unstick to path[0] and stick to path[1] if the velocity towards it is
        ////if (distToNode <= minimumDistance) ------- // if (cData.dog.path[1] != null)
        if (cData.dog.path[1] != null)
        {
            var offSetDistTwo = minimumStickDistance + additionalDistanceToAdd;
            var offSetDistTwoTwo = minimumStickDistance + additionalDistanceToAdd;

            //cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
            //cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);

            var distToNodeTwo = SC_Utilities.GetDistance(cData.dog.path[1].worldPosition, cData.nData.nCoord);

            var dirToWaypointXTwo = ((cData.dog.path[1].worldPosition.x) - (cData.nData.nCoord.x)) / distToNodeTwo;
            var dirToWaypointYTwo = ((cData.dog.path[1].worldPosition.y) - (cData.nData.nCoord.y)) / distToNodeTwo;

            var newOffsetWaypointPosXTwo = (cData.dog.path[1].worldPosition.x + (dirToWaypointXTwo * offSetDistTwo));

            var newOffsetWaypointPosYTwo = (cData.dog.path[1].worldPosition.y + (dirToWaypointYTwo * offSetDistTwo));

            var newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationXTwo = (cData.dog.path[1].worldPosition.x + (dirToWaypointXTwo * offSetDistTwoTwo));
            var newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationYTwo = (cData.dog.path[1].worldPosition.y + (dirToWaypointYTwo * offSetDistTwoTwo));

            var newOffsetWaypointPosTwo = { x: newOffsetWaypointPosXTwo, y: newOffsetWaypointPosYTwo };

            var rotationDOTRLTwo = SC_Utilities.Dot(dirToWaypointYTwo, -dirToWaypointXTwo, -cData.nData.nForward.x, -cData.nData.nForward.y);
            var rotationDOTFBTwo = SC_Utilities.Dot(dirToWaypointXTwo, dirToWaypointYTwo, cData.nData.nForward.x, cData.nData.nForward.y);
            var rotationDOTVELOTwo = SC_Utilities.Dot(dirToWaypointXTwo, dirToWaypointYTwo, cData.nData.nVelo.x, cData.nData.nVelo.y);


            if (rotationDOTVELO >= 0.99 && rotationDOTVELOTwo >= 0.99) //before i was using rotationDOTVELOTwo > 0.99
            {
                cData.dog.path.shift();
                //npc.Unstick(cData.objt.nid);
                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationXTwo, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationYTwo, 0);
                hasChosenPathIndexOne = 1;
            }
            else {
                ////npc.InstantStop(cData.objt.nid);
                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
                ////npc.InstantStop(cData.objt.nid);
            }
        }
        else //if cData.dog.path[1] == null , then there is about a 100% chance that cData.dog.path.length == 1 since a check for cData.dog.path.length > 0 or cData.dog.path != null was already checked.
        {
            cData.mSwtc = 1;
            cData.pfc = -2;
            //////npc.InstantStop(cData.objt.nid);
            ////npc.InstantStop(cData.objt.nid);
            cData.dog.path.shift();
            cData.dog.path = null;
            //npc.Unstick(cData.objt.nid);

        }



        //if (distToNode <= minimumDistance) && // if (cData.dog.path.length == 1)
        if (cData.dog.path.length == 1) {

            var nCoordx = Math.round(cData.nData.nCoord.x);
            var nCoordY = Math.round(cData.nData.nCoord.y);

            var nCoord = { x: nCoordx, y: nCoordY };
            //cData.mSwtc = 1;
            //cData.pfc = -2;

            //////npc.InstantStop(cData.objt.nid);
            //////npc.InstantStop(cData.objt.nid);

            //cData.dog.path.shift();
            //cData.dog.path = null;
            ////npc.Unstick(cData.objt.nid);

            var currentWaypoint = { x: cData.cFWP.x, y: cData.cFWP.y };//{ x: initialPos.x, y: initialPos.y };
            currentWaypoint.x = Math.round(currentWaypoint.x);
            currentWaypoint.y = Math.round(currentWaypoint.y);

            var currentWaypointSeekerPos = { x: cData.dog.path[0].worldPosition.x, y: cData.dog.path[0].worldPosition.y };//{ x: initialPos.x, y: initialPos.y };
            currentWaypointSeekerPos.x = Math.round(currentWaypointSeekerPos.x);
            currentWaypointSeekerPos.y = Math.round(currentWaypointSeekerPos.y);

            var lastCurrentWaypoint = { x: cData.cLFWP.x, y: cData.cLFWP.y };//{ x: cData.ltp.x, y: cData.ltp.y };
            lastCurrentWaypoint.x = Math.round(lastCurrentWaypoint.x);
            lastCurrentWaypoint.y = Math.round(lastCurrentWaypoint.y);

            //currentWaypointSeekerPos.x === lastCurrentWaypoint.x && currentWaypointSeekerPos.y === lastCurrentWaypoint.y

            if (nCoord.x != currentWaypoint.x || nCoord.y != currentWaypoint.y) //original minimumDistance //but it bugs at some point... why? let's search
            {
                cData.mSwtc = 1;
                cData.pfc = -2;
                //////npc.InstantStop(cData.objt.nid);
                ////npc.InstantStop(cData.objt.nid);
                cData.dog.path.shift();
                cData.dog.path = null;
                //npc.Unstick(cData.objt.nid);
            }
            else if (nCoord.x === currentWaypoint.x && nCoord.y === currentWaypoint.y) //TODOTODOTODOTODOTODOTODO SET NPC IN STANDBY AS IT HAS REACHED THE WAYPOINT. TODO
            {
                cData.mSwtc = 1;
                cData.pfc = -2;
                //////npc.InstantStop(cData.objt.nid);
                ////npc.InstantStop(cData.objt.nid);
                cData.dog.path.shift();
                cData.dog.path = null;
                //npc.Unstick(cData.objt.nid);

            }
            else if (nCoord.x === currentWaypointSeekerPos.x && nCoord.y === currentWaypointSeekerPos.y) //the current path[0] is reached but the length of path is 1 so restart the pathfind.
            {
                cData.mSwtc = 1;
                cData.pfc = -2;
                //////npc.InstantStop(cData.objt.nid);
                ////npc.InstantStop(cData.objt.nid);
                cData.dog.path.shift();
                cData.dog.path = null;
                //npc.Unstick(cData.objt.nid);
            }
            else {
                cData.mSwtc = 1;
                cData.pfc = -2;
                //////npc.InstantStop(cData.objt.nid);
                ////npc.InstantStop(cData.objt.nid);
                cData.dog.path.shift();
                cData.dog.path = null;
                //npc.Unstick(cData.objt.nid);
            }



            if (currentWaypointSeekerPos.x === currentWaypoint.x && currentWaypointSeekerPos.y === currentWaypoint.y ||
                currentWaypointSeekerPos.x === lastCurrentWaypoint.x && currentWaypointSeekerPos.y === lastCurrentWaypoint.y) //original minimumDistance //but it bugs at some point... why? let's search
            {
                cData.mSwtc = 1;
                cData.pfc = -2;
                //////npc.InstantStop(cData.objt.nid);
                //    ////npc.InstantStop(cData.objt.nid);
                cData.dog.path.shift();
                cData.dog.path = null;
                //npc.Unstick(cData.objt.nid);
            }
        }
        else if (cData.dog.path.length > 0 && cData.dog.path.length != 1)//if (distToNode <= minimumDistance) && // if (cData.dog.path.length > 0)
        {
            var offSetDistTwo = minimumStickDistance + additionalDistanceToAdd;
            var offSetDistTwoTwo = minimumStickDistance + additionalDistanceToAdd;

            //cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
            //cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);

            var distToNodeTwo = SC_Utilities.GetDistance(cData.dog.path[1].worldPosition, cData.nData.nCoord);

            var dirToWaypointXTwo = ((cData.dog.path[1].worldPosition.x) - (cData.nData.nCoord.x)) / distToNodeTwo;
            var dirToWaypointYTwo = ((cData.dog.path[1].worldPosition.y) - (cData.nData.nCoord.y)) / distToNodeTwo;

            var newOffsetWaypointPosXTwo = (cData.dog.path[1].worldPosition.x + (dirToWaypointXTwo * offSetDistTwo));

            var newOffsetWaypointPosYTwo = (cData.dog.path[1].worldPosition.y + (dirToWaypointYTwo * offSetDistTwo));

            var newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationXTwo = (cData.dog.path[1].worldPosition.x + (dirToWaypointXTwo * offSetDistTwoTwo));
            var newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationYTwo = (cData.dog.path[1].worldPosition.y + (dirToWaypointYTwo * offSetDistTwoTwo));

            var newOffsetWaypointPosTwo = { x: newOffsetWaypointPosXTwo, y: newOffsetWaypointPosYTwo };

            var rotationDOTRLTwo = SC_Utilities.Dot(dirToWaypointYTwo, -dirToWaypointXTwo, -cData.nData.nForward.x, -cData.nData.nForward.y);
            var rotationDOTFBTwo = SC_Utilities.Dot(dirToWaypointXTwo, dirToWaypointYTwo, cData.nData.nForward.x, cData.nData.nForward.y);
            var rotationDOTVELOTwo = SC_Utilities.Dot(dirToWaypointXTwo, dirToWaypointYTwo, cData.nData.nVelo.x, cData.nData.nVelo.y);

            if (rotationDOTVELOTwo >= 0.99) {
                //cData.mSwtc = 1;
                //cData.pfc = -2;
                cData.dog.path.shift();
                //npc.Unstick(cData.objt.nid);
                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationXTwo, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationYTwo, 0);
            }
            else {
                //    ////npc.InstantStop(cData.objt.nid);
                ////npc.InstantStop(cData.objt.nid);
            }

            //cData.mSwtc = 1;
            //cData.pfc = -2;
            //////npc.InstantStop(cData.objt.nid);
            ////npc.InstantStop(cData.objt.nid);
            cData.dog.path.shift();
            //cData.dog.path = null;
            //npc.Unstick(cData.objt.nid);
        }
        else {
            cData.mSwtc = 1;
            cData.pfc = -2;
            //////npc.InstantStop(cData.objt.nid);
            //////npc.InstantStop(cData.objt.nid);
            cData.dog.path.shift();
            cData.dog.path = null;
            //npc.Unstick(cData.objt.nid);
        }









  
    }
}
else //if (isNaN(rotationDOTVELO))
{
    //if isNaN(rotationDOTVELO), then the drone isn't moving. i gotta use a stop mechanism otherwise sticktopoint is making the drone move anywhere while turning so stopping the drone when it's turning so that it doesn't leave unwalkable tiles is the best way to go because,
    //stick to point is much much faster at doing a rotation than RotLeft or RotRight and FaceCoord or FaceStickPoint. i gotta retest FaceStickPoint only though as i've tested RotLeft and RotRight and FaceCoord and the only way to make them fire more than once per frame is
    //to perma write them line after line in a script as they don't work inside of a loop. StickToPoint somehow does a fast rotation as if it was doing RotLeft/RotRight inside of a loop and it's much faster...

    if (currentWaypointSeekerPos.x === currentWaypoint.x && currentWaypointSeekerPos.y === currentWaypoint.y ||
        currentWaypointSeekerPos.x === lastCurrentWaypoint.x && currentWaypointSeekerPos.y === lastCurrentWaypoint.y) //original minimumDistance //but it bugs at some point... why? let's search
    {

        //////npc.InstantStop(cData.objt.nid); // if player is !stopped
        ////npc.InstantStop(cData.objt.nid); // if player is stopped

        cData.mSwtc = 1;
        cData.pfc = -2;
        cData.dog.path.shift();
        cData.dog.path = null;
        //npc.Unstick(cData.objt.nid);


    }



    if (rotationDOTFB > 0.99) {
        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
    }
    else {
        //    ////npc.InstantStop(cData.objt.nid);

        if (rotationDOTVELO < 0.99) {
            cData.stopSCM = 2;

            if (cData.stopS == 0) {
                if (currentWaypoint.x != lastCurrentWaypoint.x && currentWaypoint.y != lastCurrentWaypoint.y) {
                    //if (stationTiles == null) {
                    //    cData.mSwtc = 1;
                    //}
                    //else {
                    //    cData.mSwtc = 2;
                    //}

                    cData.mSwtc = 1;
                    cData.pfc = -2;

                    //    ////npc.InstantStop(cData.objt.nid);
                }

                //TOREADD
                //TOREADD
                //TOREADD
                //    ////npc.InstantStop(cData.objt.nid);
                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
                //TOREADD
                //TOREADD
                //TOREADD

                cData.stopS = 0;
            }
            //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
            ////npc.InstantStop(cData.objt.nid);
        }
        else {
            cData.stopSCM = 2;
            if (cData.stopS == 0) {
                if (currentWaypoint.x != lastCurrentWaypoint.x && currentWaypoint.y != lastCurrentWaypoint.y) {
                    cData.mSwtc = 1;
                    cData.pfc = -2;
                    //
                    //    ////npc.InstantStop(cData.objt.nid);
                }

                //TOREADD
                //TOREADD
                //TOREADD
                //    ////npc.InstantStop(cData.objt.nid);
                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
                //////npc.InstantStop(cData.objt.nid);
                //TOREADD
                //TOREADD
                //TOREADD


                cData.stopS = 0;
            }
            //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
            ////npc.InstantStop(cData.objt.nid);
        }
    }


}*/














/*if (cData.dog.path[1] != null)
{
    var offSetDistTwo = minimumStickDistance + additionalDistanceToAdd;
    var offSetDistTwoTwo = minimumStickDistance + additionalDistanceToAdd;

    //cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
    //cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);

    var distToNodeTwo = SC_Utilities.GetDistance(cData.dog.path[1].worldPosition, cData.nData.nCoord);

    var dirToWaypointXTwo = ((cData.dog.path[1].worldPosition.x) - (cData.nData.nCoord.x)) / distToNodeTwo;
    var dirToWaypointYTwo = ((cData.dog.path[1].worldPosition.y) - (cData.nData.nCoord.y)) / distToNodeTwo;

    var newOffsetWaypointPosXTwo = (cData.dog.path[1].worldPosition.x + (dirToWaypointXTwo * offSetDistTwo));
    var newOffsetWaypointPosYTwo = (cData.dog.path[1].worldPosition.y + (dirToWaypointYTwo * offSetDistTwo));

    var newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationXTwo = (cData.dog.path[1].worldPosition.x + (dirToWaypointXTwo * offSetDistTwoTwo));
    var newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationYTwo = (cData.dog.path[1].worldPosition.y + (dirToWaypointYTwo * offSetDistTwoTwo));

    var newOffsetWaypointPosTwo = { x: newOffsetWaypointPosXTwo, y: newOffsetWaypointPosYTwo };

    var rotationDOTRLTwo = SC_Utilities.Dot(dirToWaypointYTwo, -dirToWaypointXTwo, -cData.nData.nForward.x, -cData.nData.nForward.y);
    var rotationDOTFBTwo = SC_Utilities.Dot(dirToWaypointXTwo, dirToWaypointYTwo, cData.nData.nForward.x, cData.nData.nForward.y);
    var rotationDOTVELOTwo = SC_Utilities.Dot(dirToWaypointXTwo, dirToWaypointYTwo, cData.nData.nVelo.x, cData.nData.nVelo.y);

    if (rotationDOTVELOTwo >= 0.99) {
        //cData.mSwtc = 1;
        //cData.pfc = -2;
        cData.dog.path.shift();
        //npc.Unstick(cData.objt.nid);
        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
    }
    else {
        //    ////npc.InstantStop(cData.objt.nid);
        ////npc.InstantStop(cData.objt.nid);
    }
}*/




/*
if (cData.dog.path.length == 1) {
    //

}
else {

    var currentWaypoint = { x: cData.cFWP.x, y: cData.cFWP.y };//{ x: initialPos.x, y: initialPos.y };
    currentWaypoint.x = Math.round(currentWaypoint.x);
    currentWaypoint.y = Math.round(currentWaypoint.y);

    var currentWaypointSeekerPos = { x: cData.dog.path[0].worldPosition.x, y: cData.dog.path[0].worldPosition.y };//{ x: initialPos.x, y: initialPos.y };
    currentWaypointSeekerPos.x = Math.round(currentWaypointSeekerPos.x);
    currentWaypointSeekerPos.y = Math.round(currentWaypointSeekerPos.y);

    var lastCurrentWaypoint = { x: cData.cLFWP.x, y: cData.cLFWP.y };//{ x: cData.ltp.x, y: cData.ltp.y };
    lastCurrentWaypoint.x = Math.round(lastCurrentWaypoint.x);
    lastCurrentWaypoint.y = Math.round(lastCurrentWaypoint.y);

    //if (cData.lip.x === cData.ltp.x && cData.lip.y === cData.ltp.y && boolWalker == 0) //cData.lip == last initial position
    //{
    //    //---//////console.PrintError("### the initial position is the same as the target position. this pathfind was started with missing logic prior to this function call, for this scenario to not even arrive to this point maybe... checking. SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2.js");
    //    ////---//////console.PrintError("00 target waypoint is invalid. the formation of the drone has to be modified. The drone can keep knowledge of the formation point but offsetting the pathfinding backwards to find a walkable tile and using that should be my next step to make this work. Also, when that will be done, i need to make sure that if the drone is 1 tile diagonally or linearly next to a station, the pathfind might not have a valid openset and that might be fixed in the script SC_PathFindInitGrid_cc_2 or inside of SC_PathFind_cc_2.js");
    //}

    //cData.cLFWP = cData.cFWP;
    //cData.cLFWPD = cData.cFWPD;

    if (currentWaypointSeekerPos.x === currentWaypoint.x && currentWaypointSeekerPos.y === currentWaypoint.y ||
        currentWaypointSeekerPos.x === lastCurrentWaypoint.x && currentWaypointSeekerPos.y === lastCurrentWaypoint.y) //original minimumDistance //but it bugs at some point... why? let's search
    {
        cData.mSwtc = 1;
        cData.pfc = -2;
        // cData.dog.path = null;
        ////npc.InstantStop(cData.objt.nid);
        //    ////npc.InstantStop(cData.objt.nid);
        //cData.dog.path.shift();

    }
    //npc.Unstick(cData.objt.nid);
    cData.dog.path.shift();


}*/













    ////////console.PrintError("00 velo is !nan");

/*if (rotationDOTFB > 0.99) {
    npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationXTwo, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationYTwo, 0);
}
else {
    //    ////npc.InstantStop(cData.objt.nid);

    if (rotationDOTVELO < 0.99) {
        cData.stopSCM = 2;

        if (cData.stopS == 0) {
            if (currentWaypoint.x != lastCurrentWaypoint.x && currentWaypoint.y != lastCurrentWaypoint.y) {
                //if (stationTiles == null) {
                //    cData.mSwtc = 1;
                //}
                //else {
                //    cData.mSwtc = 2;
                //}

                cData.mSwtc = 1;
                cData.pfc = -2;

                //    ////npc.InstantStop(cData.objt.nid);
            }

            //TOREADD
            //TOREADD
            //TOREADD
            //    ////npc.InstantStop(cData.objt.nid);
            npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
            //TOREADD
            //TOREADD
            //TOREADD

            cData.stopS = 1;
        }
        //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
        ////npc.InstantStop(cData.objt.nid);
    }
    else {
        cData.stopSCM = 2;
        if (cData.stopS == 0) {
            if (currentWaypoint.x != lastCurrentWaypoint.x && currentWaypoint.y != lastCurrentWaypoint.y) {
                cData.mSwtc = 1;
                cData.pfc = -2;
                //
                //    ////npc.InstantStop(cData.objt.nid);
            }

            //TOREADD
            //TOREADD
            //TOREADD
            //    ////npc.InstantStop(cData.objt.nid);
            npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
            //////npc.InstantStop(cData.objt.nid);
            //TOREADD
            //TOREADD
            //TOREADD


            cData.stopS = 1;
        }
        //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
        ////npc.InstantStop(cData.objt.nid);
    }
}*/



/*if (cData.dog.path[1] != null) {

    var offSetDistTwo = minimumStickDistance + additionalDistanceToAdd;
    var offSetDistTwoTwo = minimumStickDistance + additionalDistanceToAdd;

    //cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
    //cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);

    var distToNodeTwo = SC_Utilities.GetDistance(cData.dog.path[1].worldPosition, cData.nData.nCoord);

    var dirToWaypointXTwo = ((cData.dog.path[1].worldPosition.x) - (cData.nData.nCoord.x)) / distToNodeTwo;
    var dirToWaypointYTwo = ((cData.dog.path[1].worldPosition.y) - (cData.nData.nCoord.y)) / distToNodeTwo;

    var newOffsetWaypointPosXTwo = (cData.dog.path[1].worldPosition.x + (dirToWaypointXTwo * offSetDistTwo));
    var newOffsetWaypointPosYTwo = (cData.dog.path[1].worldPosition.y + (dirToWaypointYTwo * offSetDistTwo));

    var newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationXTwo = (cData.dog.path[1].worldPosition.x + (dirToWaypointXTwo * offSetDistTwoTwo));
    var newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationYTwo = (cData.dog.path[1].worldPosition.y + (dirToWaypointYTwo * offSetDistTwoTwo));

    var newOffsetWaypointPosTwo = { x: newOffsetWaypointPosXTwo, y: newOffsetWaypointPosYTwo };

    var rotationDOTRLTwo = SC_Utilities.Dot(dirToWaypointYTwo, -dirToWaypointXTwo, -cData.nData.nForward.x, -cData.nData.nForward.y);
    var rotationDOTFBTwo = SC_Utilities.Dot(dirToWaypointXTwo, dirToWaypointYTwo, cData.nData.nForward.x, cData.nData.nForward.y);
    var rotationDOTVELOTwo = SC_Utilities.Dot(dirToWaypointXTwo, dirToWaypointYTwo, cData.nData.nVelo.x, cData.nData.nVelo.y);

    if (rotationDOTVELOTwo >= 0.99) {
        //cData.mSwtc = 1;
        //cData.pfc = -2;
        cData.dog.path.shift();
        //npc.Unstick(cData.objt.nid);
        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
    }
    else {
        //    ////npc.InstantStop(cData.objt.nid);
        ////npc.InstantStop(cData.objt.nid);
    }
}


////////console.PrintError("00 velo is nan");

if (rotationDOTFB > 0.99) {
    npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
}
else {
    //    ////npc.InstantStop(cData.objt.nid);
    if (rotationDOTVELO < 0.99) {
        cData.stopSCM = 2;
        if (cData.stopS == 0) {
            if (currentWaypoint.x != lastCurrentWaypoint.x && currentWaypoint.y != lastCurrentWaypoint.y) {
                cData.mSwtc = 1;
                cData.pfc = -2;
                //
                //    ////npc.InstantStop(cData.objt.nid);
            }

            //TOREADD
            //TOREADD
            //TOREADD
            //    ////npc.InstantStop(cData.objt.nid);
            npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
            //TOREADD
            //TOREADD
            //TOREADD

            cData.stopS = 1;
        }
        //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
        ////npc.InstantStop(cData.objt.nid);
    }
    else {
        cData.stopSCM = 2;
        if (cData.stopS == 0) {
            if (currentWaypoint.x != lastCurrentWaypoint.x && currentWaypoint.y != lastCurrentWaypoint.y) {
                cData.mSwtc = 1;
                cData.pfc = -2;
                //
                //    ////npc.InstantStop(cData.objt.nid);
            }

            //TOREADD
            //TOREADD
            //TOREADD
            //    ////npc.InstantStop(cData.objt.nid);
            npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
            //////npc.InstantStop(cData.objt.nid);
            //TOREADD
            //TOREADD
            //TOREADD


            cData.stopS = 1;
        }
        //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
        ////npc.InstantStop(cData.objt.nid);
    }
}*/





/*if (distToNode > minimumDistance)
{
    //npc.FaceCoord(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY);
    //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
    //---//////console.PrintError("000isNaN " + " distToNode: " + distToNode);
    //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosX, newOffsetWaypointPosY, 0);
    //npc.GoForward(cData.objt.nid);
}
else {
    //---//////console.PrintError("111isNaN " + " distToNode: " + distToNode);


    if (cData.dog.path[1] != null) {
        var offSetDistTwo = minimumStickDistance + additionalDistanceToAdd;
        var offSetDistTwoTwo = minimumStickDistance + additionalDistanceToAdd;

        //cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
        //cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);

        var distToNodeTwo = SC_Utilities.GetDistance(cData.dog.path[1].worldPosition, cData.nData.nCoord);

        var dirToWaypointXTwo = ((cData.dog.path[1].worldPosition.x) - (cData.nData.nCoord.x)) / distToNodeTwo;
        var dirToWaypointYTwo = ((cData.dog.path[1].worldPosition.y) - (cData.nData.nCoord.y)) / distToNodeTwo;

        var newOffsetWaypointPosXTwo = (cData.dog.path[1].worldPosition.x + (dirToWaypointXTwo * offSetDistTwo));

        var newOffsetWaypointPosYTwo = (cData.dog.path[1].worldPosition.y + (dirToWaypointYTwo * offSetDistTwo));

        var newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationXTwo = (cData.dog.path[1].worldPosition.x + (dirToWaypointXTwo * offSetDistTwoTwo));
        var newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationYTwo = (cData.dog.path[1].worldPosition.y + (dirToWaypointYTwo * offSetDistTwoTwo));

        var newOffsetWaypointPosTwo = { x: newOffsetWaypointPosXTwo, y: newOffsetWaypointPosYTwo };

        var rotationDOTRLTwo = SC_Utilities.Dot(dirToWaypointYTwo, -dirToWaypointXTwo, -cData.nData.nForward.x, -cData.nData.nForward.y);
        var rotationDOTFBTwo = SC_Utilities.Dot(dirToWaypointXTwo, dirToWaypointYTwo, cData.nData.nForward.x, cData.nData.nForward.y);
        var rotationDOTVELOTwo = SC_Utilities.Dot(dirToWaypointXTwo, dirToWaypointYTwo, cData.nData.nVelo.x, cData.nData.nVelo.y);

        if (rotationDOTVELOTwo >= 0.99) {
            //cData.mSwtc = 1;
            //cData.pfc = -2;
            cData.dog.path.shift();
            //npc.Unstick(cData.objt.nid);
            npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
        }
        else {
            //    ////npc.InstantStop(cData.objt.nid);
            ////npc.InstantStop(cData.objt.nid);
        }
    }
    else {

        if (cData.dog.path.length == 1) {
            //
            ////npc.InstantStop(cData.objt.nid);
            cData.mSwtc = 1;
            cData.pfc = -2;
            cData.dog.path.shift();
            // cData.dog.path = null;
            //npc.Unstick(cData.objt.nid);
        }
        else {

            var currentWaypoint = { x: cData.cFWP.x, y: cData.cFWP.y };//{ x: initialPos.x, y: initialPos.y };
            currentWaypoint.x = Math.round(currentWaypoint.x);
            currentWaypoint.y = Math.round(currentWaypoint.y);

            var currentWaypointSeekerPos = { x: cData.dog.path[0].worldPosition.x, y: cData.dog.path[0].worldPosition.y };//{ x: initialPos.x, y: initialPos.y };
            currentWaypointSeekerPos.x = Math.round(currentWaypointSeekerPos.x);
            currentWaypointSeekerPos.y = Math.round(currentWaypointSeekerPos.y);

            var lastCurrentWaypoint = { x: cData.cLFWP.x, y: cData.cLFWP.y };//{ x: cData.ltp.x, y: cData.ltp.y };
            lastCurrentWaypoint.x = Math.round(lastCurrentWaypoint.x);
            lastCurrentWaypoint.y = Math.round(lastCurrentWaypoint.y);

            //if (cData.lip.x === cData.ltp.x && cData.lip.y === cData.ltp.y && boolWalker == 0) //cData.lip == last initial position
            //{
            //    //---//////console.PrintError("### the initial position is the same as the target position. this pathfind was started with missing logic prior to this function call, for this scenario to not even arrive to this point maybe... checking. SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2.js");
            //    ////---//////console.PrintError("00 target waypoint is invalid. the formation of the drone has to be modified. The drone can keep knowledge of the formation point but offsetting the pathfinding backwards to find a walkable tile and using that should be my next step to make this work. Also, when that will be done, i need to make sure that if the drone is 1 tile diagonally or linearly next to a station, the pathfind might not have a valid openset and that might be fixed in the script SC_PathFindInitGrid_cc_2 or inside of SC_PathFind_cc_2.js");
            //}

            //cData.cLFWP = cData.cFWP;
            //cData.cLFWPD = cData.cFWPD;

            if (currentWaypointSeekerPos.x === currentWaypoint.x && currentWaypointSeekerPos.y === currentWaypoint.y ||
                currentWaypointSeekerPos.x === lastCurrentWaypoint.x && currentWaypointSeekerPos.y === lastCurrentWaypoint.y) //original minimumDistance //but it bugs at some point... why? let's search
            {
                cData.mSwtc = 1;
                cData.pfc = -2;
                // cData.dog.path = null;
                ////npc.InstantStop(cData.objt.nid);
                //    ////npc.InstantStop(cData.objt.nid);
                //cData.dog.path.shift();

            }
            //npc.Unstick(cData.objt.nid);
            cData.dog.path.shift();


        }
    }
}*/



    //npc.GoBackward(cData.objt.nid);
    //    ////npc.InstantStop(cData.objt.nid);
    //currentSet.splice(currentSet.length - 1, 1);


/*
if (isNaN(rotationDOTVELO)) {
    npc.StickToPoint(cData.objt.nid, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX, newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY, 0);
}*/


















/*
if (currentWaypoint.x === lastCurrentWaypoint.x && currentWaypoint.y === lastCurrentWaypoint.y && cData.cFWPD < minimumDistance) //original minimumDistance //but it bugs at some point... why? let's search
{
    //---//////console.PrintError("the current waypoint and last waypoint are the same. pathfind allowed to start depending on tweaked parameters.");
    //REASON 1 FOR NOT STARTING => //---//////console.PrintError("the pathfind restarted because it has a different path to the same waypoint");
    //REASON 2 FOR NOT STARTING => //---//////console.PrintError("the pathfind restarted because the drone is right next to the waypoint and the start node is the same as the target node.");

    var pathData = SC_PathFindInitGrid_Data_cc_2_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, 0, cData, null);
    //cData.dog.gridOffset = pathData.gridOffset;
    cData.dog.grid = pathData.grid;
    cData.dog.openSet = pathData.openSet;
    //cData.dog.closedSet = pathData.closedSet; //NOT USED ANYMORE. DOUBLE CHECK BEFORE REMOVING COMPLETELY
    //cData.dog.path = pathData.path;
    var someDataOfGrid = { grid: cData.dog.grid }; //, index: cData.log.length
    cData.log.push(someDataOfGrid);
    cData.mSwtc = 3;



    //cData.mSwtc = 1;
    //return cData;
}
else {
    //---//////console.PrintError("the current waypoint and last waypoint are NOT the same. pathfind allowed to start. although it might have been restarted because of mSwtc == 1 but there are no tweaked parameters.");


    var currentWaypoint = { x: cData.cFWP.x, y: cData.cFWP.y };
    currentWaypoint.x = Math.round(currentWaypoint.x);
    currentWaypoint.y = Math.round(currentWaypoint.y);

    var lastCurrentWaypoint = { x: cData.cLFWP.x, y: cData.cLFWP.y };
    lastCurrentWaypoint.x = Math.round(lastCurrentWaypoint.x);
    lastCurrentWaypoint.y = Math.round(lastCurrentWaypoint.y);

    if (currentWaypoint.x === lastCurrentWaypoint.x && currentWaypoint.y === lastCurrentWaypoint.y && cData.cFWPD >= minimumDistance) //original minimumDistance //but it bugs at some point... why? let's search
    {

        //tpr == the pathfind restarted

        ////---//////console.PrintError("the pathfind restarted because it is the same waypoint OR");
        //---//////console.PrintError("tpr // because it has a different path to the same waypoint and the last waypoint and current waypoint are UNWALKABLE and the target tile itself is unwalkable");
        //---//////console.PrintError("tpr // because the drone is right next to the waypoint and the start node is the same as the target node OR");
        //---//////console.PrintError("tpr // because the target tile was unwalkable and the drone pathfind is being reinitiated still on doing the same thing over an over, to restart it's pathfind to an unwalkable target tile.");







        //CHECKING IF cData.dog.path.length exists and if we can salvage a path to target from that... to developp
        //CHECKING IF cData.dog.path.length exists and if we can salvage a path to target from that... to developp
        //CHECKING IF cData.dog.path.length exists and if we can salvage a path to target from that... to developp
        //NOTES 2021-MARS-19
        //NOTES 2021-MARS-19
        //NOTES 2021-MARS-19
        //boolWalker = 1; //walkable
        //boolWalker = 0;  //unwalkable
        //RESTART PATHFIND BUT MAKE THE OFFSET TO THE FORMATION WAYPOINT 1 DIAGONAL TILE LESS, AND CHECK IF THE NEIGHBOOR TILES OF THOSE HAVE ANY WALKABLE TILES ALREADY DISCOVERED AND INSIDE OF cData.dog.path and reinitate a retracePath
        //to see if the new formation waypoint is a valid path, but a shorter one since the target tile was unwalkable before the pathfind got to this step here.


        //if the target tile is unwalkable, loop the cData.dog.path backwards and check the walkability and the moment a tile is walkable, discard whatever is higher in index of cData.dog.path and make the cData.dog.path.length-1 node the target node.
        //if the target tile is unwalkable, loop the cData.dog.path backwards and check the walkability and the moment a tile is walkable, discard whatever is higher in index of cData.dog.path and make the cData.dog.path.length-1 node the target node.
        //if the target tile is unwalkable, loop the cData.dog.path backwards and check the walkability and the moment a tile is walkable, discard whatever is higher in index of cData.dog.path and make the cData.dog.path.length-1 node the target node.
        /*var foundNewTargetWaypoint = -1;

        for (var i = 0; i < cData.dog.path.length; i++) {
            if (cData.dog.path[i].boolWalk == 1) // if the tile
            {
                foundNewTargetWaypoint = 1;
                break;
            }
            else {
                cData.dog.path.splice(0, 1);
            }
        }
        //if the target tile is unwalkable, loop the cData.dog.path backwards and check the walkability and the moment a tile is walkable, discard whatever is higher in index of cData.dog.path and make the cData.dog.path.length-1 node the target node.
        //if the target tile is unwalkable, loop the cData.dog.path backwards and check the walkability and the moment a tile is walkable, discard whatever is higher in index of cData.dog.path and make the cData.dog.path.length-1 node the target node.
        //if the target tile is unwalkable, loop the cData.dog.path backwards and check the walkability and the moment a tile is walkable, discard whatever is higher in index of cData.dog.path and make the cData.dog.path.length-1 node the target node.



        if (cData.dog != null) {

            //++
            //---//////console.PrintError("00#NULL###");
            //++



            if (cData.dog.path != null) {


                //++
                //---//////console.PrintError("11#NULL###");
                //++


                if (cData.dog.path.length > 0)
                {
                    //---//////console.PrintError("22#NULL###");
                    if (cData.dog.path[0] != null) {
                        //---//////console.PrintError("33#NULL###");
                    }
                    else //cData.dog.path == null
                    {
                        //---//////console.PrintError("44#NULL###");
                    }
                }
                else //cData.dog.path == null
                {

                    //the path is empty.
                    //++
                    //---//////console.PrintError("55#NULL###");
                    //++

                    //cData.mSwtc = 1;
                    //return cData;

                    ////---//////console.PrintError("npcGetGridData");





                    /*var pathData = SC_PathFindInitGrid_Data_cc_2_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, 0, cData, null);
                    //cData.dog.gridOffset = pathData.gridOffset;
                    cData.dog.grid = pathData.grid;
                    cData.dog.openSet = pathData.openSet;
                    //cData.dog.closedSet = pathData.closedSet; //NOT USED ANYMORE. DOUBLE CHECK BEFORE REMOVING COMPLETELY
                    //cData.dog.path = pathData.path;
                    var someDataOfGrid = { grid: cData.dog.grid }; //, index: cData.log.length
                    cData.log.push(someDataOfGrid);
                    cData.mSwtc = 3;








                    cData.mSwtc = 1;
                    cData.pfc = -2;

                }
            }
            else // cData.dog == null
            {
                //---//////console.PrintError("66###NULL###");
            }
        }
        else // cData.dog == null
        {
            //---//////console.PrintError("77###NULL###");
        }





        /*if (cData.dog != null)
        {
            if (cData.dog.path != null)
            {
                if (cData.dog.path.length > 0)
                {
                    if (cData.dog.path[0] != null)
                    {
                        if (cData.dog.path[cData.dog.path.length - 1].boolWalk == 0)
                        {
                            //---//////console.PrintError("44###NULL###");

                            ////---//////console.PrintError("the pathfind restarted because the target tile was unwalkable and the drone pathfind is being reinitiated still on doing the same thing over an over, to restart it's pathfind to an unwalkable target tile. Otherwise cData.dog.path currentWaypoint and lastWaypoint wouldn't be the same, so if the path of the lastwaypoint and currentWaypoint are also the same is left to be determine with a couple more lines of code. Also, the waypoint formation is more than minimumDistance VE units diagonally further away from the drone position. currentWaypoint.x === lastCurrentWaypoint.x && currentWaypoint.y === lastCurrentWaypoint.y && cData.cFWPD > minimumDistance");

                            if (cData.dog != null)
                            {
                                if (cData.ldog != null)
                                {
                                    var currentWaypointLocation = { x: cData.dog.path[cData.dog.path.length - 1].x, y: cData.dog.path[cData.dog.path.length - 1].y };
                                    currentWaypointLocation.x = Math.round(currentWaypointLocation.x);
                                    currentWaypointLocation.y = Math.round(currentWaypointLocation.y);

                                    var lastCurrentTargetWaypointlocation = { x: cData.ldog.path[cData.ldog.path.length - 1].x, y: cData.ldog.path[cData.ldog.path.length - 1].y };
                                    lastCurrentTargetWaypointlocation.x = Math.round(lastCurrentTargetWaypointlocation.x);
                                    lastCurrentTargetWaypointlocation.y = Math.round(lastCurrentTargetWaypointlocation.y);

                                    if (currentWaypointLocation.x === lastCurrentTargetWaypointlocation.x && currentWaypointLocation.y === lastCurrentTargetWaypointlocation.y) //the last path target waypoint (1 tile to bullzeye) is the same as the current path target waypoint (1 tile to bullzeye)
                                    {
                                        //---//////console.PrintError("same waypoint");
                                    }
                                    else //the last path target waypoint (1 tile to bullzeye) is ! the same as the current path target waypoint (1 tile to bullzeye) //
                                    {
                                        //---//////console.PrintError("!same waypoint");
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        //---//////console.PrintError("33###NULL###");
                    }
                }
                else //if (cData.dog.path.length <= 0)
                {
                    //inside of pathfind start switch =>if (cData.mSwtc == 1)<=

                    ////---//////console.PrintError("22###NULL###");

                    var pathData = SC_PathFindInitGrid_Data_cc_2_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, 0, cData, null);


                    if (pathData.fSwtch == 1) {
                        //---//////console.PrintError("00fSwtch == 1 // SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2.js");
                    }
                    else if (pathData.fSwtch == 0) {
                        //---//////console.PrintError("00fSwtch == 0 // SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2.js");
                    }


                    //cData.dog.gridOffset = pathData.gridOffset;
                    cData.dog.grid = pathData.grid;
                    cData.dog.openSet = pathData.openSet;

                    //cData.dog.closedSet = pathData.closedSet; //NOT USED ANYMORE. DOUBLE CHECK BEFORE REMOVING COMPLETELY

                    //cData.dog.path = pathData.path;

                    var someDataOfGrid = { grid: cData.dog.grid }; //, index: cData.log.length
                    cData.log.push(someDataOfGrid);
                    cData.mSwtc = 3;
                }
            }
            else //cData.dog.path == null
            {
                //---//////console.PrintError("11#NULL###");
            }
        }
        else // cData.dog == null
        {
            //---//////console.PrintError("00###NULL###");
        }
        //NOTES 2021-MARS-19
        //NOTES 2021-MARS-19
        //NOTES 2021-MARS-19
        //CHECKING IF cData.dog.path.length exists and if we can salvage a path to target from that... to developp
        //CHECKING IF cData.dog.path.length exists and if we can salvage a path to target from that... to developp
        //CHECKING IF cData.dog.path.length exists and if we can salvage a path to target from that... to developp













































  





    }
    else //currentWaypoint.x === lastCurrentWaypoint.x && currentWaypoint.y === lastCurrentWaypoint.y && cData.cFWPD > minimumDistance
    {





        if (currentWaypoint.x === lastCurrentWaypoint.x && currentWaypoint.y === lastCurrentWaypoint.y)
        {
            //---//////console.PrintError("!!!currentWaypoint.x === lastCurrentWaypoint.x && currentWaypoint.y === lastCurrentWaypoint.y");
        }
        else
        {
            //---//////console.PrintError("!!!currentWaypoint.x != lastCurrentWaypoint.x || currentWaypoint.y != lastCurrentWaypoint.y");
        }


        if (cData.cFWPD <= minimumDistance) //original minimumDistance
        {
            //---//////console.PrintError("!!!cData.cFWPD <= minimumDistance");
            ////---//////console.PrintError("npcGetGridData");
            var pathData = SC_PathFindInitGrid_Data_cc_2_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, 0, cData, null);
            //cData.dog.gridOffset = pathData.gridOffset;
            cData.dog.grid = pathData.grid;
            cData.dog.openSet = pathData.openSet;

            //cData.dog.closedSet = pathData.closedSet; //NOT USED ANYMORE. DOUBLE CHECK BEFORE REMOVING COMPLETELY

            //cData.dog.path = pathData.path;

            var someDataOfGrid = { grid: cData.dog.grid }; //, index: cData.log.length
            cData.log.push(someDataOfGrid);
            cData.mSwtc = 3;


            //TO READD
            //TO READD
            //TO READD
            /*if (cData.dog.path != null)
            {
                if (cData.dog.path.length > 0)
                {
                    //---//////console.PrintError("%%%cData.mSwtc == " + cData.mSwtc + " %%% ");
                    cData.mSwtc = 5;
                    cData.pfc = 2; //
                    ////---//////console.PrintError("!!!cData.cFWPD > minimumDistance");
                }
                else {
                    //---//////console.PrintError("cData.dog.path.length <= 0");
                }

            }
            else
            {
                //---//////console.PrintError("cData.dog.path == null");
                ////---//////console.PrintError("npcGetGridData");
                var pathData = SC_PathFindInitGrid_Data_cc_2_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, 0, cData, null);
                //cData.dog.gridOffset = pathData.gridOffset;
                cData.dog.grid = pathData.grid;
                cData.dog.openSet = pathData.openSet;
                //cData.dog.closedSet = pathData.closedSet; //NOT USED ANYMORE. DOUBLE CHECK BEFORE REMOVING COMPLETELY
                //cData.dog.path = pathData.path;
                var someDataOfGrid = { grid: cData.dog.grid }; //, index: cData.log.length
                cData.log.push(someDataOfGrid);
                cData.mSwtc = 3;
            }
             //TO READD
            //TO READD
            //TO READD

            //---//////console.PrintError("cData.dog.path == null");
            ////---//////console.PrintError("npcGetGridData");
            var pathData = SC_PathFindInitGrid_Data_cc_2_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, 0, cData, null);
            //cData.dog.gridOffset = pathData.gridOffset;
            cData.dog.grid = pathData.grid;
            cData.dog.openSet = pathData.openSet;
            //cData.dog.closedSet = pathData.closedSet; //NOT USED ANYMORE. DOUBLE CHECK BEFORE REMOVING COMPLETELY
            //cData.dog.path = pathData.path;
            var someDataOfGrid = { grid: cData.dog.grid }; //, index: cData.log.length
            cData.log.push(someDataOfGrid);
            cData.mSwtc = 3;




        }

        if (cData.cFWPD > minimumDistance) //original minimumDistance
        {
            //---//////console.PrintError("!!!cData.cFWPD > minimumDistance");




            if (cData.dog.path != null)
            {
                if (cData.dog.path.length > 0) {
                    //---//////console.PrintError("%%%cData.mSwtc == " + cData.mSwtc + " %%% ");
                    cData.mSwtc = 5;
                    cData.pfc = 2;
                    ////---//////console.PrintError("!!!cData.cFWPD > minimumDistance");
                }
                else {





                    ////---//////console.PrintError("###cData.dog.path.length <= 0");


                    /*var pathData = SC_PathFindInitGrid_Data_cc_2_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, 0, cData, null);
                    //cData.dog.gridOffset = pathData.gridOffset;
                    cData.dog.grid = pathData.grid;
                    cData.dog.openSet = pathData.openSet;
                    //cData.dog.closedSet = pathData.closedSet; //NOT USED ANYMORE. DOUBLE CHECK BEFORE REMOVING COMPLETELY
                    //cData.dog.path = pathData.path;
                    var someDataOfGrid = { grid: cData.dog.grid }; //, index: cData.log.length
                    cData.log.push(someDataOfGrid);
                    cData.mSwtc = 3;
                    var pathData = SC_PathFindInitGrid_Data_cc_2_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, 0, cData, null);
                    //cData.dog.gridOffset = pathData.gridOffset;
                    cData.dog.grid = pathData.grid;
                    cData.dog.openSet = pathData.openSet;
                    //cData.dog.closedSet = pathData.closedSet; //NOT USED ANYMORE. DOUBLE CHECK BEFORE REMOVING COMPLETELY
                    //cData.dog.path = pathData.path;
                    var someDataOfGrid = { grid: cData.dog.grid }; //, index: cData.log.length
                    cData.log.push(someDataOfGrid);
                    cData.mSwtc = 3;


                    //cData.mSwtc = 1;
                    //cData.pfc = -2;

                }
            }
            else
            {
                //---//////console.PrintError("cData.dog.path == null");

                ////---//////console.PrintError("npcGetGridData");
                var pathData = SC_PathFindInitGrid_Data_cc_2_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, 0, cData, null);
                //cData.dog.gridOffset = pathData.gridOffset;
                cData.dog.grid = pathData.grid;
                cData.dog.openSet = pathData.openSet;
                //cData.dog.closedSet = pathData.closedSet; //NOT USED ANYMORE. DOUBLE CHECK BEFORE REMOVING COMPLETELY
                //cData.dog.path = pathData.path;
                var someDataOfGrid = { grid: cData.dog.grid }; //, index: cData.log.length
                cData.log.push(someDataOfGrid);
                cData.mSwtc = 3;
                cData.pfc = 2;



            }








            //IF IT ARRIVED HERE, THE WHOLE GRID WHERE THE TARGET TILE IS, IS INVALID. SO RESTARTING THE PATHFIND MAKES THE PATHFIND PERPETUALLY TRY TO FIND THE TARGETGRID/TARGETTILES... //CAN BE USEFULL MAYBE IN A SIDE QUEST, WHERE THE TARGET TILE /TARGET GRID IS INVALID DUE TO THE TILES BEING
            //UNWALKABLE BY THRASH/GARBAGE/ASTEROIDS/SHIPSALVAGINGPARTS/SPECIALOBJECTS/CRATES/
            ////---//////console.PrintError("npcGetGridData");
            var pathData = SC_PathFindInitGrid_Data_cc_2_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, 0, cData, null);
            if (pathData.fSwtch == 1)
            {
                //---//////console.PrintError("11fSwtch == 1 // SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2.js");
            }
            else if (pathData.fSwtch == 0)
            {
                //---//////console.PrintError("11fSwtch == 0 // SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2.js");
            }
            //cData.dog.gridOffset = pathData.gridOffset;
            cData.dog.grid = pathData.grid;
            cData.dog.openSet = pathData.openSet;
            //cData.dog.closedSet = pathData.closedSet; //NOT USED ANYMORE. DOUBLE CHECK BEFORE REMOVING COMPLETELY
            //cData.dog.path = pathData.path;
            var someDataOfGrid = { grid: cData.dog.grid }; //, index: cData.log.length
            cData.log.push(someDataOfGrid);
            cData.mSwtc = 3;
            //IF IT ARRIVED HERE, THE WHOLE GRID WHERE THE TARGET TILE IS, IS INVALID. SO RESTARTING THE PATHFIND MAKES THE PATHFIND PERPETUALLY TRY TO FIND THE TARGETGRID/TARGETTILES... //CAN BE USEFULL MAYBE IN A SIDE QUEST, WHERE THE TARGET TILE /TARGET GRID IS INVALID DUE TO THE TILES BEING
            //UNWALKABLE BY THRASH/GARBAGE/ASTEROIDS/SHIPSALVAGINGPARTS/SPECIALOBJECTS/CRATES/







        }



    }




    ////---//////console.PrintError("npcGetGridData");
    //var pathData = SC_PathFindInitGrid_Data_cc_2_merge.npcGetGridData(gridWorldSize, nodeRadius, 0, cData, null);
    //cData.dog.gridOffset = pathData.gridOffset;
    //cData.dog.grid = pathData.grid;
    //cData.dog.openSet = pathData.openSet;
    //cData.dog.closedSet = pathData.closedSet;
    //cData.dog.path = pathData.path;
    //var someDataOfGrid = { grid: cData.dog.grid }; //, index: cData.log.length
    //cData.log.push(someDataOfGrid);
    //cData.mSwtc = 3;
}*/












/*////---//////console.PrintError("npcGetGridData");
var pathData = SC_PathFindInitGrid_Data_cc_2_merge.npcGetGridDataMerged(gridWorldSize, nodeRadius, 0, cData, null);
//cData.dog.gridOffset = pathData.gridOffset;
cData.dog.grid = pathData.grid;
cData.dog.openSet = pathData.openSet;
cData.dog.closedSet = pathData.closedSet;
//cData.dog.path = pathData.path;

var someDataOfGrid = { grid: cData.dog.grid }; //, index: cData.log.length
cData.log.push(someDataOfGrid);
cData.mSwtc = 3;*/























/*if (distToNodeTwo > 0.25)
{

}
else
{
    //---//////console.PrintError("***PATHFIND 11***");
    ////npc.Unstick(cData.objt.nid);
    cData.dog.path.shift();
    //////npc.InstantStop(cData.objt.nid);
    //    ////npc.InstantStop(cData.objt.nid);
    ////npc.InstantStop(cData.objt.nid);
}*/

/*
if (distToNodeTwo > 0.25)
{
    //---//////console.PrintError("***PATHFIND 00***");
    //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);

    if (rotationDOTFB >= 0.99) {
        ////npc.Unstick(cData.objt.nid);
        //npc.GoForward(cData.objt.nid);
        //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
    }
    else {
        if (rotationDOTVELO < 0.99) {
            //    ////npc.InstantStop(cData.objt.nid);
            //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
        }
        else {
            //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
        }
    }
}
else {
    //---//////console.PrintError("***PATHFIND 11***");
    ////npc.Unstick(cData.objt.nid);
    cData.dog.path.shift();
    //////npc.InstantStop(cData.objt.nid);
    //    ////npc.InstantStop(cData.objt.nid);
    ////npc.InstantStop(cData.objt.nid);
}*/


/* cData.dog.path[0].worldPosition.x = Math.round(cData.dog.path[0].worldPosition.x);
 cData.dog.path[0].worldPosition.y = Math.round(cData.dog.path[0].worldPosition.y);

 cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
 cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);

 var distToNode = SC_Utilities.GetDistance(cData.dog.path[0].worldPosition, cData.nData.nCoord);

 var dirToWaypointX = ((cData.dog.path[0].worldPosition.x) - (cData.nData.nCoord.x)) / distToNode;
 var dirToWaypointY = ((cData.dog.path[0].worldPosition.y) - (cData.nData.nCoord.y)) / distToNode;

 var newOffsetWaypointPosX = (cData.dog.path[0].worldPosition.x + (dirToWaypointX * 6));
 var newOffsetWaypointPosY = (cData.dog.path[0].worldPosition.y + (dirToWaypointY * 6));

 var newOffsetWaypointPos = { x: newOffsetWaypointPosX, y: newOffsetWaypointPosY };

 var rotationDOTRL = SC_Utilities.Dot(dirToWaypointY, -dirToWaypointX, -cData.nData.nForward.x, -cData.nData.nForward.y);
 var rotationDOTFB = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nForward.x, cData.nData.nForward.y);
 var rotationDOTVELO = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nVelo.x, cData.nData.nVelo.y);

 if (distToNode > 0.1) {
     if (rotationDOTFB >= 0.99) {
         ////npc.Unstick(cData.objt.nid);
         //npc.GoForward(cData.objt.nid);
         npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
     }
     else {
         if (rotationDOTVELO < 0.97) {
             cData.stopSCM = 2;
             if (cData.stopS == 0) {
                 if (currentWaypoint.x != lastCurrentWaypoint.x && currentWaypoint.y != lastCurrentWaypoint.y) {
                     /*if (cData.stationTiles == null) {
                         cData.mSwtc = 1;
                     }
                     else {
                         cData.mSwtc = 2;
                     }
                 }
                     ////npc.InstantStop(cData.objt.nid);

                 //////npc.InstantStop(cData.objt.nid);

                 cData.stopS = 1;
             }
             npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
         }
         else {
             cData.stopSCM = 2;
             if (cData.stopS == 0) {
                 if (currentWaypoint.x != lastCurrentWaypoint.x && currentWaypoint.y != lastCurrentWaypoint.y) {
                     /*if (cData.stationTiles == null) {
                         cData.mSwtc = 1;
                     }
                     else {
                         cData.mSwtc = 2;
                     }
                 }
                     ////npc.InstantStop(cData.objt.nid);
                 //////npc.InstantStop(cData.objt.nid);

                 cData.stopS = 1;
             }
             npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
         }

         /*if (rotationDOTRL >= 0.05) {

             //npc.RotRight(cData.objt.nid);
             //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
         }
         else if (rotationDOTRL <= -0.05) {
             if (cData.nData.eSpeed < 0.00001) {
                 npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
             }
             //npc.RotLeft(cData.objt.nid);
             //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
         }
     }
 }
 else {
     if (Math.round(cData.nData.nCoord.x) == Math.round(cData.dog.path[0].worldPosition.x) && Math.round(cData.nData.nCoord.y) == Math.round(cData.dog.path[0].worldPosition.y)) {
         //npc.Unstick(cData.objt.nid);
             ////npc.InstantStop(cData.objt.nid);
         //////npc.InstantStop(cData.objt.nid);

         cData.dog = null;
         cData.mSwtc = 6;
     }
     else {
         //npc.Unstick(cData.objt.nid);
         cData.dog.path.shift();
         ////---//////console.PrintError("03");
         if (cData.stopS == 0) {
             if (currentWaypoint.x != lastCurrentWaypoint.x && currentWaypoint.y != lastCurrentWaypoint.y) {
                 /*if (cData.stationTiles == null) {
                     cData.mSwtc = 1;
                 }
                 else {
                     cData.mSwtc = 2;
                 }
             }
                 ////npc.InstantStop(cData.objt.nid);
             //////npc.InstantStop(cData.objt.nid);

             cData.stopS = 1;
         }
     }
 }*/









/*cData.nData.nCoord.x = Math.round(cData.nData.nCoord.x);
cData.nData.nCoord.y = Math.round(cData.nData.nCoord.y);

var distToNode = SC_Utilities.GetDistance(cData.dog.path[0].worldPosition, cData.nData.nCoord);

var dirToWaypointX = ((cData.dog.path[0].worldPosition.x) - (cData.nData.nCoord.x)) / distToNode;
var dirToWaypointY = ((cData.dog.path[0].worldPosition.y) - (cData.nData.nCoord.y)) / distToNode;


var newOffsetWaypointPosX = cData.nData.nCoord.x + (dirToWaypointX * 5.5);
var newOffsetWaypointPosY = cData.nData.nCoord.y + (dirToWaypointY * 5.5);

//var newOffsetWaypointPosX = (cData.dog.path[0].worldPosition.x + (dirToWaypointX * 5.5));
//var newOffsetWaypointPosY = (cData.dog.path[0].worldPosition.y + (dirToWaypointY * 5.5));

var newOffsetWaypointPos = { x: newOffsetWaypointPosX, y: newOffsetWaypointPosY };

var rotationDOTRL = SC_Utilities.Dot(dirToWaypointY, -dirToWaypointX, -cData.nData.nForward.x, -cData.nData.nForward.y);
var rotationDOTFB = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nForward.x, cData.nData.nForward.y);
var rotationDOTVELO = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData.nData.nVelo.x, cData.nData.nVelo.y);

var distToNodeTwo = SC_Utilities.GetDistance(newOffsetWaypointPos, cData.nData.nCoord);

//---//////console.PrintError("rotationDOTFB " + rotationDOTFB + " rotationDOTRL " + rotationDOTRL + " rotationDOTVELO " + rotationDOTVELO  + " SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2.js");




if (rotationDOTRL >= 0.1) {
    ////npc.InstantStop(cData.objt.nid);
    npc.RotRight(cData.objt.nid)
    npc.FaceCoord(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y);
    npc.FaceStickPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y);

    //    ////npc.InstantStop(cData.objt.nid);
    //////npc.InstantStop(cData.objt.nid);
}
else if (rotationDOTRL <= -0.1) {
    ////npc.InstantStop(cData.objt.nid);
    npc.RotLeft(cData.objt.nid)
    npc.FaceCoord(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y);
    npc.FaceStickPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y);

    //    ////npc.InstantStop(cData.objt.nid);
    //////npc.InstantStop(cData.objt.nid);
}
else
{

    if (distToNodeTwo > 0.25)
    {
        npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);

        //---//////console.PrintError("distToNodeTwo " + distToNodeTwo + " SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2.js");
        //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);

        if (rotationDOTFB >= 0.99)
        {
            //---//////console.PrintError("rotationDOTFB >= 0.99 " + " SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2.js");
            ////npc.Unstick(cData.objt.nid);
            //npc.GoForward(cData.objt.nid);
            //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
            npc.FaceCoord(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y);
            //npc.GoForward(npcID);
            npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y);
        }
        else {

            if (rotationDOTVELO < 0.99)
            {
                //---//////console.PrintError("rotationDOTFB < 0.99 " + " SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2.js");
                //    ////npc.InstantStop(cData.objt.nid);
                //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);


                ////npc.InstantStop(cData.objt.nid);
                //npc.FaceCoord(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y);
                npc.FaceStickPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y);
            }
            else {
                //---//////console.PrintError("rotationDOTFB >= 0.99 " + " SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2.js");
                //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y, 0);
                //npc.GoForward(cData.objt.nid);


                npc.FaceCoord(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y);
                //npc.GoForward(npcID);
                npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y);
            }
        }
    }
    else {
        //---//////console.PrintError("cData.dog.path.shift(); distToNodeTwo <= 0.1 " + " SC_AI_Drone_Combat_cc_Rout_For_SFWPP_2.js");
        ////npc.Unstick(cData.objt.nid);
        ////npc.Unstick(cData.objt.nid);
        cData.dog.path.shift();
        //////npc.InstantStop(cData.objt.nid);
        //    ////npc.InstantStop(cData.objt.nid);
        ////npc.InstantStop(cData.objt.nid);
    }

    //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y);

    /*if (rotationDOTFB >= 0 && rotationDOTFB < 0.99) {

        ////npc.Unstick(cData.objt.nid);
        //---//////console.PrintError("rotationDOTFB >= 0 && rotationDOTFB < 0.99");
        var newOffsetWaypointPosX = (cData.nData.nCoord.x + (dirToWaypointX * 5.5));
        var newOffsetWaypointPosY = (cData.nData.nCoord.y + (dirToWaypointY * 5.5));

        newOffsetWaypointPos = { x: newOffsetWaypointPosX, y: newOffsetWaypointPosY };

        //npc.FaceCoord(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y);
        //////npc.InstantStop(cData.objt.nid);
        //npc.FaceStickPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y);
        //npc.FaceStickPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y);
        //    ////npc.InstantStop(cData.objt.nid);
    }
    else //looking opposite to the waypoint
    {

        //---//////console.PrintError("rotationDOTFB < 0");
        var newOffsetWaypointPosX = (cData.nData.nCoord.x + (dirToWaypointX * 5.5));
        var newOffsetWaypointPosY = (cData.nData.nCoord.y + (dirToWaypointY * 5.5));

        newOffsetWaypointPos = { x: newOffsetWaypointPosX, y: newOffsetWaypointPosY };


        //npc.FaceCoord(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y);
        npc.GoForward(cData.objt.nid);
        //npc.StickToPoint(cData.objt.nid, newOffsetWaypointPos.x, newOffsetWaypointPos.y);
    }
}*/