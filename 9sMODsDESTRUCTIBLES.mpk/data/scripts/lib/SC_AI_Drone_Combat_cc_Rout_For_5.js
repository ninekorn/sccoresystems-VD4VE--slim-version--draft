using(console);
using(npc);
using(ship);
using(storage);

include(SC_Utilities.js);

include(SC_AI_Drone_Get_nData.js);
include(SC_AI_Drone_Get_pData.js);

include(SC_AI_Drone_Combat_cc_Rout_For_FWP_5.js);
include(SC_AI_Drone_Combat_cc_Rout_Com_5.js);
include(SC_AI_Drone_Combat_cc_Reset_Speed_5.js);
include(SC_AI_Drone_Combat_cc_Friendlies_5.js);

include(SC_Perceptron.js);
include(SC_AI_Drone_Combat_cc_Rout_For_Prot_5.js);
include(SC_AI_Drone_Combat_cc_Rout_For_SFWPP_5.js);

var nData = null;
var pData = null;
var hpNPC;
var hpPlayer;

var cData = [];
var cDat = null;

var ceData = [];
var ceDat = null;

var lcData = [];
var lcDat = null;

var lceData = [];
var lceDat = null;

var distToWaypoint = 0;
var lastdistToWaypoint = 0;
var lastdistToWaypointAvgDistCovered = 0;

var currentFormationWaypoint = { x: null, y: null };
var initOnce = 1;
var initArrayOfFriendlies = 1;
var arrayOfFriendlies = [];
var dataToReturn = null;
var globIndex;
var resetSwitch = 0;


var arrayOfDataRL = [];
var arrayOfErrorDataRL = [];

var arrayOfDataFR = [];
var arrayOfErrorDataFR = [];

var lengthOfArray = 100; //200 original// 100

var finalDotR = 0;
var initialDotGoal = 0;
var trainingLR = [];
var trainingFR = [];

var lengthOfTrainingArray = 100; //500 original//250
var count = 0;
var lastWaypointPos;
var lastFrameDot = 0;


var lastDistanceToWaypoint = [];
var lastDistanceToWaypointAdjacentFB = [];
var lastDistanceToWaypointoppositeRL = [];

var distToWaypoint = [];
var adjacentFB = [];
var oppositeRLDistance = [];

var lastVeloNPC = [];
var lastSpeedNPC = [];
var lastSpeedPlayer = [];

var speedNPC = [];
var speedPlayer = [];
var frontPushCounter = [];
var backPushCounter = [];
var leftPushCounter = [];
var rightPushCounter = [];


var lastFrontPushCounter = [];
var lastBackPushCounter = [];
var lastLeftPushCounter = [];
var lastRightPushCounter = [];

var lastSomeOtherRLDOT = [];
var lastSomeOtherFBDOT = [];
var lastRLDOTStrafe = [];

var someOtherRLDOT = [];
var someOtherFBDOT = [];

var pushToTheLEFT = [];
var pushToTheRIGHT = [];
var pushBACK = [];
var pushFRONT = [];

var formationSwitches = [];
var selectedTarget = [];

var mainSwitches = [];

var lastdistToWaypoint = 0;
var lastadjacentFB = 0;

var _currentWaypointPathDirection = { x: 0, y: 0 };

var minimumDistWaypoint = 0.15;// 1.55 //0.15

var framesToCoverDistanceAtSpeedRL = 0;
var framesToCoverDistanceAtSpeedBREAKERRL = 0;

var framesToCoverDistanceAtSpeedRLMax = 1;
var framesToCoverDistanceAtSpeedBREAKERRLMax = 1;
var framesToCoverDistanceAtSpeedBREAKERFBMax = 1;
var maxFrameToTargetRLDot = 1;
var oppositeRLDistanceMax = 1;

var adjacentFBMax = 1;
var framesToCoverDistanceAtSpeedFBINVERTER = 1;
var framesToCoverDistanceAtSpeedFBFrontMAX = 1;
var framesToCoverDistanceAtSpeedFBINVERTER = 1;
var framesToCoverDistanceAtSpeedFBBACKMAX = 1;




var rotationDirectionDOTVELO = 0;
var pathFindSwtch = 0;



//0 => normal pathfind.
//1 => perceptron pathfind. in the works.






var activateThisFramePathfindCounter = 0;
var activateThisFramePathfindCounterMax = 1;

var activateThisFramePathfindCounterTWO = 0;
var activateThisFramePathfindCounterTWOMax = 1;




var SC_AI_Drone_Combat_cc_Rout_For_5 =
{
    AIRoutineInit: function (currentObjective, addFriendSwitch) {
        arrayOfFriendlies = SC_AI_Drone_Combat_cc_Friendlies_5.AIAddFriendlies(currentObjective, addFriendSwitch);

        if (cData.length >= currentObjective.droneIndex && ceData.length >= currentObjective.droneIndex && cData[currentObjective.droneIndex] != null && ceData[currentObjective.droneIndex] != null)
        {



            hpNPC = ship.GetCurrentValue(currentObjective.nid, "structure");

            if (hpNPC > 0) {
                nData = SC_AI_Drone_Get_nData.npcGetSelfNPCData(currentObjective, 1);
            }
            else {
                return;
            }
            hpPlayer = ship.GetCurrentValue(currentObjective.pid, "structure");

            if (hpPlayer > 0) {
                pData = SC_AI_Drone_Get_pData.npcGetPlayerData(currentObjective, 0);
            }
            //else
            //{
            //  pData = lastArrayOfPlayerData[currentObjective.droneIndex];
            //}

            if (currentObjective.speedSwitch == 1) {
                if (nData != null && pData != null) {
                    /*if (cData[currentObjective.droneIndex] == null || ceData[currentObjective.droneIndex] == null)
                    {        
                        cData = [];
                        ceData = [];
                        addToArray(currentObjective, globIndex, 0);
                        return;
                    }*/
                    cData[currentObjective.droneIndex].nData = nData;
                    cData[currentObjective.droneIndex].pData = pData;
                    cData[currentObjective.droneIndex].objt = currentObjective;

                    ceData[currentObjective.droneIndex].nData = nData;
                    ceData[currentObjective.droneIndex].pData = pData;
                    ceData[currentObjective.droneIndex].pLData = pData;
                    ceData[currentObjective.droneIndex].objt = currentObjective;
                    ceData[currentObjective.droneIndex].arrF = arrayOfFriendlies;

                    currentFormationWaypoint = SC_AI_Drone_Combat_cc_Rout_For_FWP_5.npcGWFP(currentObjective, 5);
                    distToWaypoint = SC_Utilities.GetDistance(currentFormationWaypoint, ceData[currentObjective.droneIndex].nData.nCoord);

                    cData[currentObjective.droneIndex].cFWP = currentFormationWaypoint;
                    cData[currentObjective.droneIndex].cFWPD = distToWaypoint;

                    //if (cData[currentObjective.droneIndex].swtchForPathfind == 1 && activateThisFramePathfindCounter > activateThisFramePathfindCounterMax || cData[currentObjective.droneIndex].swtchForPathfind == 0 && activateThisFramePathfindCounter > activateThisFramePathfindCounterMax) // || cData[currentObjective.droneIndex].swtchForPathfind == 0
                    // || cData[currentObjective.droneIndex].swtchForPathfind == 0 && activateThisFramePathfindCounter > activateThisFramePathfindCounterMax
                    if (cData[currentObjective.droneIndex].swtchForPathfind == 1 && activateThisFramePathfindCounter > activateThisFramePathfindCounterMax) // || cData[currentObjective.droneIndex].swtchForPathfind == 0
                    {
                        //REPLACING THE FORMATION WAYPOINT WITH THE PATHFIND WAYPOINT SECTION. TO REVIEW .
                        //REPLACING THE FORMATION WAYPOINT WITH THE PATHFIND WAYPOINT SECTION. TO REVIEW .
                        //REPLACING THE FORMATION WAYPOINT WITH THE PATHFIND WAYPOINT SECTION. TO REVIEW .
                        cData[currentObjective.droneIndex] = SC_AI_Drone_Combat_cc_Rout_For_SFWPP_5.npcPathfindToWaypoint(cData[currentObjective.droneIndex]);

                        if (cData[currentObjective.droneIndex].mSwtc == 5 && cData[currentObjective.droneIndex].pfc == 10 ||
                            cData[currentObjective.droneIndex].mSwtc == 5 && cData[currentObjective.droneIndex].pfc == 2) {

                            if (cData[currentObjective.droneIndex].dog.path != null) {
                                if (cData[currentObjective.droneIndex].dog.path.length > 0) {

                                    var currentFormationWaypointTWO = cData[currentObjective.droneIndex].dog.path[0];
                                    //currentFormationWaypoint = SC_AI_Drone_Combat_cc_Rout_For_FWP_5.npcGWFP(currentObjective, 5);
                                    var distToWaypointTWO = SC_Utilities.GetDistance(currentFormationWaypointTWO, cData[currentObjective.droneIndex].nData.nCoord);
                                    ////////console.PrintError("cData[currentObjective.droneIndex].dog.path.length > 0");

                                    if (!isNaN(distToWaypoint)) {

                                        ////console.PrintError("is!nan distToWaypoint");
                                     
                                        /*if (cData[currentObjective.droneIndex].dog.path[1] != null) {
                                            //////console.PrintError("got pathfind first few tiles direction++");
                                            _currentWaypointPathDirection.x = cData[currentObjective.droneIndex].dog.path[1].x - cData[currentObjective.droneIndex].dog.path[0].x;
                                            _currentWaypointPathDirection.y = cData[currentObjective.droneIndex].dog.path[1].y - cData[currentObjective.droneIndex].dog.path[0].y;

                                        }*/
                                        //else {
                                        //    //////console.PrintError("got pathfind first tile direction");
                                        //    _currentWaypointPathDirection.x = cData[currentObjective.droneIndex].dog.path[0].x - cData[currentObjective.droneIndex].nData.nCoord.x;
                                        //    _currentWaypointPathDirection.y = cData[currentObjective.droneIndex].dog.path[0].y - cData[currentObjective.droneIndex].nData.nCoord.y;
                                        //
                                        //}


                                        //TOREADD
                                        //TOREADD
                                        /*_currentWaypointPathDirection.x = cData[currentObjective.droneIndex].dog.path[0].x - cData[currentObjective.droneIndex].nData.nCoord.x;
                                        _currentWaypointPathDirection.y = cData[currentObjective.droneIndex].dog.path[0].y - cData[currentObjective.droneIndex].nData.nCoord.y;
                                        cData[currentObjective.droneIndex].pData.pCoord = currentFormationWaypointTWO;
                                        cData[currentObjective.droneIndex].pData.pForward = _currentWaypointPathDirection;//cData[currentObjective.droneIndex].pData;
                                        cData[currentObjective.droneIndex].cFWP = currentFormationWaypointTWO;
                                        cData[currentObjective.droneIndex].cFWPD = distToWaypoint;*/
                                        //cData[currentObjective.droneIndex].pData.pForward.x *= -1;//
                                        //cData[currentObjective.droneIndex].pData.pForward.y *= -1;//
                                        //TOREADD
                                        //TOREADD


                                        
                                    }
                                    else //if (isNaN(distToWaypoint))
                                    {
                                        /*////console.PrintError("isnan distToWaypoint");
                                        ////////console.PrintError("ISNAN");

                                        //_currentWaypointPathDirection = cData[currentObjective.droneIndex].pData.pForward;



                                        currentFormationWaypoint = SC_AI_Drone_Combat_cc_Rout_For_FWP_5.npcGWFP(currentObjective, 5);

                                        if (currentFormationWaypoint.x != null && currentFormationWaypoint.y != null) {
                                            distToWaypoint = SC_Utilities.GetDistance(currentFormationWaypoint, cData[currentObjective.droneIndex].nData.nCoord);
                                        }


                                        cData[currentObjective.droneIndex].cFWP = currentFormationWaypoint;
                                        cData[currentObjective.droneIndex].cFWPD = distToWaypoint;*/

                                        /*
                                        cData[currentObjective.droneIndex].mSwtc = 1;
                                        cData[currentObjective.droneIndex].pfc = -2;*/
                                    }
                                }
                                else {
                                    /*_currentWaypointPathDirection = cData[currentObjective.droneIndex].pData.pForward;
                                    currentFormationWaypoint = SC_AI_Drone_Combat_cc_Rout_For_FWP_5.npcGWFP(currentObjective, 5);
                                    distToWaypoint = SC_Utilities.GetDistance(currentFormationWaypoint, cData[currentObjective.droneIndex].nData.nCoord);

                                    cData[currentObjective.droneIndex].cFWP = currentFormationWaypoint;
                                    cData[currentObjective.droneIndex].cFWPD = distToWaypoint;*/
                                    /*cData[currentObjective.droneIndex].mSwtc = 1;
                                    cData[currentObjective.droneIndex].pfc = -2;*/
                                }
                            }
                            else {

                                /*_currentWaypointPathDirection = cData[currentObjective.droneIndex].pData.pForward;

                                currentFormationWaypoint = SC_AI_Drone_Combat_cc_Rout_For_FWP_5.npcGWFP(currentObjective, 5);
                                distToWaypoint = SC_Utilities.GetDistance(currentFormationWaypoint, cData[currentObjective.droneIndex].nData.nCoord);

                                cData[currentObjective.droneIndex].cFWP = currentFormationWaypoint;
                                cData[currentObjective.droneIndex].cFWPD = distToWaypoint;*/
                                /*cData[currentObjective.droneIndex].mSwtc = 1;
                                cData[currentObjective.droneIndex].pfc = -2;*/
                            }

                        }
                        else {
                            /*_currentWaypointPathDirection = cData[currentObjective.droneIndex].pData.pForward;
                            currentFormationWaypoint = SC_AI_Drone_Combat_cc_Rout_For_FWP_5.npcGWFP(currentObjective, 5);
                            distToWaypoint = SC_Utilities.GetDistance(currentFormationWaypoint, cData[currentObjective.droneIndex].nData.nCoord);

                            cData[currentObjective.droneIndex].cFWP = currentFormationWaypoint;
                            cData[currentObjective.droneIndex].cFWPD = distToWaypoint;*/
                            /*cData[currentObjective.droneIndex].mSwtc = 1;
                            cData[currentObjective.droneIndex].pfc = -2;*/
                        }
                        //REPLACING THE FORMATION WAYPOINT WITH THE PATHFIND WAYPOINT SECTION. TO REVIEW .
                        //REPLACING THE FORMATION WAYPOINT WITH THE PATHFIND WAYPOINT SECTION. TO REVIEW .
                        //REPLACING THE FORMATION WAYPOINT WITH THE PATHFIND WAYPOINT SECTION. TO REVIEW .








                        /*if (cData[currentObjective.droneIndex].dog != null) {
                            if (cData[currentObjective.droneIndex].dog.length > 0)
                            {
                                if (cData[currentObjective.droneIndex].dog.path != null) {
                                    if (cData[currentObjective.droneIndex].dog.path.length > 0) {
                                        
                                    }
                                }
                            }
                        }*/

                        /*if (cData[currentObjective.droneIndex].mSwtc == 5 && cData[currentObjective.droneIndex].pfc == 10 ||
                            cData[currentObjective.droneIndex].mSwtc == 5 && cData[currentObjective.droneIndex].pfc == 2)
                        {
                            currentFormationWaypoint = cData[currentObjective.droneIndex].dog.path[0];
                        }*/

                        //pData.pForward = 

                        /*if (cData[currentObjective.droneIndex].mSwtc == 5 && cData[currentObjective.droneIndex].pfc == 10 ||
                            cData[currentObjective.droneIndex].mSwtc == 5 && cData[currentObjective.droneIndex].pfc == 2)
                        {
                            currentFormationWaypoint = cData[currentObjective.droneIndex].dog.path[0];
    
    
                            pData.pCoord = cData[currentObjective.droneIndex].dog.path[1].worldPosition;
                            pData.pForward = nData.nVelo;
                            /*if (cData[currentObjective.droneIndex].dog.path[1] != null)
                            {
                                var distToWaypointFromNCoord = Math.sqrt(((cData[currentObjective.droneIndex].dog.path[1].worldPosition.x - nData.nCoord.x) * (currentFormationWaypoint.x - nData.nCoord.x)) + ((currentFormationWaypoint.y - nData.nCoord.y) * (currentFormationWaypoint.y - nData.nCoord.y)));
    
                                var dirNPCToWaypointFromNCoordX = (cData[currentObjective.droneIndex].dog.path[1].worldPosition.x - nData.nCoord.x) / distToWaypointFromNCoord;
                                var dirNPCToWaypointFromNCoordY = (cData[currentObjective.droneIndex].dog.path[1].worldPosition.y - nData.nCoord.y) / distToWaypointFromNCoord;
                                var dirToWaypointFromNCoord = { x: dirNPCToWaypointFromNCoordX, y: dirNPCToWaypointFromNCoordY };
    
                                pData.pCoord = currentFormationWaypoint;
                                pData.pForward = dirToWaypointFromNCoord
                            }
                        }*/


                        var distToPlayer = Math.sqrt(((currentFormationWaypoint.x - pData.pCoord.x) * (currentFormationWaypoint.x - pData.pCoord.x)) + ((currentFormationWaypoint.y - pData.pCoord.y) * (currentFormationWaypoint.y - pData.pCoord.y)));
                        var distPlayerToDrone = Math.sqrt(((nData.nCoord.x - pData.pCoord.x) * (nData.nCoord.x - pData.pCoord.x)) + ((nData.nCoord.y - pData.pCoord.y) * (nData.nCoord.y - pData.pCoord.y)));

                        var dirNPCToWaypointX = (currentFormationWaypoint.x - nData.nCoord.x) / distToWaypoint;
                        var dirNPCToWaypointY = (currentFormationWaypoint.y - nData.nCoord.y) / distToWaypoint;
                        var dirToWaypoint = { x: dirNPCToWaypointX, y: dirNPCToWaypointY };

                        var dirPlayerToWaypointX = (currentFormationWaypoint.x - pData.pCoord.x) / distToPlayer;
                        var dirPlayerToWaypointY = (currentFormationWaypoint.y - pData.pCoord.y) / distToPlayer;
                        var dirPlayerToWaypoint = { x: dirPlayerToWaypointX, y: dirPlayerToWaypointY };

                        var dirPlayerToDroneX = (nData.nCoord.x - pData.pCoord.x) / distPlayerToDrone;
                        var dirPlayerToDroneY = (nData.nCoord.y - pData.pCoord.y) / distPlayerToDrone;
                        var dirPlayerToDrone = { x: dirPlayerToDroneX, y: dirPlayerToDroneY };

                        var pointInFrontOfWaypointX = currentFormationWaypoint.x + pData.pForward.x;
                        var pointInFrontOfWaypointY = currentFormationWaypoint.y + pData.pForward.y;

                        var pointInFrontOfWaypoint = { x: pointInFrontOfWaypointX, y: pointInFrontOfWaypointY };

                        var dirToPointInFrontOfWaypointX = pointInFrontOfWaypointX - currentFormationWaypoint.x;
                        var dirToPointInFrontOfWaypointY = pointInFrontOfWaypointY - currentFormationWaypoint.y;

                        var dirToPointInFrontOfWaypoint = { x: dirToPointInFrontOfWaypointX, y: dirToPointInFrontOfWaypointY };

                        var someOtherMAG = Math.sqrt(((nData.nCoord.x - pointInFrontOfWaypoint.x) * (nData.nCoord.x - pointInFrontOfWaypoint.x)) + ((nData.nCoord.y - pointInFrontOfWaypoint.y) * (nData.nCoord.y - pointInFrontOfWaypoint.y)));

                        var dirNPCToPointInFrontOfWaypointX = (pointInFrontOfWaypointX - nData.nCoord.x) / someOtherMAG;
                        var dirNPCToPointInFrontOfWaypointY = (pointInFrontOfWaypointY - nData.nCoord.y) / someOtherMAG;

                        var alignedDirectionTowardsWaypointDOT = SC_Utilities.Dot(nData.nForward.x, nData.nForward.y, dirNPCToWaypointX, dirNPCToWaypointY);
                        var alignedDirectionLRDOT = SC_Utilities.Dot(nData.nForward.y, -nData.nForward.x, dirNPCToWaypointX, dirNPCToWaypointY);

                        var alignedDirectionWithPlayerDirectionDOT = SC_Utilities.Dot(pData.pForward.x, pData.pForward.y, nData.nForward.x, nData.nForward.y);
                        var alignedDirectionWithPlayerDirectionRIGHTDOT = SC_Utilities.Dot(nData.nForward.y, -nData.nForward.x, pData.pForward.x, pData.pForward.y);

                        var someOtherFBDOT = SC_Utilities.Dot(dirPlayerToWaypointX, dirPlayerToWaypointY, dirPlayerToDroneX, dirPlayerToDroneY);
                        var someOtherRLDOT = SC_Utilities.Dot(-dirToPointInFrontOfWaypointX, -dirToPointInFrontOfWaypointY, -dirNPCToPointInFrontOfWaypointX, -dirNPCToPointInFrontOfWaypointY);
                        var RLDOTStrafe = SC_Utilities.Dot(-dirToPointInFrontOfWaypointY, dirToPointInFrontOfWaypointX, -dirNPCToPointInFrontOfWaypointX, -dirNPCToPointInFrontOfWaypointY);

                        var isFrontOrBack = SC_Utilities.NSDIST(pData.pCoord, currentFormationWaypoint, nData.nCoord);
                        var isLeftOrRight = SC_Utilities.EWDIST(currentFormationWaypoint, pointInFrontOfWaypoint, nData.nCoord);

                        var veloPlayer = ship.GetVelocity(currentObjective.pid);
                        var speedPlayer = Math.sqrt(veloPlayer.x * veloPlayer.x + veloPlayer.y * veloPlayer.y);
                        veloPlayer.x /= speedPlayer;
                        veloPlayer.y /= speedPlayer;

                        var veloNPC = ship.GetVelocity(currentObjective.nid);
                        var speedNPC = Math.sqrt(veloNPC.x * veloNPC.x + veloNPC.y * veloNPC.y);
                        veloNPC.x /= speedNPC;
                        veloNPC.y /= speedNPC;

                        var angleDegrerRL = Math.abs(Math.atan2(dirToPointInFrontOfWaypointY, dirToPointInFrontOfWaypointX) - Math.atan2(-dirNPCToWaypointY, -dirNPCToWaypointX)); ///* 180 Math.PI

                        var adjacentFB = (distToWaypoint * Math.cos(angleDegrerRL)); //front back
                        var oppositeRLDistance = (distToWaypoint * Math.sin(angleDegrerRL)); //right left

                        if (adjacentFB < 0) {
                            adjacentFB *= -1;
                        }
                        if (oppositeRLDistance < 0) {
                            oppositeRLDistance *= -1;
                        }

                        var distanceDifferenceRL = Math.abs(oppositeRLDistance - lastDistanceToWaypointoppositeRL[currentObjective.droneIndex]);
                        var distanceDifferenceFB = Math.abs(adjacentFB - lastDistanceToWaypointAdjacentFB[currentObjective.droneIndex]);
                        var speedDifference = Math.abs(speedNPC - lastSpeedNPC);

                        var dotVelo = SC_Utilities.Dot(veloPlayer.x, veloPlayer.y, veloNPC.x, veloNPC.y);

                        var originalPlayerShipSpeed = ship.GetFinalCacheValue(CToD.id, "speed_max");
                        var originalNpcShipFORWARDSpeed = ship.GetFinalCacheValue(CToD.id, "speed_max");
                        var originalNpcShipSTRAFESpeed = ship.GetFinalCacheValue(CToD.id, "speed_strafe");

                  
                        /*if (speedNPC > originalNpcShipFORWARDSpeed) {
                            npc.Stop(currentObjective.nid);
                        }*/


                 



                        //FROM THE SCRIPT SC_AI_Drone_Combat_cc_Rout_For_SFWPP_5.js
                        //FROM THE SCRIPT SC_AI_Drone_Combat_cc_Rout_For_SFWPP_5.js
                        var offSetDist = 6.5;
                        var offSetDistTwo = 6.5;

                        /*if (cData[currentObjective.droneIndex] != null)
                        {
                            if (cData[currentObjective.droneIndex].nData != null)
                            {
                                if (cData[currentObjective.droneIndex].nData.nCoord != null)
                                {
                                    if (cData[currentObjective.droneIndex].nData.nCoord.x == null || cData[currentObjective.droneIndex].nData.nCoord.y == null) {
                                        //////console.PrintError("null0");
                                    }
                                    else if (cData[currentObjective.droneIndex].nData.nCoord.x != null && cData[currentObjective.droneIndex].nData.nCoord.y != null) {
                                        //////console.PrintError("!null1");
                                    }
    
    
                                }
                                else
                                {
                                    //////console.PrintError("null nCoord");
                                }
                            }
                            else {
                                //////console.PrintError("null nData");
                            }
                        }
                        else {
                            //////console.PrintError("null cData");
                        }*/





                        cData[currentObjective.droneIndex].nData.nCoord.x = Math.round(cData[currentObjective.droneIndex].nData.nCoord.x);
                        cData[currentObjective.droneIndex].nData.nCoord.y = Math.round(cData[currentObjective.droneIndex].nData.nCoord.y);

                        var distToNode = SC_Utilities.GetDistance(currentFormationWaypoint, cData[currentObjective.droneIndex].nData.nCoord);

                        var dirToWaypointX = ((currentFormationWaypoint.x) - (cData[currentObjective.droneIndex].nData.nCoord.x)) / distToNode;
                        var dirToWaypointY = ((currentFormationWaypoint.y) - (cData[currentObjective.droneIndex].nData.nCoord.y)) / distToNode;

                        var newOffsetWaypointPosX = (currentFormationWaypoint.x + (dirToWaypointX * offSetDist));
                        var newOffsetWaypointPosY = (currentFormationWaypoint.y + (dirToWaypointY * offSetDist));

                        var newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationX = (currentFormationWaypoint.x + (dirToWaypointX * offSetDistTwo));
                        var newOffsetWaypointPosoffSetDistTwoForFastStickToPointRotationY = (currentFormationWaypoint.y + (dirToWaypointY * offSetDistTwo));

                        var newOffsetWaypointPos = { x: newOffsetWaypointPosX, y: newOffsetWaypointPosY };

                        var rotationDOTRL = SC_Utilities.Dot(dirToWaypointY, -dirToWaypointX, -cData[currentObjective.droneIndex].nData.nForward.x, -cData[currentObjective.droneIndex].nData.nForward.y);
                        var rotationDOTFB = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData[currentObjective.droneIndex].nData.nForward.x, cData[currentObjective.droneIndex].nData.nForward.y);
                        var rotationDirectionDOTVELO = SC_Utilities.Dot(dirToWaypointX, dirToWaypointY, cData[currentObjective.droneIndex].nData.nVelo.x, cData[currentObjective.droneIndex].nData.nVelo.y);
                        //FROM THE SCRIPT SC_AI_Drone_Combat_cc_Rout_For_SFWPP_5.js
                        //FROM THE SCRIPT SC_AI_Drone_Combat_cc_Rout_For_SFWPP_5.js











                        var data = null;

                        framesToCoverDistanceAtSpeedRL = 0;
                        framesToCoverDistanceAtSpeedBREAKERRL = 0;

                        for (framesToCoverDistanceAtSpeedRL = 0; framesToCoverDistanceAtSpeedRL <= oppositeRLDistance; framesToCoverDistanceAtSpeedRL += speedNPC) {
                            if (framesToCoverDistanceAtSpeedBREAKERRL >= framesToCoverDistanceAtSpeedBREAKERRLMax) {
                                break;
                            }
                            else {
                                framesToCoverDistanceAtSpeedBREAKERRL++;
                            }
                        }
                        if (framesToCoverDistanceAtSpeedRL > framesToCoverDistanceAtSpeedRLMax) {
                            framesToCoverDistanceAtSpeedRL = framesToCoverDistanceAtSpeedRLMax;
                        }

                        var framesToCoverDistanceAtSpeedFB = 0;
                        var framesToCoverDistanceAtSpeedBREAKERFB = 0;

                        for (framesToCoverDistanceAtSpeedFB = 0; framesToCoverDistanceAtSpeedFB <= adjacentFB; framesToCoverDistanceAtSpeedFB += speedNPC) {
                            if (framesToCoverDistanceAtSpeedBREAKERFB >= framesToCoverDistanceAtSpeedBREAKERFBMax) {
                                break;
                            }
                            else {
                                framesToCoverDistanceAtSpeedBREAKERFB++;
                            }
                        }

                        if (framesToCoverDistanceAtSpeedFB > framesToCoverDistanceAtSpeedFBFrontMAX) {
                            framesToCoverDistanceAtSpeedFB = framesToCoverDistanceAtSpeedFBFrontMAX;
                        }















                        /*
                        var useDotVelo = 1;
    
                        if (!isNaN(dotVelo)) {
                            useDotVelo = dotVelo;
                        }*/


                        //if (distToWaypoint >= minimumDistWaypoint && distToWaypoint <= 25)
                        //{

                            //new bullshit tests that i am doing right now on my other computer.
                            //new bullshit tests that i am doing right now on my other computer.
                            /*if (npc.IsLocked(currentObjective.nid))
                            {
                                npc.Unlock(currentObjective.nid);
                            }
    
                            if (npc.IsStickToPoint(currentObjective.nid) || npc.IsStickToObject(currentObjective.nid))
                            {
                                npc.Unstick(currentObjective.nid);
                            }
    
                            if (oppositeRLDistance >= oppositeRLDistanceMax)
                            {
                                oppositeRLDistance = oppositeRLDistanceMax;
                            }
    
                            if (adjacentFB >= adjacentFBMax)
                            {
                                adjacentFB = adjacentFBMax;
                            }
    
                            var dotToGoalRLLEFT = 1000;
                            var dotToGoalRLRIGHT = 1000;
    
                            if (oppositeRLDistance > lastDistanceToWaypointoppositeRL[currentObjective.droneIndex]) //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER THAN THE LAST FRAME
                            {
                                ////////console.PrintError(speedNPC);
                                if (speedNPC > lastSpeedNPC[currentObjective.droneIndex]) //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER AND GOING FASTER
                                {
                                    if (RLDOTStrafe >= 0) //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER AND GOING FASTER AND NEGATIVE SO NPC IS LEFT
                                    {
                                        //////console.PrintError("oppositeRLDistance >= last frame - DOT: " + RLDOTStrafe);
                                    }
                                    else //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER AND GOING FASTER AND POSITIVE SO NPC IS RIGHT
                                    {
                                        //////console.PrintError("oppositeRLDistance < last frame - DOT: " + RLDOTStrafe);
                                    }
                                }
                                else //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER AND GOING SLOWER
                                {
                                    if (RLDOTStrafe >= 0) //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER AND GOING SLOWER AND NEGATIVE SO NPC IS LEFT
                                    {
                                        //////console.PrintError("oppositeRLDistance >= last frame - DOT: " + RLDOTStrafe);
                                    }
                                    else //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER AND GOING SLOWER AND POSITIVE SO NPC IS RIGHT
                                    {
                                        //////console.PrintError("oppositeRLDistance < last frame - DOT: " + RLDOTStrafe);
                                    }
                                }
                            }
                            else
                            {
                                if (speedNPC > lastSpeedNPC[currentObjective.droneIndex]) //DISTANCE TO WAYPOINT HORIZONTAL IS LOWER AND GOING FASTER
                                {
                                    if (RLDOTStrafe >= 0) //DISTANCE TO WAYPOINT HORIZONTAL IS LOWER AND GOING FASTER AND NEGATIVE SO NPC IS LEFT
                                    {
                                        //////console.PrintError("01");
                                    }
                                    else //DISTANCE TO WAYPOINT HORIZONTAL IS LOWER AND GOING FASTER AND POSITIVE SO NPC IS RIGHT
                                    {
                                        //////console.PrintError("02");
                                    }
                                }
                                else //DISTANCE TO WAYPOINT HORIZONTAL IS LOWER AND GOING SLOWER
                                {
                                    if (RLDOTStrafe >= 0) //DISTANCE TO WAYPOINT HORIZONTAL IS LOWER AND GOING SLOWER AND NEGATIVE SO NPC IS LEFT
                                    {
                                        var tempDistToWaypointRLLEFT = oppositeRLDistance; // * 0.1 //LOWER when getting CLOSER TO waypoint. //HIGHER when getting FAR FROM waypoint.
                                        var tempDistToWaypointRLRIGHT = oppositeRLDistanceMax - oppositeRLDistance;
    
                                        var tempSpeed = speedNPC;
                                        var tempFramesRLRIGHT = framesToCoverDistanceAtSpeedRL;
                                        var tempFramesRLLEFT = framesToCoverDistanceAtSpeedRL;
    
                                        /*if (framesToCoverDistanceAtSpeedRL <= framesToCoverDistanceAtSpeedFBFrontMAX) {
                                            tempFramesRLRIGHT = framesToCoverDistanceAtSpeedFBINVERTER - framesToCoverDistanceAtSpeedRL;
                                            dotToGoalRLRIGHT = Math.floor(tempSpeed * tempDistToWaypointRLRIGHT * tempFramesRLRIGHT);
                                            dotToGoalRLLEFT = Math.floor(tempSpeed * tempDistToWaypointRLLEFT * tempFramesRLLEFT) * 0.75;
                                            pushToTheLEFT[currentObjective.droneIndex] = 1;
                                        }
                                        else {
                                            dotToGoalRLRIGHT = Math.floor(tempSpeed * tempDistToWaypointRLRIGHT * tempFramesRLRIGHT);
                                        }
                                        dotToGoalRLRIGHT = Math.floor(tempSpeed * tempDistToWaypointRLRIGHT * tempFramesRLRIGHT * useDotVelo);
                                        ////////console.PrintError(dotToGoalRLRIGHT);                      
                                        pushToTheRIGHT[currentObjective.droneIndex] = 1;
    
                                        //////console.PrintError("02: " + dotToGoalRLLEFT + " ___ " + dotToGoalRLRIGHT);
                                    }
                                    else //DISTANCE TO WAYPOINT HORIZONTAL IS LOWER AND GOING SLOWER AND POSITIVE SO NPC IS RIGHT
                                    {
                                        ////////console.PrintError("04");
                                        var tempDistToWaypointRLLEFT = oppositeRLDistanceMax - oppositeRLDistance; // * 0.1 //LOWER when getting CLOSER TO waypoint. //HIGHER when getting FAR FROM waypoint.
                                        var tempDistToWaypointRLRIGHT = oppositeRLDistance;
    
    
                                        //tempDistToWaypointRL = 1 - tempDistToWaypointRL; //LOWER when getting FAR AWAY from waypoint. //HIGHER when getting CLOSER TO waypoint.
    
                                        var tempSpeed = speedNPC;
                                        var tempFramesRLLEFT = framesToCoverDistanceAtSpeedRL;
                                        var tempFramesRLRIGHT = framesToCoverDistanceAtSpeedRL;
    
                                        /*if (framesToCoverDistanceAtSpeedRL <= framesToCoverDistanceAtSpeedFBFrontMAX)
                                        {
                                            tempFramesRLLEFT = framesToCoverDistanceAtSpeedFBINVERTER - framesToCoverDistanceAtSpeedRL;
                                            dotToGoalRLLEFT = Math.floor(tempSpeed * tempDistToWaypointRLLEFT * tempFramesRLLEFT);
                                            dotToGoalRLRIGHT = Math.floor(tempSpeed * tempDistToWaypointRLRIGHT * tempFramesRLRIGHT) * 0.75;
                                            pushToTheRIGHT[currentObjective.droneIndex] = 1;
                                        }
                                        else {
                                            dotToGoalRLLEFT = Math.floor(tempSpeed * tempDistToWaypointRLLEFT * tempFramesRLLEFT);
                                        }
                                        dotToGoalRLLEFT = Math.floor(tempSpeed * tempDistToWaypointRLLEFT * tempFramesRLLEFT * useDotVelo);
                                        pushToTheLEFT[currentObjective.droneIndex] = 1;
    
                                        //////console.PrintError("03: " + dotToGoalRLLEFT + " ___ " + dotToGoalRLRIGHT);
                                        ////////console.PrintError("03");
                                    }
                                }
                            }*/
                            //new bullshit tests that i am doing right now on my other computer.
                            //new bullshit tests that i am doing right now on my other computer.











                            //CURRENT TESTS
                            //CURRENT TESTS
                            //CURRENT TESTS
                            


                            /*if (npc.IsLocked(currentObjective.nid)) {
                                npc.Unlock(currentObjective.nid);
                            }

                            if (npc.IsStickToPoint(currentObjective.nid) || npc.IsStickToObject(currentObjective.nid)) {
                                npc.Unstick(currentObjective.nid);
                            }*/


                            if (oppositeRLDistance >= oppositeRLDistanceMax) {
                                oppositeRLDistance = oppositeRLDistanceMax;
                            }

                            if (adjacentFB >= adjacentFBMax) {
                                adjacentFB = adjacentFBMax;
                            }

                            var dotToGoalRLLEFT = 1000;
                            var dotToGoalRLRIGHT = 1000;


                            //////console.PrintError("RLDOTStrafe " + RLDOTStrafe);


                            //RLDOTStrafe => the dot is closing in on 0 when the drone is heading towards the waypoint.

                            var RLDOTStrafeMax = 0;


                            //if (cData[currentObjective.droneIndex].veloLF > 0.99)
                            //{
                            //    if (rotationDirectionDOTVELO > 0.99) {
                            //
                            //    }
                            //}


                            var speedCushionNearingWaypoint = -1;

                            //npc.Stop(currentObjective.nid);

                            //FOR STRAFE RIGHT AND LEFT
                            //RLDOTStrafe positive => waypoint is to the right. => PUSHTOTHERIGHT
                            //RLDOTStrafe negative => waypoint is to the left. => PUSHTOTHELEFT
                            /*if (cData[currentObjective.droneIndex].RLDOTStrafeLast > 0 && RLDOTStrafe <= 0) //waypoint changed sides. waypoint was to the right and now waypoint is to the left.
                            {
                                if (speedNPC > 0)
                                {
                                    //////console.PrintError("0");
                                    var tempFrames = framesToCoverDistanceAtSpeedRL;
                                    dotToGoalRLRIGHT = RLDOTStrafe;//Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                    pushToTheRIGHT[currentObjective.droneIndex] = 1;
                                }
                                else {
                                    //////console.PrintError("1");
                                    var tempFrames = framesToCoverDistanceAtSpeedRL;
                                    dotToGoalRLLEFT = (RLDOTStrafe);//Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                    pushToTheLEFT[currentObjective.droneIndex] = 1;
                                    //npc.Stop(currentObjective.nid);
                                    speedCushionNearingWaypoint = 0;
                                }
                            }
                            else if (cData[currentObjective.droneIndex].RLDOTStrafeLast < 0 && RLDOTStrafe <= 0) // drone is to the RIGHT    and going to the LEFT
                            {
                                if (RLDOTStrafe > -0.85)
                                {
                                    //////console.PrintError("3");
                                    var tempFrames = framesToCoverDistanceAtSpeedRL;
                                    dotToGoalRLRIGHT = 0;//Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                    pushToTheRIGHT[currentObjective.droneIndex] = 1;
                                    //npc.Stop(currentObjective.nid);
                                }
                                else
                                {
                                    //////console.PrintError("33");
                                    var tempFrames = framesToCoverDistanceAtSpeedRL;
                                    dotToGoalRLLEFT = (RLDOTStrafe) * tempFrames;//Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                    pushToTheLEFT[currentObjective.droneIndex] = 1;
                                }
                            }
                            else if (cData[currentObjective.droneIndex].RLDOTStrafeLast > 0 && RLDOTStrafe >= 0) //// drone is to the LEFT and going to the RIGHT
                            {
                                if (RLDOTStrafe < 0.85)
                                {
                                    //////console.PrintError("4");
                                    var tempFrames = framesToCoverDistanceAtSpeedRL;
                                    dotToGoalRLLEFT = (0);//Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                    pushToTheLEFT[currentObjective.droneIndex] = 1;
                                }
                                else {
                                    //////console.PrintError("44");
                                    var tempFrames = framesToCoverDistanceAtSpeedRL;
                                    dotToGoalRLRIGHT = ((1 - RLDOTStrafe) * (framesToCoverDistanceAtSpeedRLMax - tempFrames));//Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                    pushToTheRIGHT[currentObjective.droneIndex] = 1;
                                    //////console.PrintError("DOT: " + dotToGoalRLRIGHT);
                                }
                            }
                            else if (cData[currentObjective.droneIndex].RLDOTStrafeLast < 0 && RLDOTStrafe >= 0) ////waypoint changed sides. waypoint was to the left and now waypoint is to the right.
                            {
                                if (speedNPC > 0)
                                {
                                    //////console.PrintError("5");
                                    var tempFrames = framesToCoverDistanceAtSpeedRL;
                                    dotToGoalRLRIGHT = (0);//Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                    pushToTheRIGHT[currentObjective.droneIndex] = 1;
                                    //npc.Stop(currentObjective.nid);
                                    speedCushionNearingWaypoint = 1;

                                }
                                else {
                                    //////console.PrintError("6");
                                    var tempFrames = framesToCoverDistanceAtSpeedRL;
                                    dotToGoalRLLEFT = (RLDOTStrafe);//Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                    pushToTheLEFT[currentObjective.droneIndex] = 1;
                                }
                            }
                            else
                            {
                                //////console.PrintError("7");
                            }




                            if (speedCushionNearingWaypoint == 0 || speedCushionNearingWaypoint == 1)
                            {
                                if (speedCushionNearingWaypoint == 0)
                                {

                                }
                                else if (speedCushionNearingWaypoint == 1)
                                {

                                }
                            }*/










                            //oppositeRLLast: 0,
                            //adjacentRLLast: 0


                            /*
                            if (oppositeRLDistance > lastDistanceToWaypointoppositeRL[currentObjective.droneIndex]) //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER THAN THE LAST FRAME
                            {
                                if (speedNPC > lastSpeedNPC[currentObjective.droneIndex]) //the speed is increasing and the drone is heading towards the waypoint
                                {
                                    if (RLDOTStrafe >= RLDOTStrafeMax) //waypoint is to the right
                                    {
                                        var tempFrames = framesToCoverDistanceAtSpeedRL;
                                        dotToGoalRLLEFT = (1 - RLDOTStrafe) ;// Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                        pushToTheLEFT[currentObjective.droneIndex] = 1;
                                        //////console.PrintError("0");
                                    }
                                    else  //waypoint is to the left
                                    {
                                        var tempFrames = framesToCoverDistanceAtSpeedRL;
                                        dotToGoalRLRIGHT = (1 - RLDOTStrafe);//Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                        pushToTheRIGHT[currentObjective.droneIndex] = 1;
                                        //////console.PrintError("1");
                                    }
                                }
                                else //the speed is decreasing and the drone is heading towards the waypoint
                                {
                                    if (RLDOTStrafe >= RLDOTStrafeMax) //waypoint is to the right
                                    {
                                        var tempFrames = framesToCoverDistanceAtSpeedRL;
                                        dotToGoalRLRIGHT = RLDOTStrafe;// Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                        pushToTheRIGHT[currentObjective.droneIndex] = 1;
                                        //////console.PrintError("2");
                                    }
                                    else  //waypoint is to the left
                                    {
                                        var tempFrames = framesToCoverDistanceAtSpeedRL;
                                        dotToGoalRLLEFT = RLDOTStrafe;//Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                        pushToTheLEFT[currentObjective.droneIndex] = 1;
                                        //////console.PrintError("3");
                                    }
                                }
                            }
                            else //DISTANCE TO WAYPOINT HORIZONTAL IS SMALLER THAN THE LAST FRAME
                            {
                                if (speedNPC > lastSpeedNPC[currentObjective.droneIndex]) //the speed is increasing and the drone is heading towards the waypoint
                                {
                                    if (RLDOTStrafe >= RLDOTStrafeMax) //waypoint is to the right
                                    {
                                        var tempFrames = framesToCoverDistanceAtSpeedRL;
                                        dotToGoalRLRIGHT = (RLDOTStrafe) ;// Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                        pushToTheRIGHT[currentObjective.droneIndex] = 1;
                                        //////console.PrintError("4");
                                    }
                                    else  //waypoint is to the left
                                    {
                                        var tempFrames = framesToCoverDistanceAtSpeedRL;
                                        dotToGoalRLLEFT = (RLDOTStrafe) ;//Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                        pushToTheLEFT[currentObjective.droneIndex] = 1;
                                        //////console.PrintError("5");
                                    }
                                }
                                else {
                                    if (RLDOTStrafe >= RLDOTStrafeMax) //waypoint is to the right
                                    {
                                        var tempFrames = framesToCoverDistanceAtSpeedRL;
                                        dotToGoalRLLEFT = RLDOTStrafe ;//Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                        pushToTheLEFT[currentObjective.droneIndex] = 1;
                                        //////console.PrintError("6");
                                    }
                                    else  //waypoint is to the left
                                    {
                                        var tempFrames = framesToCoverDistanceAtSpeedRL;
                                        dotToGoalRLRIGHT = RLDOTStrafe ;// Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                        pushToTheRIGHT[currentObjective.droneIndex] = 1;
                                        //////console.PrintError("7");
                                       
                                    }
                                }
                            }*/




                            //CURRENT TESTS
                            //CURRENT TESTS
                            //CURRENT TESTS
                            






















                            //TO READD
                            //TO READD
                            //TO READD
                            /*if (npc.IsLocked(currentObjective.nid)) {
                                npc.Unlock(currentObjective.nid);
                            }

                            if (npc.IsStickToPoint(currentObjective.nid) || npc.IsStickToObject(currentObjective.nid)) {
                                npc.Unstick(currentObjective.nid);
                            }*/


                            if (oppositeRLDistance >= oppositeRLDistanceMax) {
                                oppositeRLDistance = oppositeRLDistanceMax;
                            }

                            if (adjacentFB >= adjacentFBMax) {
                                adjacentFB = adjacentFBMax;
                            }

                            var dotToGoalRLLEFT = 1000;
                            var dotToGoalRLRIGHT = 1000;

                            if (oppositeRLDistance > lastDistanceToWaypointoppositeRL[currentObjective.droneIndex]) //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER
                            {
                                ////////console.PrintError(speedNPC);
                                if (speedNPC > lastSpeedNPC[currentObjective.droneIndex]) //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER AND GOING FASTER
                                {
                                    if (RLDOTStrafe >= 0) //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER AND GOING FASTER AND NEGATIVE SO NPC IS LEFT
                                    {
                                        //if (oppositeRLDistance <= 1)
                                        //{
                                        //    oppositeRLDistance = 0;     
                                        //}
                                        var tempDistToWaypointRL = oppositeRLDistance; //LOWER when getting CLOSER TO waypoint. //HIGHER when getting FAR FROM waypoint.
                                        //tempDistToWaypointRL = 1 - tempDistToWaypointRL; //LOWER when getting FAR AWAY from waypoint. //HIGHER when getting CLOSER TO waypoint.

                                        var tempSpeed = speedNPC;
                                        var tempFrames = framesToCoverDistanceAtSpeedRL;

                                        if (framesToCoverDistanceAtSpeedRL <= 2) {
                                            //tempFrames = 10 - framesToCoverDistanceAtSpeedRL;
                                        }

                                        dotToGoalRLRIGHT = Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                        ////////console.PrintError("00: " + dotToGoalRLLEFT + " ___ " + dotToGoalRLRIGHT);
                                        pushToTheRIGHT[currentObjective.droneIndex] = 1;
                                        ////////console.PrintError("00");
                                    }
                                    else //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER AND GOING FASTER AND POSITIVE SO NPC IS RIGHT
                                    {
                                        //if (oppositeRLDistance <= 1)
                                        //{
                                        //    oppositeRLDistance = 0;
                                        //}
                                        var tempDistToWaypointRL = oppositeRLDistance; //LOWER when getting CLOSER TO waypoint. //HIGHER when getting FAR FROM waypoint.
                                        //tempDistToWaypointRL = 1 - tempDistToWaypointRL; //LOWER when getting FAR AWAY from waypoint. //HIGHER when getting CLOSER TO waypoint.

                                        var tempSpeed = speedNPC;
                                        var tempFrames = framesToCoverDistanceAtSpeedRL;

                                        if (framesToCoverDistanceAtSpeedRL <= 2) {
                                            //tempFrames = 10 - framesToCoverDistanceAtSpeedRL;
                                        }

                                        dotToGoalRLLEFT = Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                        ////////console.PrintError("01: " + dotToGoalRLLEFT + " ___ " + dotToGoalRLRIGHT);
                                        pushToTheLEFT[currentObjective.droneIndex] = 1;
                                        ////////console.PrintError("01");
                                    }
                                }
                                else //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER AND GOING SLOWER
                                {
                                    if (RLDOTStrafe >= 0) //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER AND GOING SLOWER AND NEGATIVE SO NPC IS LEFT
                                    {
                                        //if (oppositeRLDistance <= 1)
                                        //{
                                        //    oppositeRLDistance = 0;
                                        //}
                                        var tempDistToWaypointRL = Math.ceil(oppositeRLDistance); //LOWER when getting CLOSER TO waypoint. //HIGHER when getting FAR FROM waypoint.
                                        tempDistToWaypointRL = tempDistToWaypointRL - oppositeRLDistance;

                                        var tempSpeed = Math.ceil(speedNPC);
                                        tempSpeed = tempSpeed - speedNPC;

                                        var tempFrames = framesToCoverDistanceAtSpeedRL;

                                        if (framesToCoverDistanceAtSpeedRL <= 2) {
                                            //tempFrames = 10 - framesToCoverDistanceAtSpeedRL;
                                        }

                                        dotToGoalRLRIGHT = Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                        ////////console.PrintError("02: " + dotToGoalRLLEFT + " ___ " + dotToGoalRLRIGHT);
                                        pushToTheRIGHT[currentObjective.droneIndex] = 1;
                                        ////////console.PrintError("02");
                                    }
                                    else //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER AND GOING SLOWER AND POSITIVE SO NPC IS RIGHT
                                    {
                                        //if (oppositeRLDistance <= 1)
                                        //{
                                        //    oppositeRLDistance = 0;
                                        //}
                                        var tempDistToWaypointRL = Math.ceil(oppositeRLDistance); //LOWER when getting CLOSER TO waypoint. //HIGHER when getting FAR FROM waypoint.
                                        tempDistToWaypointRL = tempDistToWaypointRL - oppositeRLDistance;

                                        //tempDistToWaypointRL = 1 - tempDistToWaypointRL; //LOWER when getting FAR AWAY from waypoint. //HIGHER when getting CLOSER TO waypoint.

                                        var tempSpeed = Math.ceil(speedNPC);
                                        tempSpeed = tempSpeed - speedNPC;
                                        var tempFrames = framesToCoverDistanceAtSpeedRL;

                                        if (framesToCoverDistanceAtSpeedRL <= 2) {
                                            //tempFrames = 10 - framesToCoverDistanceAtSpeedRL;
                                        }

                                        ////////console.PrintError("03: " + dotToGoalRLLEFT + " ___ " + dotToGoalRLRIGHT);
                                        ////////console.PrintError(tempSpeed * tempDistToWaypointRL * tempFrames);
                                        dotToGoalRLLEFT = Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                        pushToTheLEFT[currentObjective.droneIndex] = 1;
                                        ////////console.PrintError("03");
                                    }
                                }
                                ////////console.PrintError(dotToGoalRLLEFT + " ___ " + dotToGoalRLRIGHT);
                            }
                            else //DISTANCE IS GETTING LOWER
                            {
                                ////////console.PrintError(framesToCoverDistanceAtSpeedRL);
                                if (speedNPC > lastSpeedNPC[currentObjective.droneIndex]) //DISTANCE TO WAYPOINT HORIZONTAL IS LOWER AND GOING FASTER
                                {
                                    if (RLDOTStrafe >= 0) //DISTANCE TO WAYPOINT HORIZONTAL IS LOWER AND GOING FASTER AND NEGATIVE SO NPC IS LEFT
                                    {
                                        var tempDistToWaypointRL = oppositeRLDistanceMax - oppositeRLDistance;  // * 0.1//LOWER when getting CLOSER TO waypoint. //HIGHER when getting FAR FROM waypoint.


                                        //tempDistToWaypointRL = 1 - tempDistToWaypointRL; //LOWER when getting FAR AWAY from waypoint. //HIGHER when getting CLOSER TO waypoint.
                                        ////////console.PrintError(oppositeRLDistance);
                                        var tempSpeed = speedNPC;
                                        var tempFramesRLRIGHT = framesToCoverDistanceAtSpeedRL;
                                        var tempFramesRLLEFT = framesToCoverDistanceAtSpeedRL;


                                        if (framesToCoverDistanceAtSpeedRL <= framesToCoverDistanceAtSpeedFBFrontMAX) {

                                            tempFramesRLRIGHT = framesToCoverDistanceAtSpeedFBINVERTER - framesToCoverDistanceAtSpeedRL;
                                            dotToGoalRLRIGHT = Math.floor(tempSpeed * tempDistToWaypointRL * tempFramesRLRIGHT);

                                            dotToGoalRLLEFT = Math.floor(tempSpeed * tempDistToWaypointRL * tempFramesRLLEFT) * 1.5;
                                            pushToTheLEFT[currentObjective.droneIndex] = 1;
                                        }
                                        else {
                                            dotToGoalRLRIGHT = Math.floor(tempSpeed * tempDistToWaypointRL * tempFramesRLRIGHT);
                                        }

                                        //////console.PrintError("00: " + dotToGoalRLLEFT + " ___ " + dotToGoalRLRIGHT);
                                        pushToTheRIGHT[currentObjective.droneIndex] = 1;

                                    }
                                    else //DISTANCE TO WAYPOINT HORIZONTAL IS LOWER AND GOING FASTER AND POSITIVE SO NPC IS RIGHT
                                    {


                                        ////////console.PrintError(oppositeRLDistance);
                                        var tempDistToWaypointRL = oppositeRLDistanceMax - oppositeRLDistance;  // * 0.1 //LOWER when getting CLOSER TO waypoint. //HIGHER when getting FAR FROM waypoint.
                                        //tempDistToWaypointRL = 1 - tempDistToWaypointRL; //LOWER when getting FAR AWAY from waypoint. //HIGHER when getting CLOSER TO waypoint.

                                        var tempSpeed = speedNPC;

                                        var tempFramesRLRIGHT = framesToCoverDistanceAtSpeedRL;
                                        var tempFramesRLLEFT = framesToCoverDistanceAtSpeedRL;


                                        if (framesToCoverDistanceAtSpeedRL <= framesToCoverDistanceAtSpeedFBFrontMAX) {
                                            tempFramesRLLEFT = framesToCoverDistanceAtSpeedFBINVERTER - framesToCoverDistanceAtSpeedRL;
                                            dotToGoalRLLEFT = Math.floor(tempSpeed * tempDistToWaypointRL * tempFramesRLLEFT);
                                            dotToGoalRLRIGHT = Math.floor(tempSpeed * tempDistToWaypointRL * tempFramesRLRIGHT) * 1.5;
                                            pushToTheRIGHT[currentObjective.droneIndex] = 1;
                                        }
                                        else {
                                            dotToGoalRLLEFT = Math.floor(tempSpeed * tempDistToWaypointRL * tempFramesRLLEFT);
                                        }


                                        pushToTheLEFT[currentObjective.droneIndex] = 1;
                                        //////console.PrintError("01: " + dotToGoalRLLEFT + " ___ " + dotToGoalRLRIGHT);
                                    }
                                }
                                else //DISTANCE TO WAYPOINT HORIZONTAL IS LOWER AND GOING SLOWER
                                {
                                    if (RLDOTStrafe >= 0) //DISTANCE TO WAYPOINT HORIZONTAL IS LOWER AND GOING SLOWER AND NEGATIVE SO NPC IS LEFT
                                    {


                                        var tempDistToWaypointRLLEFT = oppositeRLDistance; // * 0.1 //LOWER when getting CLOSER TO waypoint. //HIGHER when getting FAR FROM waypoint.
                                        var tempDistToWaypointRLRIGHT = oppositeRLDistanceMax - oppositeRLDistance;


                                        var tempSpeed = speedNPC;
                                        var tempFramesRLRIGHT = framesToCoverDistanceAtSpeedRL;
                                        var tempFramesRLLEFT = framesToCoverDistanceAtSpeedRL;

                                        if (framesToCoverDistanceAtSpeedRL <= framesToCoverDistanceAtSpeedFBFrontMAX) {
                                            tempFramesRLRIGHT = framesToCoverDistanceAtSpeedFBINVERTER - framesToCoverDistanceAtSpeedRL;
                                            dotToGoalRLRIGHT = Math.floor(tempSpeed * tempDistToWaypointRLRIGHT * tempFramesRLRIGHT);
                                            dotToGoalRLLEFT = Math.floor(tempSpeed * tempDistToWaypointRLLEFT * tempFramesRLLEFT) * 0.75;
                                            pushToTheLEFT[currentObjective.droneIndex] = 1;
                                        }
                                        else {
                                            dotToGoalRLRIGHT = Math.floor(tempSpeed * tempDistToWaypointRLRIGHT * tempFramesRLRIGHT);
                                        }
                                        ////////console.PrintError(dotToGoalRLRIGHT);                      
                                        pushToTheRIGHT[currentObjective.droneIndex] = 1;

                                        //////console.PrintError("02: " + dotToGoalRLLEFT + " ___ " + dotToGoalRLRIGHT);
                                    }
                                    else //DISTANCE TO WAYPOINT HORIZONTAL IS LOWER AND GOING SLOWER AND POSITIVE SO NPC IS RIGHT
                                    {

                                        var tempDistToWaypointRLLEFT = oppositeRLDistanceMax - oppositeRLDistance; // * 0.1 //LOWER when getting CLOSER TO waypoint. //HIGHER when getting FAR FROM waypoint.
                                        var tempDistToWaypointRLRIGHT = oppositeRLDistance;


                                        //tempDistToWaypointRL = 1 - tempDistToWaypointRL; //LOWER when getting FAR AWAY from waypoint. //HIGHER when getting CLOSER TO waypoint.

                                        var tempSpeed = speedNPC;
                                        var tempFramesRLLEFT = framesToCoverDistanceAtSpeedRL;
                                        var tempFramesRLRIGHT = framesToCoverDistanceAtSpeedRL;

                                        if (framesToCoverDistanceAtSpeedRL <= framesToCoverDistanceAtSpeedFBFrontMAX) {
                                            tempFramesRLLEFT = framesToCoverDistanceAtSpeedFBINVERTER - framesToCoverDistanceAtSpeedRL;
                                            dotToGoalRLLEFT = Math.floor(tempSpeed * tempDistToWaypointRLLEFT * tempFramesRLLEFT);
                                            dotToGoalRLRIGHT = Math.floor(tempSpeed * tempDistToWaypointRLRIGHT * tempFramesRLRIGHT) * 0.75;
                                            pushToTheRIGHT[currentObjective.droneIndex] = 1;
                                        }
                                        else {
                                            dotToGoalRLLEFT = Math.floor(tempSpeed * tempDistToWaypointRLLEFT * tempFramesRLLEFT);
                                        }
                                        pushToTheLEFT[currentObjective.droneIndex] = 1;

                                        //////console.PrintError("03: " + dotToGoalRLLEFT + " ___ " + dotToGoalRLRIGHT);
                                        ////////console.PrintError("03");
                                    }
                                }
                            }
                            //TO READD
                            //TO READD
                            //TO READD


                            ////console.PrintError("dotToGoalRLRIGHT: " + dotToGoalRLRIGHT + " dotToGoalRLLEFT: " + dotToGoalRLLEFT);








                            var dotToGoalRLBACK = 1000;
                            var dotToGoalRLFRONT = 1000;

                            ////////console.PrintError(someOtherRLDOT);

                            /////////////////////////////////////////////////FRONT BACK
                            /////////////////////////////////////////////////FRONT BACK
                            /////////////////////////////////////////////////FRONT BACK

                            if (adjacentFB > lastDistanceToWaypointAdjacentFB[currentObjective.droneIndex]) //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER
                            {
                                ////////console.PrintError(speedNPC);
                                if (speedNPC > lastSpeedNPC[currentObjective.droneIndex]) //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER AND GOING FASTER
                                {
                                    if (someOtherRLDOT >= 0) //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER AND GOING FASTER AND NEGATIVE SO NPC IS LEFT
                                    {
                                        //if (adjacentFB <= 1)
                                        //{
                                        //    adjacentFB = 0;
                                        //}

                                        var tempDistToWaypointRL = Math.ceil(adjacentFB); //LOWER when getting CLOSER TO waypoint. //HIGHER when getting FAR FROM waypoint.
                                        tempDistToWaypointRL = tempDistToWaypointRL - adjacentFB;

                                        var tempSpeed = speedNPC;
                                        var tempFrames = framesToCoverDistanceAtSpeedFB;

                                        if (framesToCoverDistanceAtSpeedFB <= framesToCoverDistanceAtSpeedFBFrontMAX)
                                        {
                                            tempFrames = adjacentFBMax - framesToCoverDistanceAtSpeedFB;
                                        }

                                        dotToGoalRLFRONT = Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);



                                        ////console.PrintError("00: " + dotToGoalRLBACK + " ___ " + dotToGoalRLFRONT);
                                        pushFRONT[currentObjective.droneIndex] = 1;
                                    }
                                    else //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER AND GOING FASTER AND POSITIVE SO NPC IS RIGHT
                                    {
                                        //if (adjacentFB <= 1)
                                        //{
                                        //    adjacentFB = 0;
                                        //}
                                        var tempDistToWaypointRL = adjacentFB; //LOWER when getting CLOSER TO waypoint. //HIGHER when getting FAR FROM waypoint.


                                        var tempSpeed = speedNPC;
                                        var tempFrames = framesToCoverDistanceAtSpeedFB;

                                        if (framesToCoverDistanceAtSpeedFB >= framesToCoverDistanceAtSpeedFBBACKMAX) {
                                            tempFrames = adjacentFBMax - framesToCoverDistanceAtSpeedFB;
                                        }
                                        //else if (framesToCoverDistanceAtSpeedFB <= framesToCoverDistanceAtSpeedFBFrontMAX)
                                        //{
                                        //    tempFrames = 10 - framesToCoverDistanceAtSpeedFB;
                                        //}

                                        dotToGoalRLBACK = Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                        ////console.PrintError("01: " + dotToGoalRLBACK + " ___ " + dotToGoalRLFRONT);
                                        pushBACK[currentObjective.droneIndex] = 1;
                                    }
                                }
                                else //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER AND GOING SLOWER
                                {
                                    if (someOtherRLDOT >= 0) //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER AND GOING SLOWER AND NEGATIVE SO NPC IS LEFT
                                    {
                                        //if (adjacentFB <= 1)
                                        //{
                                        //    adjacentFB = 0;
                                        //}
                                        var tempDistToWaypointRL = Math.ceil(adjacentFB); //LOWER when getting CLOSER TO waypoint. //HIGHER when getting FAR FROM waypoint.
                                        tempDistToWaypointRL = tempDistToWaypointRL - adjacentFB;

                                        var tempSpeed = Math.ceil(speedNPC);
                                        tempSpeed = tempSpeed - speedNPC;

                                        var tempFrames = framesToCoverDistanceAtSpeedFB;

                                        if (framesToCoverDistanceAtSpeedFB >= framesToCoverDistanceAtSpeedFBBACKMAX) {
                                            tempFrames = adjacentFBMax - framesToCoverDistanceAtSpeedRL;
                                        }

                                        dotToGoalRLFRONT = Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                        ////console.PrintError("02: " + dotToGoalRLBACK + " ___ " + dotToGoalRLFRONT);
                                        pushFRONT[currentObjective.droneIndex] = 1;
                                    }
                                    else //DISTANCE TO WAYPOINT HORIZONTAL IS HIGHER AND GOING SLOWER AND POSITIVE SO NPC IS RIGHT
                                    {
                                        if (adjacentFB <= 1) {
                                            adjacentFB = 0;
                                            //pushToTheRIGHT = 1;
                                            //dotToGoalRLRIGHT = 0;
                                        }
                                        var tempDistToWaypointRL = Math.ceil(adjacentFB); //LOWER when getting CLOSER TO waypoint. //HIGHER when getting FAR FROM waypoint.
                                        tempDistToWaypointRL = tempDistToWaypointRL - adjacentFB;

                                        //tempDistToWaypointRL = 1 - tempDistToWaypointRL; //LOWER when getting FAR AWAY from waypoint. //HIGHER when getting CLOSER TO waypoint.

                                        var tempSpeed = Math.ceil(speedNPC);
                                        tempSpeed = tempSpeed - speedNPC;

                                        var tempFrames = framesToCoverDistanceAtSpeedFB;

                                        if (framesToCoverDistanceAtSpeedFB >= framesToCoverDistanceAtSpeedFBBACKMAX) {
                                            tempFrames = adjacentFBMax - framesToCoverDistanceAtSpeedRL;
                                        }

                                        dotToGoalRLBACK = Math.round(tempSpeed * tempDistToWaypointRL * tempFrames);
                                        ////console.PrintError("03: " + dotToGoalRLBACK + " ___ " + dotToGoalRLFRONT);
                                        pushBACK[currentObjective.droneIndex] = 1;
                                    }
                                }
                                ////////console.PrintError(dotToGoalRLLEFT + " ___ " + dotToGoalRLRIGHT);
                            }
                            else //DISTANCE IS GETTING LOWER
                            {
                                ////////console.PrintError(framesToCoverDistanceAtSpeedFB);
                                if (speedNPC > lastSpeedNPC[currentObjective.droneIndex]) //DISTANCE TO WAYPOINT HORIZONTAL IS LOWER AND GOING FASTER
                                {
                                    if (someOtherRLDOT >= 0) //DISTANCE TO WAYPOINT HORIZONTAL IS LOWER AND GOING FASTER AND NEGATIVE SO NPC IS LEFT
                                    {

                                        var tempDistToWaypointRL = adjacentFBMax - adjacentFB;  // * 0.1//LOWER when getting CLOSER TO waypoint. //HIGHER when getting FAR FROM waypoint.

                                        //tempDistToWaypointRL = 1 - tempDistToWaypointRL; //LOWER when getting FAR AWAY from waypoint. //HIGHER when getting CLOSER TO waypoint.
                                        ////////console.PrintError(oppositeRLDistance);
                                        var tempSpeed = speedNPC;
                                        var tempFramesRLRIGHT = framesToCoverDistanceAtSpeedFB;
                                        var tempFramesRLLEFT = framesToCoverDistanceAtSpeedFB;


                                        if (framesToCoverDistanceAtSpeedFB <= framesToCoverDistanceAtSpeedFBFrontMAX) {
                                            tempFramesRLRIGHT = framesToCoverDistanceAtSpeedFBINVERTER - framesToCoverDistanceAtSpeedFB;
                                            dotToGoalRLFRONT = Math.floor(tempSpeed * tempDistToWaypointRL * tempFramesRLRIGHT);

                                            dotToGoalRLBACK = Math.floor(tempSpeed * tempDistToWaypointRL * tempFramesRLLEFT) * 1.5;
                                            pushBACK[currentObjective.droneIndex] = 1;
                                        }
                                        else {
                                            dotToGoalRLFRONT = Math.floor(tempSpeed * tempDistToWaypointRL * tempFramesRLRIGHT);
                                        }

                                        ////////console.PrintError("00: " + dotToGoalRLBACK + " ___ " + dotToGoalRLFRONT);
                                        pushFRONT[currentObjective.droneIndex] = 1;

                                    }
                                    else //DISTANCE TO WAYPOINT HORIZONTAL IS LOWER AND GOING FASTER AND POSITIVE SO NPC IS RIGHT
                                    {

                                        ////////console.PrintError(oppositeRLDistance);
                                        var tempDistToWaypointRL = adjacentFBMax - adjacentFB;  // * 0.1 //LOWER when getting CLOSER TO waypoint. //HIGHER when getting FAR FROM waypoint.
                                        //tempDistToWaypointRL = 1 - tempDistToWaypointRL; //LOWER when getting FAR AWAY from waypoint. //HIGHER when getting CLOSER TO waypoint.

                                        var tempSpeed = speedNPC;

                                        var tempFramesRLRIGHT = framesToCoverDistanceAtSpeedFB;
                                        var tempFramesRLLEFT = framesToCoverDistanceAtSpeedFB;


                                        if (framesToCoverDistanceAtSpeedFB <= framesToCoverDistanceAtSpeedFBFrontMAX) {
                                            tempFramesRLLEFT = framesToCoverDistanceAtSpeedFBINVERTER - framesToCoverDistanceAtSpeedFB;
                                            dotToGoalRLBACK = Math.floor(tempSpeed * tempDistToWaypointRL * tempFramesRLLEFT);
                                            dotToGoalRLFRONT = Math.floor(tempSpeed * tempDistToWaypointRL * tempFramesRLRIGHT) * 1.5;
                                            pushFRONT[currentObjective.droneIndex] = 1;
                                        }
                                        else {
                                            dotToGoalRLBACK = Math.floor(tempSpeed * tempDistToWaypointRL * tempFramesRLLEFT);
                                        }


                                        pushBACK[currentObjective.droneIndex] = 1;
                                        ////////console.PrintError("01: " + dotToGoalRLBACK + " ___ " + dotToGoalRLFRONT);
                                    }
                                }
                                else //DISTANCE TO WAYPOINT HORIZONTAL IS LOWER AND GOING SLOWER
                                {
                                    if (someOtherRLDOT >= 0) //DISTANCE TO WAYPOINT HORIZONTAL IS LOWER AND GOING SLOWER AND NEGATIVE SO NPC IS LEFT
                                    {

                                        var tempDistToWaypointRLLEFT = adjacentFB; // * 0.1 //LOWER when getting CLOSER TO waypoint. //HIGHER when getting FAR FROM waypoint.
                                        var tempDistToWaypointRLRIGHT = adjacentFBMax - adjacentFB;


                                        var tempSpeed = speedNPC * 0.1;
                                        var tempFramesRLRIGHT = framesToCoverDistanceAtSpeedFB;
                                        var tempFramesRLLEFT = framesToCoverDistanceAtSpeedFB;

                                        if (framesToCoverDistanceAtSpeedFB <= framesToCoverDistanceAtSpeedFBFrontMAX) {
                                            //tempFramesRLRIGHT = framesToCoverDistanceAtSpeedFBINVERTER - framesToCoverDistanceAtSpeedFB;
                                            dotToGoalRLFRONT = Math.floor(tempSpeed * tempDistToWaypointRLRIGHT * tempFramesRLRIGHT);
                                            dotToGoalRLBACK = Math.floor(tempSpeed * tempDistToWaypointRLLEFT * tempFramesRLLEFT) * 0.75;
                                            pushBACK[currentObjective.droneIndex] = 1;
                                        }
                                        else {
                                            dotToGoalRLFRONT = Math.floor(tempSpeed * tempDistToWaypointRLRIGHT * tempFramesRLRIGHT);
                                        }

                                        pushFRONT[currentObjective.droneIndex] = 1;

                                        ////////console.PrintError("02: " + dotToGoalRLBACK + " ___ " + dotToGoalRLFRONT);
                                    }
                                    else //DISTANCE TO WAYPOINT HORIZONTAL IS LOWER AND GOING SLOWER AND POSITIVE SO NPC IS RIGHT
                                    {
                                        var tempDistToWaypointRLLEFT = adjacentFBMax - adjacentFB; // * 0.1 //LOWER when getting CLOSER TO waypoint. //HIGHER when getting FAR FROM waypoint.
                                        var tempDistToWaypointRLRIGHT = adjacentFB;


                                        //tempDistToWaypointRL = 1 - tempDistToWaypointRL; //LOWER when getting FAR AWAY from waypoint. //HIGHER when getting CLOSER TO waypoint.

                                        var tempSpeed = speedNPC;
                                        var tempFramesRLLEFT = framesToCoverDistanceAtSpeedFB;
                                        var tempFramesRLRIGHT = framesToCoverDistanceAtSpeedFB;

                                        if (framesToCoverDistanceAtSpeedFB <= framesToCoverDistanceAtSpeedFBFrontMAX) {
                                            //tempFramesRLLEFT = framesToCoverDistanceAtSpeedFBINVERTER - framesToCoverDistanceAtSpeedFB;
                                            dotToGoalRLBACK = Math.floor(tempSpeed * tempDistToWaypointRLLEFT * tempFramesRLLEFT);
                                            dotToGoalRLFRONT = Math.floor(tempSpeed * tempDistToWaypointRLRIGHT * tempFramesRLRIGHT) * 0.75;
                                            pushFRONT[currentObjective.droneIndex] = 1;
                                        }
                                        else {
                                            dotToGoalRLBACK = Math.floor(tempSpeed * tempDistToWaypointRLLEFT * tempFramesRLLEFT);
                                        }
                                        pushBACK[currentObjective.droneIndex] = 1;

                                        ////////console.PrintError("03: " + dotToGoalRLBACK + " ___ " + dotToGoalRLFRONT);
                                    }
                                }
                            }




                            dataToReturn = SC_AI_Drone_Combat_cc_Rout_Com_5.AICombatRoutine(cData[currentObjective.droneIndex], ceData[currentObjective.droneIndex]);
                            cData[currentObjective.droneIndex] = dataToReturn.forData;
                            ceData[currentObjective.droneIndex] = dataToReturn.comData;



                            if (alignedDirectionWithPlayerDirectionDOT >= 0) {
                                if (oppositeRLDistance > 0.05 ||
                                    adjacentFB > 0.05) {
                                    if (oppositeRLDistance > 0.05) {
                                        if (alignedDirectionWithPlayerDirectionDOT >= 0)//|| alignedDirectionWithPlayerDirectionDOT <= 0.35
                                        {
                                            if (alignedDirectionWithPlayerDirectionDOT >= 0) {
                                                if (isFrontOrBack > 0) {
                                                    if (isLeftOrRight > 0) {
                                                        if (pushToTheLEFT[currentObjective.droneIndex] == 1) {
                                                            if (leftPushCounter[currentObjective.droneIndex] >= dotToGoalRLLEFT) {
                                                                SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //npc.StrafeLeft(currentObjective.nid);
                                                                leftPushCounter[currentObjective.droneIndex] = 0;
                                                            }
                                                        }
                                                        if (pushToTheRIGHT[currentObjective.droneIndex] == 1) {
                                                            if (rightPushCounter[currentObjective.droneIndex] >= dotToGoalRLRIGHT) {
                                                                SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //npc.StrafeRight(currentObjective.nid);
                                                                rightPushCounter[currentObjective.droneIndex] = 0;
                                                            }
                                                        }
                                                    }
                                                    else {
                                                        if (pushToTheRIGHT[currentObjective.droneIndex] == 1) {
                                                            if (rightPushCounter[currentObjective.droneIndex] >= dotToGoalRLRIGHT) {
                                                                SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //npc.StrafeRight(currentObjective.nid);
                                                                rightPushCounter[currentObjective.droneIndex] = 0;
                                                            }
                                                        }
                                                        if (pushToTheLEFT[currentObjective.droneIndex] == 1) {
                                                            if (leftPushCounter[currentObjective.droneIndex] >= dotToGoalRLLEFT) {
                                                                SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //npc.StrafeLeft(currentObjective.nid);
                                                                leftPushCounter[currentObjective.droneIndex] = 0;
                                                            }
                                                        }
                                                    }
                                                }
                                                else {
                                                    if (isLeftOrRight > 0) {
                                                        if (pushToTheLEFT[currentObjective.droneIndex] == 1) {
                                                            if (leftPushCounter[currentObjective.droneIndex] >= dotToGoalRLLEFT) {
                                                                SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //npc.StrafeLeft(currentObjective.nid);
                                                                leftPushCounter[currentObjective.droneIndex] = 0;
                                                            }
                                                        }
                                                        if (pushToTheRIGHT[currentObjective.droneIndex] == 1) {
                                                            if (rightPushCounter[currentObjective.droneIndex] >= dotToGoalRLRIGHT) {
                                                                SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //npc.StrafeRight(currentObjective.nid);
                                                                rightPushCounter[currentObjective.droneIndex] = 0;
                                                            }
                                                        }
                                                    }
                                                    else {
                                                        if (pushToTheRIGHT[currentObjective.droneIndex] == 1) {
                                                            if (rightPushCounter[currentObjective.droneIndex] >= dotToGoalRLRIGHT) {
                                                                SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //npc.StrafeRight(currentObjective.nid);
                                                                rightPushCounter[currentObjective.droneIndex] = 0;
                                                            }
                                                        }
                                                        if (pushToTheLEFT[currentObjective.droneIndex] == 1) {
                                                            if (leftPushCounter[currentObjective.droneIndex] >= dotToGoalRLLEFT) {
                                                                SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //npc.StrafeLeft(currentObjective.nid);
                                                                leftPushCounter[currentObjective.droneIndex] = 0;
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                            else if (alignedDirectionWithPlayerDirectionDOT <= 0.35) {
                                                //strafeToGoForward
                                            }
                                        }
                                    }
                                    if (adjacentFB > 0.05) {
                                        if (isFrontOrBack > 0) {
                                            if (isLeftOrRight > 0) {
                                                if (pushBACK[currentObjective.droneIndex] == 1) {
                                                    if (backPushCounter[currentObjective.droneIndex] >= dotToGoalRLBACK) {
                                                        SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 1, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                        //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 1, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                        //npc.GoBackward(currentObjective.nid);
                                                        backPushCounter[currentObjective.droneIndex] = 0;
                                                    }
                                                }
                                                if (pushFRONT[currentObjective.droneIndex] == 1) {
                                                    if (frontPushCounter[currentObjective.droneIndex] >= dotToGoalRLFRONT) {
                                                        SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 1, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                        //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 1, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                        //npc.GoForward(currentObjective.nid);
                                                        frontPushCounter[currentObjective.droneIndex] = 0;
                                                    }
                                                }
                                            }
                                            else {
                                                if (pushBACK[currentObjective.droneIndex] == 1) {
                                                    if (backPushCounter[currentObjective.droneIndex] >= dotToGoalRLBACK) {
                                                        SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 1, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                        //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 1, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                        //npc.GoBackward(currentObjective.nid);
                                                        backPushCounter[currentObjective.droneIndex] = 0;
                                                    }
                                                }
                                                if (pushFRONT[currentObjective.droneIndex] == 1) {
                                                    if (frontPushCounter[currentObjective.droneIndex] >= dotToGoalRLFRONT) {
                                                        SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 1, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                        //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 1, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                        //npc.GoForward(currentObjective.nid);
                                                        frontPushCounter[currentObjective.droneIndex] = 0;
                                                    }
                                                }
                                            }
                                        }
                                        else {
                                            if (isLeftOrRight > 0) {
                                                if (pushBACK[currentObjective.droneIndex] == 1) {
                                                    if (backPushCounter[currentObjective.droneIndex] >= dotToGoalRLBACK) {
                                                        SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 1, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                        //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 1, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                        //npc.GoBackward(currentObjective.nid);
                                                        backPushCounter[currentObjective.droneIndex] = 0;
                                                    }
                                                }
                                                if (pushFRONT[currentObjective.droneIndex] == 1) {
                                                    if (frontPushCounter[currentObjective.droneIndex] >= dotToGoalRLFRONT) {
                                                        SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 1, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                        //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 1, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                        //npc.GoForward(currentObjective.nid);
                                                        frontPushCounter[currentObjective.droneIndex] = 0;
                                                    }
                                                }
                                            }
                                            else {
                                                if (pushBACK[currentObjective.droneIndex] == 1) {
                                                    if (backPushCounter[currentObjective.droneIndex] >= dotToGoalRLBACK) {
                                                        SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 1, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                        //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 1, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                        //npc.GoBackward(currentObjective.nid);
                                                        backPushCounter[currentObjective.droneIndex] = 0;
                                                    }
                                                }
                                                if (pushFRONT[currentObjective.droneIndex] == 1) {
                                                    if (frontPushCounter[currentObjective.droneIndex] >= dotToGoalRLFRONT) {
                                                        SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 1, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                        //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 1, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                        //npc.GoForward(currentObjective.nid);
                                                        frontPushCounter[currentObjective.droneIndex] = 0;
                                                    }
                                                }
                                            }
                                        }

                                    }
                                }
                                else {

                                }
                            }
                            else {
                                if (oppositeRLDistance > 0.05 ||
                                    adjacentFB > 0.05) {
                                    if (oppositeRLDistance > 0.05) {
                                        if (alignedDirectionWithPlayerDirectionDOT < 0) // || alignedDirectionWithPlayerDirectionDOT >= -0.35
                                        {
                                            if (alignedDirectionWithPlayerDirectionDOT < 0) {
                                                if (isFrontOrBack > 0) {
                                                    if (isLeftOrRight > 0) {
                                                        if (pushToTheLEFT[currentObjective.droneIndex] == 1) {
                                                            if (rightPushCounter[currentObjective.droneIndex] >= dotToGoalRLLEFT) {
                                                                SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //npc.StrafeRight(currentObjective.nid);
                                                                rightPushCounter[currentObjective.droneIndex] = 0;
                                                            }

                                                        }
                                                        if (pushToTheRIGHT[currentObjective.droneIndex] == 1) {
                                                            if (leftPushCounter[currentObjective.droneIndex] >= dotToGoalRLRIGHT) {
                                                                SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //npc.StrafeLeft(currentObjective.nid);
                                                                leftPushCounter[currentObjective.droneIndex] = 0;
                                                            }
                                                        }
                                                    }
                                                    else {
                                                        if (pushToTheLEFT[currentObjective.droneIndex] == 1) {
                                                            if (rightPushCounter[currentObjective.droneIndex] >= dotToGoalRLLEFT) {
                                                                SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //npc.StrafeRight(currentObjective.nid);
                                                                rightPushCounter[currentObjective.droneIndex] = 0;
                                                            }

                                                        }
                                                        if (pushToTheRIGHT[currentObjective.droneIndex] == 1) {
                                                            if (leftPushCounter[currentObjective.droneIndex] >= dotToGoalRLRIGHT) {
                                                                SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //npc.StrafeLeft(currentObjective.nid);
                                                                leftPushCounter[currentObjective.droneIndex] = 0;
                                                            }
                                                        }
                                                    }
                                                }
                                                else {
                                                    if (isLeftOrRight > 0) {
                                                        if (pushToTheLEFT[currentObjective.droneIndex] == 1) {
                                                            if (rightPushCounter[currentObjective.droneIndex] >= dotToGoalRLLEFT) {
                                                                SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //npc.StrafeRight(currentObjective.nid);
                                                                rightPushCounter[currentObjective.droneIndex] = 0;
                                                            }

                                                        }
                                                        if (pushToTheRIGHT[currentObjective.droneIndex] == 1) {
                                                            if (leftPushCounter[currentObjective.droneIndex] >= dotToGoalRLRIGHT) {
                                                                SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //npc.StrafeLeft(currentObjective.nid);
                                                                leftPushCounter[currentObjective.droneIndex] = 0;
                                                            }
                                                        }
                                                    }
                                                    else {
                                                        if (pushToTheLEFT[currentObjective.droneIndex] == 1) {
                                                            if (rightPushCounter[currentObjective.droneIndex] >= dotToGoalRLLEFT) {
                                                                SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //npc.StrafeRight(currentObjective.nid);
                                                                rightPushCounter[currentObjective.droneIndex] = 0;
                                                            }

                                                        }
                                                        if (pushToTheRIGHT[currentObjective.droneIndex] == 1) {
                                                            if (leftPushCounter[currentObjective.droneIndex] >= dotToGoalRLRIGHT) {

                                                                SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 2, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                                                //npc.StrafeLeft(currentObjective.nid);
                                                                leftPushCounter[currentObjective.droneIndex] = 0;
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                            else if (alignedDirectionWithPlayerDirectionDOT >= -0.35) {
                                                //strafeToGoForward.
                                            }
                                        }
                                    }
                                }
                                else {


                                }
                            }


                        






                            cData.cFWPAlt = pointInFrontOfWaypoint;
                            cData.cFWPAltDot = alignedDirectionWithPlayerDirectionRIGHTDOT;


                            if (activateThisFramePathfindCounterTWO > activateThisFramePathfindCounterTWOMax) {

                                if (alignedDirectionWithPlayerDirectionRIGHTDOT <= -0.05 || alignedDirectionWithPlayerDirectionRIGHTDOT >= 0.05) {
                                    SC_AI_Drone_Combat_cc_Rout_For_Prot_5.AIMoveProtocol(cData, currentObjective, 0, 0, currentFormationWaypoint, _currentWaypointPathDirection);
                                }

                                activateThisFramePathfindCounterTWO = 0;
                            }

                            pushToTheRIGHT[currentObjective.droneIndex] = 0;
                            pushToTheLEFT[currentObjective.droneIndex] = 0;
                            pushBACK[currentObjective.droneIndex] = 0;
                            pushFRONT[currentObjective.droneIndex] = 0;

                            lastDistanceToWaypoint[currentObjective.droneIndex] = distToWaypoint;
                            lastDistanceToWaypointAdjacentFB[currentObjective.droneIndex] = adjacentFB;
                            lastDistanceToWaypointoppositeRL[currentObjective.droneIndex] = oppositeRLDistance;


                            lastSpeedNPC[currentObjective.droneIndex] = speedNPC;
                            lastSpeedPlayer[currentObjective.droneIndex] = speedPlayer;

                            lastFrontPushCounter[currentObjective.droneIndex] = frontPushCounter[currentObjective.droneIndex];
                            lastBackPushCounter[currentObjective.droneIndex] = backPushCounter[currentObjective.droneIndex];
                            lastLeftPushCounter[currentObjective.droneIndex] = leftPushCounter[currentObjective.droneIndex];
                            lastRightPushCounter[currentObjective.droneIndex] = rightPushCounter[currentObjective.droneIndex];

                            lastSomeOtherRLDOT[currentObjective.droneIndex] = someOtherRLDOT;
                            lastSomeOtherFBDOT[currentObjective.droneIndex] = someOtherFBDOT;

                            frontPushCounter[currentObjective.droneIndex]++;
                            backPushCounter[currentObjective.droneIndex]++;
                            leftPushCounter[currentObjective.droneIndex]++;
                            rightPushCounter[currentObjective.droneIndex]++;

                            lastdistToWaypoint = distToWaypoint;
                            lastadjacentFB = adjacentFB;


                            /*dataToReturn = SC_AI_Drone_Combat_cc_Rout_Com_5.AICombatRoutine(cData[currentObjective.droneIndex], ceData[currentObjective.droneIndex]);
                            cData[currentObjective.droneIndex] = dataToReturn.forData;
                            ceData[currentObjective.droneIndex] = dataToReturn.comData;*/
                            

                         

      

                        //}
                        //else if (distToWaypoint > 25)
                        //{
                        //    npc.StickToPoint(currentObjective.nid, currentFormationWaypoint.x, currentFormationWaypoint.y, 0);





                        //}
                        /*if (alignedDirectionWithPlayerDirectionRIGHTDOT >= 0.05) {

                            npc.RotRight(currentObjective.nid);
                        }
                        else if (alignedDirectionWithPlayerDirectionRIGHTDOT <= -0.05) {
                            npc.RotLeft(currentObjective.nid);

                        }
                        */
                        activateThisFramePathfindCounter = 0;
                    }
                    else //cData[currentObjective.droneIndex].swtchForPathfind == 0 or else
                    {
                        /*if (distToWaypoint >= minimumDistWaypoint && distToWaypoint <= 25) {
                            //THE SECOND PATHFIND IS STARTING FROM THE CALL TO THE FUNCTION BELOW. TO CHANGE IN THE FUTURE. I WANTED A PATHFIND THAT WORKED FOR THE MOMENT, AND I WILL CLEAN THIS AND ORGANIZE THIS LATER. FIXING BUGS FIRST.
                            dataToReturn = SC_AI_Drone_Combat_cc_Rout_Com_5.AICombatRoutine(cData[currentObjective.droneIndex], ceData[currentObjective.droneIndex]);
                            //THE SECOND PATHFIND IS STARTING FROM THE CALL TO THE FUNCTION BELOW. TO CHANGE IN THE FUTURE. I WANTED A PATHFIND THAT WORKED FOR THE MOMENT, AND I WILL CLEAN THIS AND ORGANIZE THIS LATER. FIXING BUGS FIRST.

                            cData[currentObjective.droneIndex] = dataToReturn.forData;
                            ceData[currentObjective.droneIndex] = dataToReturn.comData;

                            ////////console.PrintError("swtchForPathfind " + cData[currentObjective.droneIndex].swtchForPathfind);
                        }
                        else {
                            npc.StickToPoint(currentObjective.nid, currentFormationWaypoint.x, currentFormationWaypoint.y, 0);
                        }*/
                        dataToReturn = SC_AI_Drone_Combat_cc_Rout_Com_5.AICombatRoutine(cData[currentObjective.droneIndex], ceData[currentObjective.droneIndex]);
                        //THE SECOND PATHFIND IS STARTING FROM THE CALL TO THE FUNCTION BELOW. TO CHANGE IN THE FUTURE. I WANTED A PATHFIND THAT WORKED FOR THE MOMENT, AND I WILL CLEAN THIS AND ORGANIZE THIS LATER. FIXING BUGS FIRST.

                        cData[currentObjective.droneIndex] = dataToReturn.forData;
                        ceData[currentObjective.droneIndex] = dataToReturn.comData;




                        if (activateThisFramePathfindCounterTWO > activateThisFramePathfindCounterTWOMax)
                        {

                            

                        /*if (cData[currentObjective.droneIndex].objt.command == 0 && ceData.ts == -1) {
                            cData[currentObjective.droneIndex].mSwtc = 1;
                            cData[currentObjective.droneIndex] = SC_AI_Drone_Combat_cc_Rout_For_SFWP_5.npcStayInFormation(cData[currentObjective.droneIndex]);
                        }
                        else if (cData[currentObjective.droneIndex].objt.command == 1 && ceData.ts == -1) {
                            ////////console.PrintError("test");
                            if (cData[currentObjective.droneIndex].mSwtc == 0) {
                                npc.InstantStop(cData[currentObjective.droneIndex].objt.nid);
                                cData[currentObjective.droneIndex].mSwtc = 1;
                            }

                            cData[currentObjective.droneIndex] = SC_AI_Drone_Combat_cc_Rout_For_SFWPP_5.npcPathfindToWaypoint(cData[currentObjective.droneIndex]);


                            //ADD SCRIPTS FOR THE PERCEPTRON, TO GET THE NEXT NODE OUT OF THE ABOVE FUNCTION, REMOVE any sorts or form of npc.InstantStop or npc.Stop or npc.StickToPoint. create scripts and have the perceptron start a new script hierarchy otherwise the SC_PathFind_cc_5.js will be cluttered and bigger than 20-30 kb.
                        }*/

                            activateThisFramePathfindCounterTWO = 0;
                        }

                    }



                    activateThisFramePathfindCounter++;
                    activateThisFramePathfindCounterTWO++;



                    cData[currentObjective.droneIndex].veloLF = rotationDirectionDOTVELO;
                    cData[currentObjective.droneIndex].RLDOTStrafeLast = RLDOTStrafe;




      


                    //ADDED FROM ninesMODAIGen2PathGen2StationsPathNMove.mpk
                    //ADDED FROM ninesMODAIGen2PathGen2StationsPathNMove.mpk
                    //ADDED FROM ninesMODAIGen2PathGen2StationsPathNMove.mpk
                    cData.cLFWP = cData.cFWP;
                    cData.cLFWPD = cData.cFWPD;
                    //cData.ltPEarlierFrame = cData.cFWP;
                    if (cData.dog != null) {
                        cData.ldog == cData.dog;
                    }
                    //ADDED FROM ninesMODAIGen2PathGen2StationsPathNMove.mpk
                    //ADDED FROM ninesMODAIGen2PathGen2StationsPathNMove.mpk
                    //ADDED FROM ninesMODAIGen2PathGen2StationsPathNMove.mpk

                }
                else if (nData != null && pData == null) {
                    if (ceData[currentObjective.droneIndex].pLData != null) {
                        //////console.PrintError("check if player is docked? or destroyed");
                    }
                }
            }
            else if (currentObjective.speedSwitch == 0)
            {
                SC_AI_Drone_Combat_cc_Reset_Speed_5.AICombatResetSpeed(currentObjective);
            }

            if (currentObjective.swtchForPathfind == 1)
            {

                /*SC_Perceptron.create_AI_perceptron(3, 0.01);

                for (var i = 0; i < lengthOfArray; i++) {
                    arrayOfDataRL.push({ dir: -1, error: -1 });
                    arrayOfDataFR.push({ dir: -1, error: -1 });
                }

                for (var i = 0; i < lengthOfTrainingArray; i++) {
                    //var inputs = [];
                    trainingLR.push(null); //{x:-1,y:-1,a:-1,input:inputs}
                    trainingFR.push(null);
                }*/

                //perc = new Perceptron(3, (float)0.01);
                //////console.PrintError("perceptron created");
            }
        }
        else {
            //arrays for machine learning
            for (var i = 0; i < 5; i++) {
                lastDistanceToWaypoint.push(0);
                lastDistanceToWaypointAdjacentFB.push(0);
                lastDistanceToWaypointoppositeRL.push(0);


                lastSpeedNPC.push(0);
                lastSpeedPlayer.push(0);

                frontPushCounter.push(0);
                backPushCounter.push(0);
                leftPushCounter.push(0);
                rightPushCounter.push(0);

                lastFrontPushCounter.push(0);
                lastBackPushCounter.push(0);
                lastLeftPushCounter.push(0);
                lastRightPushCounter.push(0);

                lastSomeOtherRLDOT.push(0);
                lastSomeOtherFBDOT.push(0);
                lastRLDOTStrafe.push(0);

                pushToTheLEFT.push(0);
                pushToTheRIGHT.push(0);
                pushBACK.push(0);
                pushFRONT.push(0);

                formationSwitches.push(1);
                mainSwitches.push(0);

                selectedTarget.push(-1);
            }





            if (cData.length < currentObjective.droneIndex || cData[0] == null) {
                cDat =
                    {
                        nData: null,
                        pData: null,
                        pLData: null,
                        objt: currentObjective,
                        cFWP: { x: null, y: null },
                        cFWPD: 0,
                        cLFWP: { x: null, y: null },
                        cLFWPD: 0,
                        cFWPAlt: { x: null, y: null },
                        cFWPAltDot: 0,


                        fSwtc: 1,
                        mSwtc: 0,
                        lsP: { x: null, y: null },
                        ltP: { x: null, y: null },
                        liP: { x: null, y: null },
                        gliP: { x: null, y: null },

                        dog: [],
                        log: [],
                        pfc: -2,
                        cdSwtc: 0,
                        cdtSwtc: 0,
                        loRLDOT: 0,
                        stopS: 1,
                        stopSC: 0,
                        stopSCM: 5,
                        noFWP: 0,
                        swtchForPathfind: 0,
                        veloLF: { x: null, y: null },
                        oppositeRLLast: 0,
                        adjacentRLLast: 0,
                    RLDOTStrafeLast: 0,
                    footStepsCounter: 0,
                    footStepsCounterMAX: 3,
                    hasUnwalkableTiles: -1,
                    hasUnwalkableTilesArray: [],
                    lLSwtc: 0,
                    lLSwtcCounter: 0,

                    };
                cData.push(cDat);
            }
            else if (cData[currentObjective.droneIndex] == null) {
                cDat =
                    {
                        nData: null,
                        pData: null,
                        pLData: null,
                        objt: currentObjective,
                        cFWP: { x: null, y: null },
                        cLFWP: { x: null, y: null },
                        cFWPD: 0,
                        cFWPAlt: { x: null, y: null },
                        cFWPAltDot: 0,

                        fSwtc: 1,
                        mSwtc: 0,
                        lsP: { x: null, y: null },
                        ltP: { x: null, y: null },
                        liP: { x: null, y: null },
                        gliP: { x: null, y: null },

                        dog: [],
                        log: [],
                        pfc: -2,
                        cdSwtc: 0,
                        cdtSwtc: 0,
                        loRLDOT: 0,
                        stopS: 1,
                        stopSC: 0,
                        stopSCM: 5,
                        noFWP: 0,
                        stRot: null,
                        stCoord: null,
                        hasUnwalkableTiles: -1,
                        hasUnwalkableTilesArray: [],
                        lLSwtc: 0,
                        lLSwtcCounter: 0,
                    };
                cData[currentObjective.droneIndex] = cDat;
            }


            if (ceData.length < currentObjective.droneIndex || ceData[0] == null) {
                ceDat =
                    {
                        nData: null,
                        pData: null,
                        eData: null,
                        arrF: arrayOfFriendlies,
                        objt: currentObjective,
                        ts: -1,
                        lLSwtc: 0,

                        hasNA: 0,

                        hasStP: 0,
                        hasStO: 0,

                        tec: 0,
                        tecC: 0,
                        mReset: 0,
                        evad: 0,
                        evadC: 0,

                        tec2: 0,
                        tec2C: 0,
                        loRLDOT: 0,
                        tec3: 0,
                        tec3C: 0,
                        dLStrc: 0,
                        eneL: 0,
                        eToP: 0,
                        dToE: 0,
                        fSwtc: 0,

                        eneLSh: 0,
                        npcLD: 0,

                        stopS: null,
                    stopSC: null,
                    hasUnwalkableTiles: -1,
                    hasUnwalkableTilesArray: []
                    };
                ceData.push(ceDat);
            }
            else if (ceData[currentObjective.droneIndex] == null) {
                ceDat =
                    {
                        nData: null,
                        pData: null,
                        eData: null,
                        arrF: arrayOfFriendlies,
                        objt: currentObjective,
                        ts: -1,
                        lLSwtc: 0,

                        hasNA: 0,

                        hasStP: 0,
                        hasStO: 0,

                        tec: 0,
                        tecC: 0,
                        mReset: 0,
                        evad: 0,
                        evadC: 0,

                        tec2: 0,
                        tec2C: 0,
                        loRLDOT: 0,
                        tec3: 0,
                        tec3C: 0,
                        dLStrc: 0,
                        eneL: 0,
                        eToP: 0,
                        dToE: 0,
                        fSwtc: 0,

                        eneLSh: 0,
                        npcLD: 0,

                        stopS: null,
                    stopSC: null,
                    hasUnwalkableTiles: -1,
                    hasUnwalkableTilesArray: []
                    };
                ceData[currentObjective.droneIndex] = ceDat;
            }
        }
    }
};













/*if (guess > 0)
{
    if (dotProdGoal > lastFrameDot)
    {
        arrayOfDataRL[indexer].error = 0;
        arrayOfDataRL[indexer].dir = 0;
    }
    else
    {
        arrayOfDataRL[indexer].error = 1;
        arrayOfDataRL[indexer].dir = 1;
    }
}
else
{
    if (dotProdGoal > lastFrameDot)
    {
        arrayOfDataRL[indexer].error = 0;
        arrayOfDataRL[indexer].dir = 1;
    }
    else
    {
        arrayOfDataRL[indexer].error = 1;
        arrayOfDataRL[indexer].dir = 0;
    }
}*/