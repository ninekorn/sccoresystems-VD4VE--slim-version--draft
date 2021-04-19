//TODO
//1. code how many frames needed to stop at current speed




using(console);
using(npc);
using(ship);
using(storage);

include(SC_Utilities.js);

include(SC_AI_Drone_Get_nData.js);
include(SC_AI_Drone_Get_pData.js);

include(SC_AI_Drone_Combat_cc_Rout_For_FWP_4.js);
include(SC_AI_Drone_Combat_cc_Rout_Com_4.js);
include(SC_AI_Drone_Combat_cc_Reset_Speed_4.js);
include(SC_AI_Drone_Combat_cc_Friendlies_4.js);

include(SC_Perceptron.js);




//OLD VARIABLES may 2019
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
var someTester = 1;
var perceptron;
var arrayOfDataRL = [];
var arrayOfErrorDataRL = [];
var arrayOfDataFR = [];
var arrayOfErrorDataFR = [];
var lengthOfArray = 10; //200
var finalDotR = 0;
var initialDotGoal = 0;
var trainingLR = [];
var trainingFR = [];
var lengthOfTrainingArray = 3; //500
//var count = 0;
var lastWaypointPos;
var lastFrameDot = 0;
var lastDistanceToWaypoint = [];
var lastDistanceToWaypointAdjacentFB = [];
var lastDistanceToWaypointOppositeRL = [];
var distToWaypoint = [];
var adjacentFB = [];
var oppositeRL = [];
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
var lastSomeOtherRLDOTTWO = [];
var someOtherRLDOT = [];
var someOtherFBDOT = [];
var pushLEFT = [];
var pushRIGHT = [];
var pushBACK = [];
var pushFRONT = [];
var formationSwitches = [];
var selectedTarget = [];
var mainSwitches = [];
var player_id;
var playerName;
var sys_idNPC;
var sys_idPlayer;
var npcCoord;
var nextCommandToDispatch = null;
var mainFrameCounter = 0;
var currentObjective;
var initSystems = 1;
var lockdist = 25;
var getawaydist = 0;
var lockSwitches = [];
var lockBattlePositionSwitches = [];
var lockBattlePositionSpeedValueSwitch = [];
var lockLastBattleDOTDirectionValueSwitch = [];
//OLD VARIABLES may 2019






var maxBrollofPercInst = 3;
var SC_Angle_Divider = 10;
var SC_anglesNumber = 360;
var SC_anglesQuarterNumber = 360 * 10; //360 * SC_Angle_Divider
//3x number of valves to 
var inputsNumber = 3; //send only 3 random position to the Perceptron Training
var weightsNumber = inputsNumber;
var saveCurrentWeightForTheCurrentAngleInsideOfUnitsOf360 = [];//float[][] saveCurrentWeightForTheCurrentAngleInsideOfUnitsOf360 = new float[SC_anglesQuarterNumber][];
var perceptronLearningRate = 0.00012321;
//Perceptron.Perceptron perc;
var weights = [];//float[] weights;
var xmin, xmax, ymin, ymax;
var training = [];//Trainer[] training = new Trainer[inputsNumber];
var count = 0;
var _guessedCorrectRight = 0;
var _guessedCorrectLeft = 0;
var _dotGoal;
var swtch = 0;
var answer = -1;
var lastAngle = -999;
var bullseyepos = { x: 0, y: 0 };
var dronePos = { x: 0, y: 0 };

var someError = 0;
var someDir = 0;

var guessedCorrect = 0;
var guessedWrong = 0;

var turnRight = 0;
var turnLeft = 0;

var SC_AI_Drone_Combat_cc_Rout_For_Prot_4 =
{
    AIMoveProtocol: function (cData, currentObjective, swtchwaypointtype, invertdir, _currentFormationWaypoint, _currentWaypointPathDirection) {
        nData = cData[currentObjective.droneIndex].nData;

        //pData.pCoord = _currentFormationWaypoint;
        //pData.pForward = _currentWaypointPathDirection;//cData[currentObjective.droneIndex].pData;


        if (swtch == 0) {
            SC_Perceptron.create_AI_perceptron(maxBrollofPercInst, perceptronLearningRate);
            //console.PrintError("swtch == 0");
            weights = SC_Perceptron.GetWeights();

            if (weights != null) {
                if (weights.length > 0) {
                    for (var i = 0; i < SC_anglesNumber * SC_Angle_Divider; i++) {
                        saveCurrentWeightForTheCurrentAngleInsideOfUnitsOf360.push(weights);
                    }
                    swtch = 1;
                }
            }
        }

        if (swtch == 1) {
            _guessedCorrectRight = 0;
            _guessedCorrectLeft = 0;

            dronePos = nData.nCoord;//new Vector2(hardpointpivot.transform.position.x, hardpointpivot.transform.position.y);
            //var currentdronePos = nData.nCoord;
            //var currentplayerPos = pData.pCoord;


            var angle = (ship.GetRotation(cData[currentObjective.droneIndex].objt.nid));//SC_Utilities.Clamp0360
            angle = angle * (180.0 / Math.PI);
            angle = SC_Utilities.ClampValue(angle, 0, SC_anglesQuarterNumber);

            var angleRounded = Math.floor(angle);
            var currentDiff = (angle - angleRounded);
            var currentQuarterRoundedAngle = Math.floor((currentDiff * SC_Angle_Divider) / SC_Angle_Divider); //Mathf.Round(angle * 4) / 4;
            currentQuarterRoundedAngle *= 100;

            currentQuarterRoundedAngle = (angle * SC_Angle_Divider);

            currentQuarterRoundedAngle = SC_Utilities.ClampValue(currentQuarterRoundedAngle, 0, SC_anglesQuarterNumber);


            weights = SC_Perceptron.GetWeights();

            var flooredAngle = Math.floor(currentQuarterRoundedAngle);

            //console.PrintError(flooredAngle);

            if (flooredAngle < saveCurrentWeightForTheCurrentAngleInsideOfUnitsOf360.length) {
                SC_Perceptron.SetWeights(saveCurrentWeightForTheCurrentAngleInsideOfUnitsOf360[Math.round(currentQuarterRoundedAngle)]);

                var pointForwardDirNPCX = (1 * Math.cos(Math.PI * angle / 180.0)) + dronePos.x; // * Math.PI / 180
                var pointForwardDirNPCY = (1 * Math.sin(Math.PI * angle / 180.0)) + dronePos.y;

                var dirForwardNPC = { x: pointForwardDirNPCX - dronePos.x, y: pointForwardDirNPCY - dronePos.y };//(pointForwardDirNPCX - dronePos.x, pointForwardDirNPCY - dronePos.y);

                var dirRightNPC = { x: pointForwardDirNPCY - dronePos.y, y: -1 * (pointForwardDirNPCX - dronePos.x) };//(pointForwardDirNPCY - dronePos.y, -1 * (pointForwardDirNPCX - dronePos.x));
                var dirNPCToPlayer = { x: bullseyepos.x - dronePos.x, y: bullseyepos.y - dronePos.y };//(bullseyepos.x - dronePos.x, bullseyepos.y - dronePos.y);

                var someOtherMAG = Math.sqrt((dirNPCToPlayer.x * dirNPCToPlayer.x) + (dirNPCToPlayer.y * dirNPCToPlayer.y));
                dirNPCToPlayer.x /= someOtherMAG;
                dirNPCToPlayer.y /= someOtherMAG;




















                if (swtchwaypointtype == 0) {
                    var dirbulletprimerright = { x: nData.nForward.y, y: -nData.nForward.x }; // transform.right from the transform.forward is {x: y, y:-x}
                    var dirbulletprimerforward = nData.nForward;

                    var dirprimertobullseyeX = currentFormationWaypoint.x - nData.nCoord.x;
                    var dirprimertobullseyeY = currentFormationWaypoint.y - nData.nCoord.y;
                    var dirprimertobullseye = { x: dirprimertobullseyeX, y: dirprimertobullseyeY };
                    var lengther = Math.sqrt((dirprimertobullseye.x * dirprimertobullseye.x) + (dirprimertobullseye.y * dirprimertobullseye.y));
                    dirprimertobullseye.x /= lengther;
                    dirprimertobullseye.y /= lengther;

                    //_dotGoal = SC_Utilities.Dot(dirbulletprimerright.x, dirbulletprimerright.y, dirprimertobullseye.x, dirprimertobullseye.y);
                    _dotGoal = SC_Utilities.Dot(nData.nForward.y, -nData.nForward.x, pData.pForward.x, pData.pForward.y);


                    if (_dotGoal >= 0.001) {
                        //Debug.Log("player is right" + _dotGoal);
                        answer = 1;
                    }
                    else if (_dotGoal < -0.001) {
                        //Debug.Log("player is left" + _dotGoal);
                        answer = -1;
                    }
                }
                else if (swtchwaypointtype == 1) {
                    var dirbulletprimerright = { x: nData.nForward.x, y: nData.nForward.y }; // transform.right from the transform.forward is {x: y, y:-x}
                    var dirbulletprimerforward = nData.nForward;

                    var dirprimertobullseyeX = currentFormationWaypoint.x - nData.nCoord.x;
                    var dirprimertobullseyeY = currentFormationWaypoint.y - nData.nCoord.y;
                    var dirprimertobullseye = { x: dirprimertobullseyeX, y: dirprimertobullseyeY };

                    var lengther = Math.sqrt((dirprimertobullseye.x * dirprimertobullseye.x) + (dirprimertobullseye.y * dirprimertobullseye.y));
                    dirprimertobullseye.x /= lengther;
                    dirprimertobullseye.y /= lengther;

                    _dotGoal = SC_Utilities.Dot(dirbulletprimerright.x, dirbulletprimerright.y, dirprimertobullseye.x, dirprimertobullseye.y);

                    if (_dotGoal >= 0.001) {
                        //Debug.Log("player is right" + _dotGoal);
                        answer = 1;
                    }
                    else if (_dotGoal < -0.001) {
                        //Debug.Log("player is left" + _dotGoal);
                        answer = -1;
                    }
                }
                else if (swtchwaypointtype == 2) {
                    var dirbulletprimerright = { x: nData.nForward.y, y: -nData.nForward.x }; // transform.right from the transform.forward is {x: y, y:-x}
                    var dirbulletprimerforward = nData.nForward;

                    var dirprimertobullseyeX = currentFormationWaypoint.x - nData.nCoord.x;
                    var dirprimertobullseyeY = currentFormationWaypoint.y - nData.nCoord.y;
                    var dirprimertobullseye = { x: dirprimertobullseyeX, y: dirprimertobullseyeY };
                    var lengther = Math.sqrt((dirprimertobullseye.x * dirprimertobullseye.x) + (dirprimertobullseye.y * dirprimertobullseye.y));
                    dirprimertobullseye.x /= lengther;
                    dirprimertobullseye.y /= lengther;

                    _dotGoal = SC_Utilities.Dot(dirbulletprimerright.x, dirbulletprimerright.y, dirprimertobullseye.x, dirprimertobullseye.y);

                    if (_dotGoal >= 0.001) {
                        //Debug.Log("player is right" + _dotGoal);
                        answer = 1;
                    }
                    else if (_dotGoal < -0.001) {
                        //Debug.Log("player is left" + _dotGoal);
                        answer = -1;
                    }
                }

                
                trainingLR = [];
                for (var i = 0; i < 3; i++) {
                    //var angleInRadians = random.Next(360) / (2 * Math.PI);
                    var angleInRadians = (Math.floor(Math.random() * (360 - 0) + 0)) / (2 * Math.PI);
                    var x = Math.round(0.001 * Math.cos(angleInRadians) + dronePos.x);
                    var y = Math.round(0.001 * Math.sin(angleInRadians) + dronePos.y);

                    var dirToGuessedWaypoint = { x: x - dronePos.x, y: y - dronePos.y };

                    var x = currentFormationWaypoint.x;
                    var y = currentFormationWaypoint.y;

                    var inputs = [];
                    inputs.push(x);
                    inputs.push(y);
                    inputs.push(1);
                  

                    var someData = { x: x, y: y, a: answer, input: inputs };
                    trainingLR.push(someData);
                }

















                /*trainingLR = [];
                var compassPos = nData.nCoord;//new Vector2(nData.nCoord.x, nData.nCoord.y);

                for (var i = 0; i < trainingLR.length; i++) {
                    var angleInRadians = random.Next(360) / (2 * Math.PI);

                    // randomly getting a point at the location of the compass
                    var x = (0.001 * Math.cos(angleInRadians) + compassPos.x);
                    var y = (0.001 * Math.sin(angleInRadians) + compassPos.y);

                    //trainingLR[i] = new Trainer(weightsNumber, x, y, answer);

                    var inputs = [];
                    inputs.push(x);
                    inputs.push(y);
                    inputs.push(1);

                    var someData = { x: x, y: y, a: answer, input: inputs };
                    trainingLR.push(someData);
                    //perc.Train( trainingLR[i].inputs,  trainingLR[i].answer);
                }

                for (var i = 0; i < trainingLR.length; i++) {
                    SC_Perceptron.train_perceptron(trainingLR[i].inputs, trainingLR[i].answer);
                }*/



                /*if (swtchwaypointtype == 0) {
                    trainingLR = [];
                    compassPos = { x: nData.nCoord.x, y: nData.nCoord.y };//new Vector2(compasspivot.transform.position.x, compasspivot.transform.position.y);

                    for (var i = 0; i < trainingLR.length; i++) {
                        var angleInRadians = random.Next(360) / (2 * Math.PI);

                        // randomly getting a point at the location of the compass
                        var x = (0.001 * Math.cos(angleInRadians) + compassPos.x);
                        var y = (0.001 * Math.sin(angleInRadians) + compassPos.y);

                        //trainingLR[i] = new Trainer(weightsNumber, x, y, answer);
                        //perc.Train( trainingLR[i].inputs,  trainingLR[i].answer);
                     
                        var inputs = [];
                        inputs.push(x);
                        inputs.push(y);
                        inputs.push(1);

                        var someData = { x: x, y: y, a: answer, input: inputs };
                        trainingLR.push(someData);
                    }

                    for (var i = 0; i < trainingLR.length; i++) {
                        SC_Perceptron.train_perceptron(trainingLR[i].inputs, trainingLR[i].answer);
                    }
                }
              
                else if (swtchwaypointtype == 1) {
                    trainingLR = [];
                    compassPos = { x: nData.nCoord.x, y: nData.nCoord.y };
                    //compassPos = new Vector2(-compasspivot.transform.position.z, compasspivot.transform.position.x);

                    for (var i = 0; i < trainingLR.length; i++) {
                        var angleInRadians = random.Next(360) / (2 * Math.PI);

                        // randomly getting a point at the location of the compass
                        var x = (0.001 * Math.cos(angleInRadians) + compassPos.x);
                        var y = (0.001 * Math.sin(angleInRadians) + compassPos.y);

                        //trainingLR[i] = new Trainer(weightsNumber, x, y, answer);
                        //perc.Train( trainingLR[i].inputs,  trainingLR[i].answer);

                        var inputs = [];
                        inputs.push(x);
                        inputs.push(y);
                        inputs.push(1);

                        var someData = { x: x, y: y, a: answer, input: inputs };
                        trainingLR.push(someData);
                    }
                    for (var i = 0; i < trainingLR.length; i++) {
                        SC_Perceptron.train_perceptron(trainingLR[i].inputs, trainingLR[i].answer);
                    }
                }
                else if (swtchwaypointtype == 2) {
                    trainingLR = [];
                    compassPos = new Vector2(compasspivot.transform.position.z, compasspivot.transform.position.y);

                    for (var i = 0; i < trainingLR.length; i++) {
                        var angleInRadians = random.Next(360) / (2 * Math.PI);

                        // randomly getting a point at the location of the compass
                        var x = (0.001 * Math.cos(angleInRadians) + compassPos.x);
                        var y = (0.001 * Math.sin(angleInRadians) + compassPos.y);

                        //trainingLR[i] = new Trainer(weightsNumber, x, y, answer);
                        //perc.Train( trainingLR[i].inputs,  trainingLR[i].answer);

                        var inputs = [];
                        inputs.push(x);
                        inputs.push(y);
                        inputs.push(1);

                        var someData = { x: x, y: y, a: answer, input: inputs };
                        trainingLR.push(someData);
                    }
                    for (var i = 0; i < trainingLR.length; i++) {
                        SC_Perceptron.train_perceptron(trainingLR[i].inputs, trainingLR[i].answer);
                    }
                }*/








                guessedCorrect = 0;
                guessedWrong = 0;

                turnRight = 0;
                turnLeft = 0;

                for (var i = 0; i < trainingLR.Length; i++) {
                    var guess = SC_Perceptron.Guess(trainingLR[i].inputs);
                    //Vector2 neededPos = new Vector2( trainingLR[i].inputs[0],  trainingLR[i].inputs[1]);

                    if (trainingLR[i].answer == 1) {
                        turnRight++;
                    }
                    else if (trainingLR[i].answer == -1) {
                        turnLeft++;
                    }

                    if (guess >= 0) {

                        guessedCorrect++;
                    }
                    else {
                        guessedWrong++;
                    }
                }


                var errormargin = 5;
                var randguess = 0;


                if (guessedCorrect >= (trainingLR.Length * 0.5) - errormargin || // if the guess is higher than half of training.length
                    guessedWrong >= (trainingLR.Length * 0.5) - errormargin ||
                    guessedCorrect <= (trainingLR.Length * 0.5) + errormargin ||
                    guessedWrong <= (trainingLR.Length * 0.5) + errormargin) {
                    if (turnRight >= (trainingLR.Length * 0.5) - errormargin ||
                        turnLeft >= (trainingLR.Length * 0.5) - errormargin ||
                        turnRight <= (trainingLR.Length * 0.5) + errormargin ||
                        turnLeft <= (trainingLR.Length * 0.5) + errormargin) {
                        if (turnRight > turnLeft) {
                            _guessedCorrectRight++;
                        }
                        else if (turnRight < turnLeft) {
                            _guessedCorrectLeft++;
                        }
                        else {
                            randguess = (Math.floor(SC_Utilities.getSomeRandNumThousandDecimal(0, 2, 0))); // random value between 0 and 1

                            if (randguess == 0) {
                                _guessedCorrectRight++;
                            }
                            else {
                                _guessedCorrectLeft++;
                            }
                            //Debug.Log("Data is too similar");
                        }
                    }
                    else {
                        randguess = (Math.floor(SC_Utilities.getSomeRandNumThousandDecimal(0, 2, 0))); // random value between 0 and 1

                        if (randguess == 0) {
                            _guessedCorrectRight++;
                        }
                        else {
                            _guessedCorrectLeft++;
                        }
                        //Debug.Log("Data is too similar");
                    }
                }
                else {
                    randguess = (Math.floor(SC_Utilities.getSomeRandNumThousandDecimal(0, 2, 0))); // random value between 0 and 1

                    if (randguess == 0) {
                        _guessedCorrectRight++;
                    }
                    else {
                        _guessedCorrectLeft++;
                    }
                    //Debug.Log("Data is too similar 0");
                }



                /*if (count < trainingLR.length)
                {
                   
                }
                else
                {
                    console.PrintError("out of range0: " + count);
                }*/



                /*if (weights != null) {
                    if (weights.length > 0) {
                        console.PrintError("weights.length > 0");
                    }
                    else {
                        console.PrintError("weights null00");
                        swtch = 0;
                        return;
                    }
                }
                else {
                    console.PrintError("weights null01");
                    swtch = 0;
                    return;
                }*/


                /*
                for (var i = 0; i < trainingLR.length; i++) {
                    if (trainingLR != null) {
                        if (trainingLR[i] != null) {
                            if (trainingLR[i].input != null) {
                                SC_Perceptron.train_perceptron(trainingLR[i].input, trainingLR[i].a);
                            }
                            else {
                                console.PrintError("null02");
                            }
                        }
                        else {
                            console.PrintError("null01");
                        }
                    }
                    else {
                        console.PrintError("null00");
                    }
                }


          

                //count = (count + 1) % trainingLR.length;

                if (weights != null) {
                    if (weights.length > 0) {
                        if (weights[0] != null) {

                            if (SC_Perceptron.weights != null) {
                                var guessedCorrect = 0;
                                var guessedWrong = 0;

                                var turnRight = 0;
                                var turnLeft = 0;

                                var guess = -1;

                                for (var i = 0; i < trainingLR.length; i++) {
                                    if (trainingLR != null) {
                                        if (trainingLR[i] != null) {
                                            if (trainingLR[i].input != null) {
                                                guess = SC_Perceptron.Guess(trainingLR[i].input);
                                            }
                                            else {
                                                console.PrintError("null2");
                                            }
                                        }
                                        else {
                                            console.PrintError("null1");
                                        }
                                    }
                                    else {
                                        console.PrintError("null0");
                                    }

                                    //var neededPos = { x: trainingLR[i].inputs[0], y: trainingLR[i].inputs[1] };//new Vector2(trainingLR[i].inputs[0], trainingLR[i].inputs[1]);

                                    if (guess > 0) {
                                        if (_dotGoal < lastFrameDot) {
                                            someError = 0;
                                            someDir = 0;
                                        }
                                        else {
                                            someError = 1;
                                            someDir = 1;
                                        }
                                    }
                                    else {
                                        if (_dotGoal < lastFrameDot) {
                                            someError = 0;
                                            someDir = 1;
                                        }
                                        else {
                                            someError = 1;
                                            someDir = 0;
                                        }
                                    }

                                    /*console.PrintError(guess);
                    
                                    if (trainingLR[i].a == 1) {
                                        turnRight++;
                                    }
                                    else if (trainingLR[i].a == -1) {
                                        turnLeft++;
                                    }

                                    if (trainingLR[i].a == 1) {
                                        turnRight++;
                                    }
                                    else if (trainingLR[i].a == -1) {
                                        turnLeft++;
                                    }

                                    if (guess > 0) {

                                        guessedCorrect++;
                                    }
                                    else {
                                        guessedWrong++;
                                    }
                                }
                            }
                            else {
                                swtch = 0;
                            }


                        }
                        else {
                            swtch = 0;
                        }
                    }
                    else {
                        swtch = 0;
                    }
                }*/





                /*
                if (guessedCorrect >= (trainingLR.length * 0.5) - 5 && guessedCorrect <= (trainingLR.length) ||
                    guessedWrong >= (trainingLR.length * 0.5) - 5 && guessedWrong <= (trainingLR.length)) {
                    if (turnRight >= (trainingLR.length * 0.5) - 5 && turnRight <= (trainingLR.length) ||
                        turnLeft >= (trainingLR.length * 0.5) - 5 && turnLeft <= (trainingLR.length)) {
                        if (turnRight > turnLeft) {
                            _guessedCorrectRight++;
                            //Debug.Log("player is Right 0-0");
                            //drone.transform.Rotate(new Vector3(0, 0, -2f), Space.World);       
                        }
                        else if (turnRight < turnLeft) {
                            _guessedCorrectLeft++;
                            //Debug.Log("player is Left 0-0");
                            //drone.transform.Rotate(new Vector3(0, 0, 2f), Space.World);
                        }
                        else {
                            // _guessedCorrectLeft++;
                            // _guessedCorrectRight++;
                            //console.PrintError("Data is too similar");
                        }
                    }
                    else if (turnRight <= (trainingLR.length * 0.5) + 5 && turnRight >= (trainingLR.length) ||
                        turnLeft <= (trainingLR.length * 0.5) + 5 && turnLeft >= (trainingLR.length)) {
                        if (turnRight > turnLeft) {
                            _guessedCorrectRight++;
                            //Debug.Log("player is Right 0-0");
                            //drone.transform.Rotate(new Vector3(0, 0, -2f), Space.World);       
                        }
                        else if (turnRight < turnLeft) {
                            _guessedCorrectLeft++;
                            //Debug.Log("player is Left 0-0");
                            //drone.transform.Rotate(new Vector3(0, 0, 2f), Space.World);
                        }
                        else {
                             //_guessedCorrectLeft++;
                            // _guessedCorrectRight++;
                            //console.PrintError("Data is too similar");
                        }
                    }
                    else {
                        //_guessedCorrectLeft++;
                        //_guessedCorrectRight++;
                        //Debug.Log("Data is too similar");
                    }
                }
                else if (guessedCorrect <= (trainingLR.length * 0.5) + 5 && guessedCorrect >= (trainingLR.length) ||
                    guessedWrong <= (trainingLR.length * 0.5) + 5 && guessedWrong >= (trainingLR.length)) {

                    if (turnRight >= (trainingLR.length * 0.5) - 5 && turnRight <= (trainingLR.length) ||
                        turnLeft >= (trainingLR.length * 0.5) - 5 && turnLeft <= (trainingLR.length)) {
                        if (turnRight > turnLeft) {
                            _guessedCorrectRight++;
                            //Debug.Log("player is Right 0-0");
                            //drone.transform.Rotate(new Vector3(0, 0, -2f), Space.World);       
                        }
                        else if (turnRight < turnLeft) {
                            _guessedCorrectLeft++;
                            //Debug.Log("player is Left 0-0");
                            //drone.transform.Rotate(new Vector3(0, 0, 2f), Space.World);
                        }
                        else {
                            //_guessedCorrectLeft++;
                            //_guessedCorrectRight++;
                            //console.PrintError("Data is too similar");
                        }
                    }
                    else if (turnRight <= (trainingLR.length * 0.5) + 5 && turnRight >= (trainingLR.length) ||
                        turnLeft <= (trainingLR.length * 0.5) + 5 && turnLeft >= (trainingLR.length)) {
                        if (turnRight > turnLeft) {
                            _guessedCorrectRight++;
                            //Debug.Log("player is Right 0-0");
                            //drone.transform.Rotate(new Vector3(0, 0, -2f), Space.World);       
                        }
                        else if (turnRight < turnLeft) {
                            _guessedCorrectLeft++;
                            //Debug.Log("player is Left 0-0");
                            //drone.transform.Rotate(new Vector3(0, 0, 2f), Space.World);
                        }
                        else {
                             //_guessedCorrectLeft++;
                             //_guessedCorrectRight++;
                            //console.PrintError("Data is too similar");
                        }
                    }
                    else {
                         //_guessedCorrectLeft++;
                         //_guessedCorrectRight++;
                        //console.PrintError("Data is too similar");
                    }
                }
                else {
                     //_guessedCorrectLeft++;
                     //_guessedCorrectRight++;
                    //console.PrintError("Data is too similar");
                }*/
                saveCurrentWeightForTheCurrentAngleInsideOfUnitsOf360[Math.round(currentQuarterRoundedAngle)] = SC_Perceptron.GetWeights();
            }
            else {
                console.PrintError("out of range1: " + currentQuarterRoundedAngle);
            }

























            //console.PrintError("_dotGoal: " + _dotGoal);

            if (swtchwaypointtype == 0) {
                if (invertdir == 0) {
                    if (cData.cFWPAltDot < -0.05 || cData.cFWPAltDot > 0.05) //_dotGoal
                    {

                        //TO READD AFTER STRAFE RIGHT LEFT TESTS
                        //TO READD AFTER STRAFE RIGHT LEFT TESTS
                        //TO READD AFTER STRAFE RIGHT LEFT TESTS
                        if (_guessedCorrectLeft > _guessedCorrectRight) {
                            npc.RotLeft(cData[currentObjective.droneIndex].objt.nid);

                            //console.PrintError("0");
                        }
                        else if (_guessedCorrectLeft < _guessedCorrectRight) {
                            npc.RotRight(cData[currentObjective.droneIndex].objt.nid);
                            //console.PrintError("1");
                        }
                        else {
                            /*if (Math.floor(Math.random() * (2 - 0) + 0) == 0) {
                                npc.RotRight(cData[currentObjective.droneIndex].objt.nid);
                            }
                            else {
                                npc.RotLeft(cData[currentObjective.droneIndex].objt.nid);
                            }*/
                            //console.PrintError("2");
                        }
                        //TO READD AFTER STRAFE RIGHT LEFT TESTS
                        //TO READD AFTER STRAFE RIGHT LEFT TESTS
                        //TO READD AFTER STRAFE RIGHT LEFT TESTS


                    }
                }
                else {

                    //TO READD AFTER STRAFE RIGHT LEFT TESTS
                    //TO READD AFTER STRAFE RIGHT LEFT TESTS
                    //TO READD AFTER STRAFE RIGHT LEFT TESTS
                    if (_dotGoal < -0.05 || _dotGoal > 0.05) {
                        if (_guessedCorrectLeft > _guessedCorrectRight) {

                            npc.RotRight(cData[currentObjective.droneIndex].objt.nid);

                            //console.PrintError("0");
                        }
                        else if (_guessedCorrectLeft < _guessedCorrectRight) {
                            npc.RotLeft(cData[currentObjective.droneIndex].objt.nid);
                            //console.PrintError("1");
                        }
                        else {
                            /*if (Math.floor(Math.random() * (2 - 0) + 0) == 0) {
                                npc.RotLeft(cData[currentObjective.droneIndex].objt.nid);
                            }
                            else {
                                npc.RotRight(cData[currentObjective.droneIndex].objt.nid);
                            }*/
                            //console.PrintError("2");
                        }
                    }
                    //TO READD AFTER STRAFE RIGHT LEFT TESTS
                    //TO READD AFTER STRAFE RIGHT LEFT TESTS
                    //TO READD AFTER STRAFE RIGHT LEFT TESTS


                }
            }
            else if (swtchwaypointtype == 1) {
                if (invertdir == 0) {
                    /*if (_guessedCorrectLeft > _guessedCorrectRight) {
                        npc.GoBackward(cData[currentObjective.droneIndex].objt.nid);
                        console.PrintError("0");
                    }
                    else if (_guessedCorrectLeft < _guessedCorrectRight) {
                        npc.GoForward(cData[currentObjective.droneIndex].objt.nid);
                        console.PrintError("1");
                    }
                    else
                    {
                        if (Math.floor(Math.random() * (2 - 0) + 0) == 0)
                        {
                            npc.GoBackward(cData[currentObjective.droneIndex].objt.nid);
                        }
                        else {
                            npc.GoForward(cData[currentObjective.droneIndex].objt.nid);
    
                        }
                        console.PrintError("2");
                    }*/


                    //TO READD AFTER STRAFE RIGHT LEFT TESTS
                    //TO READD AFTER STRAFE RIGHT LEFT TESTS
                    //TO READD AFTER STRAFE RIGHT LEFT TESTS
                    if (_dotGoal < -0.05 || _dotGoal > 0.05) {
                        if (_guessedCorrectLeft > _guessedCorrectRight) {
                            npc.GoBackward(cData[currentObjective.droneIndex].objt.nid);
                            //console.PrintError("0");
                        }
                        else if (_guessedCorrectLeft < _guessedCorrectRight) {
                            npc.GoForward(cData[currentObjective.droneIndex].objt.nid);
                            //console.PrintError("1");
                        }
                        else {
                            /*if (Math.floor(Math.random() * (2 - 0) + 0) == 0) {
                                npc.GoForward(cData[currentObjective.droneIndex].objt.nid);
                            }
                            else {
                                npc.GoBackward(cData[currentObjective.droneIndex].objt.nid);
                            }*/
                            //console.PrintError("2");
                        }
                    }
                    //TO READD AFTER STRAFE RIGHT LEFT TESTS
                    //TO READD AFTER STRAFE RIGHT LEFT TESTS
                    //TO READD AFTER STRAFE RIGHT LEFT TESTS
                }
                else {
                    /*if (_guessedCorrectLeft > _guessedCorrectRight) {
                        npc.GoForward(cData[currentObjective.droneIndex].objt.nid);
    
                        console.PrintError("00");
                    }
                    else if (_guessedCorrectLeft < _guessedCorrectRight) {
                        npc.GoBackward(cData[currentObjective.droneIndex].objt.nid);
                        console.PrintError("01");
                    }
                    else {
                        if (Math.floor(Math.random() * (2 - 0) + 0) == 0) {
                            npc.GoBackward(cData[currentObjective.droneIndex].objt.nid);
                        }
                        else {
                            npc.GoForward(cData[currentObjective.droneIndex].objt.nid);
    
                        }
                        console.PrintError("02");
                    }*/


                    //TO READD AFTER STRAFE RIGHT LEFT TESTS
                    //TO READD AFTER STRAFE RIGHT LEFT TESTS
                    //TO READD AFTER STRAFE RIGHT LEFT TESTS
                    if (_dotGoal < -0.05 || _dotGoal > 0.05) {
                        if (_guessedCorrectLeft > _guessedCorrectRight) {
                            npc.GoForward(cData[currentObjective.droneIndex].objt.nid);

                            //console.PrintError("00");
                        }
                        else if (_guessedCorrectLeft < _guessedCorrectRight) {
                            npc.GoBackward(cData[currentObjective.droneIndex].objt.nid);
                            //console.PrintError("01");
                        }
                        else {
                            /*if (Math.floor(Math.random() * (2 - 0) + 0) == 0) {
                                npc.GoBackward(cData[currentObjective.droneIndex].objt.nid);
                            }
                            else {
                                npc.GoForward(cData[currentObjective.droneIndex].objt.nid);

                            }*/
                            //console.PrintError("02");
                        }
                    }
                    //TO READD AFTER STRAFE RIGHT LEFT TESTS
                    //TO READD AFTER STRAFE RIGHT LEFT TESTS
                    //TO READD AFTER STRAFE RIGHT LEFT TESTS
                }
            }
            else if (swtchwaypointtype == 2) {
                if (invertdir == 0) {
                    if (_dotGoal < -0.05 || _dotGoal > 0.05) {
                        if (_guessedCorrectLeft > _guessedCorrectRight) {
                            npc.StrafeLeft(cData[currentObjective.droneIndex].objt.nid);
                            //console.PrintError("0");
                        }
                        else if (_guessedCorrectLeft < _guessedCorrectRight) {
                            npc.StrafeRight(cData[currentObjective.droneIndex].objt.nid);
                            //console.PrintError("1");
                        }
                        else {
                            /*if (Math.floor(Math.random() * (2 - 0) + 0) == 0) {
                                npc.StrafeRight(cData[currentObjective.droneIndex].objt.nid);
                            }
                            else {
                                npc.StrafeLeft(cData[currentObjective.droneIndex].objt.nid);
                            }*/
                            //console.PrintError("2");
                        }
                    }
                }
                else {
                    if (_dotGoal < -0.05 || _dotGoal > 0.05) {
                        if (_guessedCorrectLeft > _guessedCorrectRight) {
                            npc.StrafeRight(cData[currentObjective.droneIndex].objt.nid);
                            //console.PrintError("0");
                        }
                        else if (_guessedCorrectLeft < _guessedCorrectRight) {

                            npc.StrafeLeft(cData[currentObjective.droneIndex].objt.nid);
                            //console.PrintError("1");
                        }
                        else {
                            /*if (Math.floor(Math.random() * (2 - 0) + 0) == 0) {
                                npc.StrafeLeft(cData[currentObjective.droneIndex].objt.nid);
                            }
                            else {
                                npc.StrafeRight(cData[currentObjective.droneIndex].objt.nid);
                                
                            }*/
                            //console.PrintError("2");
                        }
                    }
                }
            }
        }
        else {
            console.PrintError("===WEIGHTS SUDDENLY BECAME NULL===?");
            swtch = 0;
        }


        lastFrameDot = _dotGoal;
    },
}





function NSEW(a, b, c) {
    return ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) > 0;
}

function NSDIST(a, b, c) {
    return ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
}

function EWDIST(a, b, c) {
    return ((b.y - a.y) * (c.x - a.x) - (b.x - a.x) * (c.y - a.y));
}


