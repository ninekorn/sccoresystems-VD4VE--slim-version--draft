using(console);

include(SC_Utilities.js);
include(SC_AI_PathFind_Mining_InitGrid_5.js);

var stationTilesArray = [];
var initialPathfindStartingPos = [];
var initialPathfindTargetPos = [];

var indexOfPlayer;
var globIndex;

var areThereAnyStations = -1;
var someData = [];

var SC_AI_PathFind_Mining_InitGrid_Data_Mining_5 =
{
    npcGetGridData: function (gridWorldSize, nodeRadius, indexOfGrid, cData, noder) {
        /*if (stationTilesArray[cData.iop] == null || cData.sSwtc == 1)
        {
            //var current_base = ship.GetCurrentBase(cData.objt.bid);
            stationTilesArray[cData.iop] = storage.GetGlobal("station_tiles" + cData.);
            cData.stRot = stationTilesArray[cData.iop].rot;
            cData.stCoord = stationTilesArray[cData.iop].coord;
            cData.sSwtc = 2;
        }
        else
        {
            cData.stRot = stationTilesArray[cData.iop].rot;
            cData.stCoord = stationTilesArray[cData.iop].coord;
        }*/

        if (storage.IsSetGlobal("GlobalIndex_Player_" + cData.objt.pName)) {
            //cData.iop = storage.GetGlobal("GlobalIndex_Player_" + cData.objt.pName);

            console.PrintError("player index: " + cData.iop);
            //console.PrintError(cData.sSwtc);

            if (storage.IsSetGlobal("station_tiles" + cData.SelectedStation)) {
                console.PrintError("station tiles for base " + cData.SelectedStation + " -is- set." + " SC_AI_PathFind_Mining_InitGrid_Data_Mining_5.js");

                if (stationTilesArray[cData.iop] == null && cData.sSwtc == 1)
                {
                    stationTilesArray[cData.iop] = storage.GetGlobal("station_tiles" + cData.SelectedStation);

                    if (stationTilesArray[cData.iop] == null)
                    {
                        console.PrintError("station tiles null00");
                    }
                    else {
                        console.PrintError("station tiles !null01");

                        //stationTiles = stationTilesArray[cData.iop];

                        cData.stRot = stationTilesArray[cData.iop].rot;
                        cData.stCoord = stationTilesArray[cData.iop].coord;
                        //cData.sSwtc = 2;
                        areThereAnyStations = 1;
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
                    areThereAnyStations = -1;
                    console.PrintError("station tiles for base " + cData.SelectedStation + " -is not- set global." + " SC_AI_PathFind_Mining_InitGrid_Data_Mining_5.js");
                }
                else
                {
                    areThereAnyStations = 1;
                    if (cData.SelectedStation == null || isNaN(cData.SelectedStation)) {
                        console.PrintError("station tiles for base " + cData.SelectedStation + " -is not- set." + " SC_AI_PathFind_Mining_InitGrid_Data_Mining_5.js");
                    }
                    else {
                        console.PrintError("station tiles for base " + cData.SelectedStation + " -is set global" + " SC_AI_PathFind_Mining_InitGrid_Data_Mining_5.js");//
                    }
                }

                //console.PrintError("station tiles for base " + cData.SelectedStation + " -is not- set.");

                //storage.SetGlobal("station_tiles" + cData.SelectedStation);
                //var dataOfStation = SC_Station_Tiles_Outpost_01.buildTiles(theBase, cData.SelectedStation);
                //storage.SetGlobal("station_tiles" + cData.SelectedStation, dataOfStation);	
            }
        }


        //SIMPLE FIX FOR ISSUES WITH NO STATIONS AVAILABLE FOR PATHFIND. MOVING ON TO FINDING THE ISSUE IN THE GLOBAL VARIABLE FOR THE STARTING SYSTEM. THE STARTING SYSTEM IS THE SAME CURRENTLY FOR EVERY PLAYERS BUT IF I MAKE THE STATIONS MODELS EVEN LARGER I WOULD BE ABLE TO HAVE SHIPS WITH A PATHFIND STATION EMBEDDED BETWEEN
        //2 COLLIDERS SO THAT THE WALL GARAGE DOOR CAN SLIDE AND FOLLOW THE CORRECT COLLIDER PATH WHATEVER THE ROTATION OF THE STATION IS. TURRETS HULLS FOR UNBREAKABLE GARAGE DOOR UNLESS YOU SHOOT AT THEM AND DESTROY THEM AND SHIPS HULLS FOR BREAKABLE GARAGE DOORS THAT CAN SLIDE WITH A PATHFIND SCRIPT FOR IT TO BE "STUCK" BY 2
        //FARSEER ENGINE OTHER P�LYGON COLLIDERS. ADDING A SCRIPT OR ADDFORCE TO MAKE SURE ANOTHER SHIP NEVER TRAVERSES THE DOOR IF IT ISN'T POSSIBLE FOR THE PATHFIND TO GO THROUGH BECAUSE THERE IS STILL A GARAGE DOOR TO SHOOT AT AND ADD A LASER TO THE DOOR THAT IS INVISIBLE OR DEVICE AND USE THE API NPC.FIRE WITH A DIFFERENT
        //TEXTURE THAT DOES SPARKS AND THE SHIP SCRIPT ON THAT GARAGE DOOR CAN MAKE SURE IT KEEPS BLOCKING THE PLAYER WHEN IS HACKED OR THAT YOU HAVE TO MAKE A PUZZLE OF ACESSING AN NPC COMPUTER AND TALK TO NPC WITH LONG DISCUSSIONS IN ORDER FOR PLAYERS TO GET CLUES ON HOW TO GET OUT OF A STATION WITH WALLS. I BUILT IT BEFORE ONLY
        //WORKING WHEN A PLAYER WAS CHANGING SYSTEMS and there were station walls impedding the player from going anywhere but station inside of the station.

        if (areThereAnyStations == -1) // NO STATIONS AVAILABLE SO THE PATHFIND CANNOT ACCESS THE STATION TILES. RETURM
        {
            console.PrintError("there are no stations " + " pathfind initiated without a main goal. a mistake to find here.");
            var nocoords = { x: 0, y: 0 };


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



       

            if (initialPathfindStartingPos[globIndex] == null || cData.sSwtc == 2)
            {

                if (initialPathfindStartingPos[globIndex] == null) {
                    console.PrintError("initialPathfindStartingPos null ");
                }


                if (cData.sSwtc == 2) {
                    console.PrintError("cData.sSwtc supposed 2 == " +  " cData.sSwtc " + cData.sSwtc);
                }



                var initialPosX = Math.round(cData.nData.nCoord.x);
                var initialPosY = Math.round(cData.nData.nCoord.y);
                var initialPos = { x: initialPosX, y: initialPosY };

                var remnant = 360 - cData.stRot;

                var pathfind
                var pathfindCoord = [];

                pathfindCoord.push()

                initialPathfindStartingPos[globIndex] = SC_Utilities.RotatePoint(initialPos, nocoords, remnant); //nocoords

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
                initialPathfindTargetPos[globIndex] = SC_Utilities.RotatePoint(target, nocoords, remnant); //stationTilesArray[cData.iop].coord //nocoords

                initialPathfindTargetPos[globIndex].x = Math.round(initialPathfindTargetPos[globIndex].x);
                initialPathfindTargetPos[globIndex].y = Math.round(initialPathfindTargetPos[globIndex].y);

                cData.ltp = { x: initialPathfindTargetPos[globIndex].x, y: initialPathfindTargetPos[globIndex].y };
                cData.sSwtc = 0;
            }

            //send no station tiles to the pathfind
            someData = SC_AI_PathFind_Mining_InitGrid_5.npcBuildGrid(gridWorldSize, nodeRadius, indexOfGrid, cData, noder, nocoords);//set to null tiles array //nocoords
            //send no station tiles to the pathfind
        }
        else if (areThereAnyStations == 1) //TODO STATION EXISTS SO THE PATHFIND CANNOT ACCESS STATION TILES. RETURN
        {
            console.PrintError("pathfind for station: " + stationTilesArray[cData.iop].baseID + " pathfind is for something else.");

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

            if (initialPathfindStartingPos[globIndex] == null || cData.sSwtc == 2)
            {
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
            else if (initialPathfindTargetPos[globIndex] == null || cData.sSwtc == 3)
            {
                var targetX = Math.round(cData.cFWP.x);
                var targetY = Math.round(cData.cFWP.y);
                var target = { x: targetX, y: targetY };

                var remnant = 360 - cData.stRot;
                initialPathfindTargetPos[globIndex] = SC_Utilities.RotatePoint(target, stationTilesArray[cData.iop].coord, remnant);

                initialPathfindTargetPos[globIndex].x = Math.round(initialPathfindTargetPos[globIndex].x);
                initialPathfindTargetPos[globIndex].y = Math.round(initialPathfindTargetPos[globIndex].y);

                cData.ltp = { x: initialPathfindTargetPos[globIndex].x, y: initialPathfindTargetPos[globIndex].y };
                cData.sSwtc = 0;
            }

            someData = SC_AI_PathFind_Mining_InitGrid_5.npcBuildGrid(gridWorldSize, nodeRadius, indexOfGrid, cData, noder, stationTilesArray[cData.iop]);
        }



        return someData;
    }
};