include(SystemsPresets.js);
include(SC_Utilities.js);

using(generator);
using(relations);
using(spawn);
using(console);
using(storage);
using(game);
using(timer);
using(player);

var NumOfSystems = 1; //75
var MinJumpsBetweenCapitals = 1; //3
var curNumOfSystems = 1; // number of systems created
var systems = [];
//GALAXY MARKET//
var startOnce = false;
var newArray = [];
//GALAXY MARKET//

var start_system_id;







function indexOfStuff(someIndex) {
    if (someIndex == "0") {
        return 0;
    }
    if (someIndex == "45") {
        return 1;
    }
    else if (someIndex == "90") {
        return 2;
    }
    else if (someIndex == "135") {
        return 3;
    }
    else if (someIndex == "180") {
        return 4;
    }
    else if (someIndex == "225") {
        return 5;
    }
    else if (someIndex == "270") {
        return 6;
    }
    else if (someIndex == "315") {
        return 7;
    }
}


//https: //stackoverflow.com/questions/13695317/rotate-a-point-around-another-point
function RotatePoint(pointToRotate, centerPoint, angleInDegrees) {
    var angleInRadians = angleInDegrees * (Math.PI / 180);
    var cosTheta = Math.cos(angleInRadians);
    var sinTheta = Math.sin(angleInRadians);

    var newX = (cosTheta * (pointToRotate.x - centerPoint.x) - sinTheta * (pointToRotate.y - centerPoint.y) + centerPoint.x);
    var newY = (sinTheta * (pointToRotate.x - centerPoint.x) + cosTheta * (pointToRotate.y - centerPoint.y) + centerPoint.y);

    var newPos = { x: newX, y: newY };

    return newPos;
}






function SpawnTerrainManager(chosenBase, sys_id, inf, player_id) {
    var id;
    id = generator.AddNPCShipToSystem("terrain Manager", "terrainGenerator", 5, "special_human_techship", sys_id, chosenBase.coords.x, chosenBase.coords.y, { class: "terrainManager", greeting: "terminal", unique_id: "terrainManager", ownerPlayerShipId: player_id });
    relations.SetShipFaction(id, inf.faction);
    generator.DockShipToBase(id, chosenBase.baseID);
    ship.SetCurrentArea(id, "offices");
    generator.SetNPCAvatarImg(id, "avatars/unique/terminal_01.png");
    npc.SetHiddenInStation(id, true);
}








function CheckForPlayers(args) {


  
    if (storage.IsSetGlobal("startingsystem"))//startingsystem args: id and name
    {
        var someStartingBase = storage.GetGlobal("startingsystem");
        if (someStartingBase != null) {
            if (someStartingBase.coords != null) {
                if (someStartingBase.coords.x != null && someStartingBase.coords.y != null) {
                    var player_id = player.GetShipOfPlayer("ninekorn");
                    if (player_id != null && !isNaN(player_id))
                    {
                        //console.PrintError("player is in the server");
                        if (!storage.IsSetGlobal("playerspawnset"))
                        {
                            spawn.SetCoordinates(someStartingBase.coords.x, someStartingBase.coords.y - 12.5);
                            //generator.DockShipToBase(player_id, someStartingBase.base_id);
                            storage.SetGlobal("playerspawnset", player_id);
                            //console.Print("set " + args.what_to_say);             
                        }
                        else {

                            var sys_id_player = ship.GetSystemID(player_id);

                            //var glob = storage.GetGlobal("stationINT" + baseInteriorGlobalVariable.sys_id);

                            if (sys_id_player != null)
                            {

                                var infplayersys = generator.GetSystemByID(sys_id_player);
                                var glob = storage.GetGlobal("planet earth INT" + sys_id_player);//someStartingBase.sys_id);

                                if (infplayersys != null)
                                {
                                    if (infplayersys.name != null)
                                    {
                                        if (hasSpawnedTerrainManager == -1) {
                                            console.Print("spawning terrain manager for player");
                                            SpawnTerrainManager(someStartingBase, sys_id_player, infplayersys, player_id);
                                            hasSpawnedTerrainManager = 1;
                                        }

                                        /*if (infplayersys.name == "Earth") {
                                            if (hasSpawnedTerrainManager == -1) {
                                                console.Print("spawning terrain manager for player");
                                                SpawnTerrainManager(someStartingBase, glob.sys_id, infplayersys, player_id);
                                                hasSpawnedTerrainManager = 1;
                                            }
                                        }*/
                                    }
                                }
                            }

                            /*var glob = storage.GetGlobal("planet earth INT" + baseInteriorGlobalVariable.sys_id);
                            glob.x = someCoordsEarth.x;
                            glob.y = someCoordsEarth.y;
                            glob.id = null;//decoID0;
                            storage.SetGlobal("planet earth INT" + baseInteriorGlobalVariable.sys_id, glob);
                            */



                        }
                    }
                }
            }
        }
    }


    //console.Print("I was told to say " + args.what_to_say);
}

var hasSpawnedTerrainManager = -1;












// Generate objects inside the system
function GenSystemObjects(sys_id, args) {
    //GALAXY MARKET//
    if (startOnce == false) {
        //var systemId = storage.GetGlobal("marketTerminalChosenSystem", whatever);
        var globalShopPriceVariable = storage.SetGlobal("globalShopPrice", newArray);
        startOnce = true;
    }
    //GALAXY MARKET//


    // setup some basic variables
    args.sys_id = sys_id;
    args.system_info = generator.GetSystemByID(sys_id);

    if (args.system_info.faction.length == 0) {
        args.system_info.faction = "none";
        //console.PrintError("System with no faction occurred. Something is broken in GenerateGalaxy script.");
        console.WaitForUserInteraction();
    }

    //------------------------------------------------------
    // Generate JUMPGATES
    //

    // add jumpgates - one for each system
    //9sMODs => it doesn't add any jumpgates anymore from that function, only the station exterior and interior global variables as the jumpgates will be deleted and recreated depending on if the performance will allow later on. i will need tons of jumpgates
    //for alien "jump" invasions in this "rts mmo action space game with rpg elements" 9sMODs final rappatriation of all of my work since i started coding mods for void expanse. A feeling of Command and Conquer 4, that i have played 5 mins, with movable
    //space stations (ships) for mining/outpost/military/science/business. Now i need to detach the mining station from the minable asteroid in blender, so that the mining ship station is able to move from one asteroid to another and attach itself to it
    //with pathfind, but that pathfind variant will have to accomodate the bigger size of the station compared to the tiny size of the drones models i have made. I have a tiny idea about how to do it, and will developp this idea further but for the moment,
    //i will continu doing what i am doing with jumpgates.
    args.jumpgates = AddJumpgates(args);


    var timerClass = timer.SetTimer(0, "CheckForPlayers", { what_to_say: "hello" }, 0);


    //--------------------------------------------------------
    // P R E S E T - B A S E D
    //
    // generation system

    // go through function check	
    var chosen_presets = [];
    var total_chance = 0;
    for (var i in SystemsPresets) {
        var preset = SystemsPresets[i];
        if (preset.CheckIfFits(args)
            && preset.QuantityCheck(args)) {
            chosen_presets.push(preset);
            total_chance += preset.Chance;
        }
    }

    // pick by chances
    var presetResult;
    var pickedIndex;
    if (chosen_presets.length == 0) {
        //console.PrintError("No presets were chosen. Should never occur, fix SystemsPresets.");
        console.WaitForUserInteraction();
        return false;
    }
    else if (chosen_presets.length == 1) {
        presetResult = chosen_presets[0];
    }
    else {
        var currentChance = 0;
        var pickChance = MathExt.RandRange(1, total_chance + 1);
        for (var k = 0; k < chosen_presets.length; k++) {
            if (pickChance <= chosen_presets[k].Chance) {
                presetResult = chosen_presets[k];
                break;
            }
            pickChance -= chosen_presets[k].Chance;
        }
    }
    presetResult.Quantity++;
    generator.AddSystemTag(args.sys_id, "preset", presetResult.Name);

    console.Print("new system: " + presetResult.Name + " - " + args.system_info.name + " (" + args.system_info.faction + ")");



    // generation process
    args.bases = presetResult.CreateSystemBases(args);








    // create safe zone of radius 60 around every base
    args.safe_zones_centers_x = [];
    args.safe_zones_centers_y = [];
    args.safe_zones_radiuses = [];


    for (var i = 0; i < args.bases.length; i++) {
        var coord = args.bases[i].coord;
        args.safe_zones_centers_x.push(coord.x);
        args.safe_zones_centers_y.push(coord.y);
        args.safe_zones_radiuses.push(60);
    }


    if (args.jumpgates != null)  /////////////////////////////////////// FIX ///////////////////////////////////////
    {
        // create save zone of radius 20 around every jumpgate
        for (var i = 0; i < args.jumpgates.length; i++) {
            var coord = args.jumpgates[i].coord;
            args.safe_zones_centers_x.push(coord.x);
            args.safe_zones_centers_y.push(coord.y);
            args.safe_zones_radiuses.push(20);
            // console.Print("Safe zone around jumpgate at x=" + coord.x + " and y=" + coord.y); 
        }
    }


    storage.SetGlobal("system_" + args.sys_id + "_safe_zones_centers_x", args.safe_zones_centers_x); //store safe zones
    storage.SetGlobal("system_" + args.sys_id + "_safe_zones_centers_y", args.safe_zones_centers_y); //store safe zones
    storage.SetGlobal("system_" + args.sys_id + "_safe_zones_radiuses", args.safe_zones_radiuses); //store safe zones





    presetResult.CreateSystemObjects(args);



    return true;
}

function AddJumpgates(args) {
    var sys_id = args.sys_id;
    var inf = args.system_info;

    var system_vec = { x: inf.coord_x, y: inf.coord_y };
    var bounds = CSGen.GetWorldBounds();
    var linkedSystems = generator.GetLinkedSystems(sys_id);

    var res = [];

    /*for (var i = 0; i < linkedSystems.length; i++) {
        var curinf = generator.GetSystemByID(linkedSystems[i]);
        var angle = -MathExt.Vector2Angle(system_vec, { x: curinf.coord_x, y: curinf.coord_y });
        var radius = bounds.x * 0.75 * MathExt.RandRangeDouble(0.6, 1.2);
        var coord_x = radius * Math.cos(angle);
        var coord_y = radius * Math.sin(angle);

        var linkedSystem = linkedSystems[i];

        var jg_id = generator.AddJumpgate(
            sys_id,
            linkedSystem,
            coord_x,
            coord_y,
            angle,
            "jumpgate_01");

        var jumpgate_coords = { x: coord_x, y: coord_y };

        res.push({ id: jg_id, coord: jumpgate_coords });
    }*/




    ////console.PrintError(linkedSystems.length );
    /*if (linkedSystems.length == 0)
    {
        var curinf = generator.GetSystemByID(sys_id);
        var angle = -MathExt.Vector2Angle(system_vec, { x: curinf.coord_x, y: curinf.coord_y });
        //var angle = -MathExt.Vector2Angle(system_vec, { x: curinf.coord_x, y: curinf.coord_y });
        var radius = bounds.x * 0.75 * MathExt.RandRangeDouble(0.6, 1.2);
        var coord_x = curinf.coord_x;
        var coord_y = curinf.coord_y;

        //var linkedSystem = linkedSystems[i];
        var jg_id = generator.AddJumpgate(
            sys_id,
            null,
            99,
            99,
            0,
            "jumpgate_01");

        var jumpgate_coords = { x: 99, y: 99 };

        res.push({ id: jg_id, coord: jumpgate_coords });
        
        var id = generator.AddNPCShipToSystem("station test", "ai_station_interior", 100, "xml_station_interior", sys_id, 99, 99, { someTag: "stationTest", class: "stationInterior", greeting: "terminal", unique_id: "stationInterior"}); //    
        
    }
    else*/
    {
        /*var curinf = generator.GetSystemByID(sys_id);
        if (curinf.name != "player base") {
            for (var i = 0; i < linkedSystems.length; i++) {
                var curinf = generator.GetSystemByID(linkedSystems[i]);
                var angle = -MathExt.Vector2Angle(system_vec, { x: curinf.coord_x, y: curinf.coord_y });
                var radius = bounds.x * 0.75 * MathExt.RandRangeDouble(0.6, 1.2);
                var coord_x = radius * Math.cos(angle);
                var coord_y = radius * Math.sin(angle);

                var linkedSystem = linkedSystems[i];

                var jg_id = generator.AddJumpgate(
                    sys_id,
                    sys_id,
                    coord_x,
                    coord_y,
                    angle,
                    "jumpgate_01");

                var jumpgate_coords = { x: coord_x, y: coord_y };

                res.push({ id: jg_id, coord: jumpgate_coords });
            }

            //var id = generator.AddNPCShipToSystem("station test", "ai_station_interior", 100, "xml_station_interior", sys_id, coord_x, coord_y, { someTag: "stationTest", class: "stationInterior", greeting: "terminal", unique_id: "stationInterior" }); //    

        }
        else {
            //console.PrintError("SPAWNED JUMPGATE IN player base");
            for (var i = 0; i < linkedSystems.length; i++) {
                var curinf = generator.GetSystemByID(linkedSystems[i]);
                var angle = -MathExt.Vector2Angle(system_vec, { x: curinf.coord_x, y: curinf.coord_y });
                var radius = bounds.x * 0.75 * MathExt.RandRangeDouble(0.6, 1.2);
                var coord_x = radius * Math.cos(angle);
                var coord_y = radius * Math.sin(angle);

                var linkedSystem = linkedSystems[i];

                var jg_id = generator.AddJumpgate(
                    sys_id,
                    sys_id,
                    coord_x,
                    coord_y,
                    angle,
                    "jumpgate_01");

                var jumpgate_coords = { x: coord_x, y: coord_y };

                res.push({ id: jg_id, coord: jumpgate_coords });
            }
            var id = generator.AddNPCShipToSystem("station test", "ai_station_interior", 100, "xml_station_interior", sys_id, coord_x, coord_y, { someTag: "stationINT", class: "stationINT", greeting: "terminal", unique_id: "stationINT" }); //    
        }*/


        
        var curinf = generator.GetSystemByID(sys_id);

        if (curinf.name == "player base") {

            //console.PrintError("testing 3");
            /*var randomIndexStationOutpost = Math.floor(Math.random() * arrayOfOutpostStations.length);
          
            var getSomeIndex = arrayOfOutpostStations[randomIndexStationOutpost].substring(11, arrayOfOutpostStations[randomIndexStationOutpost].length); //outpost_01_0

            //var index = indexOfStuff(getSomeIndex);
            var parsedAngle = parseInt(getSomeIndex);

            var tagClass = "";
            // assign class
            if (arrayOfOutpostStations[randomIndexStationOutpost].indexOf("mining") >= 0) {
                tagClass = "mining";
            }
            else if (arrayOfOutpostStations[randomIndexStationOutpost].indexOf("outpost") >= 0) {
                tagClass = "outpost";
            }
            else if (arrayOfOutpostStations[randomIndexStationOutpost].indexOf("science") >= 0) {
                tagClass = "science";
            }
            else if (arrayOfOutpostStations[randomIndexStationOutpost].indexOf("business") >= 0) {
                tagClass = "business";
            }
            else if (arrayOfOutpostStations[randomIndexStationOutpost].indexOf("military") >= 0) {
                tagClass = "military";
            }

            var angle = -MathExt.Vector2Angle(system_vec, { x: curinf.coord_x, y: curinf.coord_y });
            var radius = bounds.x * 0.75 * MathExt.RandRangeDouble(0.6, 1.2);
            var coord_x = radius * Math.cos(angle);
            var coord_y = radius * Math.sin(angle);
            //var id = generator.AddNPCShipToSystem("station test", "ai_station_interior", 100, "xml_station_interior", sys_id1, coord_x - 5, coord_y + 10, { someTag: "stationINT", class: "stationINT", greeting: "terminal", unique_id: "stationINT" }); //    
            //var radToDegAngle = angle;//SC_Utilities.RadianToDegree(angle);
            storage.SetGlobal("stationINT" + sys_id1, { id: null, x: coord_x, y: coord_y, sys_id: sys_id1, xml_id: arrayOfOutpostStations[randomIndexStationOutpost], DegAngle: parsedAngle, class: tagClass});
            storage.SetGlobal("stationEXT" + sys_id1, { id: null, x: coord_x, y: coord_y, sys_id: sys_id0, xml_id: arrayOfOutpostStations[randomIndexStationOutpost], DegAngle: parsedAngle, class: tagClass });
            */
            ////console.PrintError("station ship interior generated");
















            /*var angle = ship.GetRotation(id);
            var radToDeg = (angle * (180.0 / Math.PI)) % 360;
            ////console.PrintError("not working0");
            var correctSystem = sys_id;

            //radToDeg = (360 - radToDeg);

            if (radToDeg > 180) {
                radToDeg -= 180;

            }
            else if (radToDeg < 180) {
                radToDeg += 180;
            }

            var degToRad = (radToDeg * Math.PI / 180);

            //var angler = 360 - radToDeg;
            var jg_id0 = generator.AddJumpgate(
                sys_id,
                systems[j],
                pointFrontX,
                pointFrontY,
                degToRad,
                "jumpgate_station");

            lastSpawnedJumpgateSameSystemFront = jg_id0;

            ////console.PrintError("TEST");     

            var stationShips = game.GetSystemShips(systems[j]);

            ////console.PrintError(stationShips.length + " stationShips.length ");     

            for (var g = 0; g < stationShips.length; g++) {
                var correctTag = npc.GetTag(stationShips[g], "someTag");

                ////console.PrintError(correctTag);

                if (correctTag == "stationINT") {
                    var angleStationINT = ship.GetRotation(stationShips[g]);
                    ////console.PrintError(angleStationINT + " ANGLE INTERIOR");
                    var radToDegStationINT = (angleStationINT * (180.0 / Math.PI)) % 360;

                    if (radToDegStationINT < 0) {
                        radToDegStationINT *= -1;
                    }

                    //var degToRad = (angleStationINT * (Math.PI / 180.0));

                    var coordsNPCINT = ship.GetCoordinates(stationShips[g]);
                    ////console.PrintError(coordsNPCINT.x + " _ " + coordsNPCINT.y + " _ " + " coords INTERIOR");

                    var pointFrontINTX = (19 * Math.cos(radToDegStationINT * Math.PI / 180)) + coordsNPCINT.x; //radToDeg * Math.PI / 180
                    var pointFrontINTY = (19 * Math.sin(radToDegStationINT * Math.PI / 180)) + coordsNPCINT.y; //radToDeg * Math.PI / 180)

                    //var distPlayerFrontINT = Math.sqrt(((pointFrontINTX - coordsPlayer.x) * (pointFrontINTX - coordsPlayer.x)) + ((pointFrontINTY - coordsPlayer.y) * (pointFrontINTY - coordsPlayer.y)));

                    var jg_id1 = generator.AddJumpgate(
                        systems[j],
                        sys_id,
                        pointFrontINTX,
                        pointFrontINTY,
                        angleStationINT,
                        "jumpgate_station");

                    //lastSpawnedJumpgateSystemInteriorFront = jg_id1;

                    someOtherSwitchFront = true;
                    mainFrameCounterFront = 0;
                    ////console.PrintError("spawned station INTERIOR");
                }
            }*/
        }





        /*for (var i = 0; i < linkedSystems.length; i++)
        {
            var curinf = generator.GetSystemByID(linkedSystems[i]);
            var angle = -MathExt.Vector2Angle(system_vec, { x: curinf.coord_x, y: curinf.coord_y });
            var radius = bounds.x * 0.75 * MathExt.RandRangeDouble(0.6, 1.2);
            var coord_x = radius * Math.cos(angle);
            var coord_y = radius * Math.sin(angle);

            var linkedSystem = linkedSystems[i];

            var jg_id = generator.AddJumpgate(
                sys_id,
                linkedSystem,
                coord_x,
                coord_y,
                angle,
                "jumpgate_station");

            var jumpgate_coords = { x: coord_x, y: coord_y };

            res.push({ id: jg_id, coord: jumpgate_coords });



            // add turrets	
            /*var turret_radius = 10;
            var level = inf.danger_level + Math.round(MathExt.RandDouble() * 4 - 2);
            level = utils.Clamp(level, 1, 100);
            var angle_offset = generator.GetJumpgateRotation(jg_id);
 
            if (inf.faction == "aliens") {
                // do not spawn turrets in aliens system
            }
            else {
                // do not spawn turrets in neutral system except starting system
                if (inf.faction != "none" || sys_id == spawn.GetSystemID()) {
                    NpcGenerator.SpawnJumpgateTurretsOfType(args, jumpgate_coords, 2, level, turret_radius, "Turret", "special_human_turret_gate", inf.faction, { class: "turret", non_talkable: true }, angle_offset);
                }
            }
            //var id = generator.AddNPCShipToSystem("station test", "ai_station", 100, "xml_station_test", sys_id, 50, 0, { someTag: "stationTest", class: "stationTest", greeting: "terminal", unique_id: "stationTest", stationIDFrom: sys_id }); //    
        }*/
    }




    return res;
}



/*
//-----------------------------------------------------------
// GenerateGalaxy
// entry point of this script
//-----------------------------------------------------------	
function GenerateGalaxy(args) {
    //set seed of random
    MathExt.RandSeedStr(args.seed);

    NumOfSystems = 2;//Math.floor(50 + 50 * (args.galaxy_size / 255));
    args.NumOfSystems = NumOfSystems; //NumOfSystems
    console.Print("Creating galaxy with " + NumOfSystems + " systems. Seed is: " + args.seed);

    ResetPresetsQuantity();

    // generate systems
    generator.ReportLoadingPhase($x0001); // Generating systems
    GenerateSystems(args);

    // generate links
    //generator.ReportLoadingPhase($x0002); // Generating links
    //GenerateLinks(args);

    var sys_id0 = generator.AddSystem(5, 5, "Solar System", MathExt.RandInt());
    var sys_id1 = generator.AddSystem(5, 7, "player base", MathExt.RandInt());

    //var Systems = generator.GetAllSystems();

    /*var index = 0;

    for (var i = 0; i < Systems.length; i++) {
        var inf = generator.GetSystemByID(Systems[i]);
        if (inf.name == "Solar System") {
            index = i;
        }
    } 

    Systems.splice(index, 1);
    


    generator.AddSystemsLink(sys_id0, sys_id1);
    generator.ReportLoadingProgress(1);
    generator.FixConnectivity();


    start_system_id = sys_id0;




    var start_system = generator.GetSystemByID(start_system_id);
    start_coord.x = start_system.coord_x;
    start_coord.y = start_system.coord_y;
    args.start_system_id = start_system_id;
    spawn.SetSystemID(start_system_id);
    //console.Print("start system: " + start_system_id);

    // generate factions
    generator.ReportLoadingPhase($x0003); // Generating factions
    if (!GenerateFactions(args, start_coord, start_system_id)) {
        // failed to generate galaxy
        return false;
    }

    // fix some wrongly generated systems
    FixFactions(args, start_coord, start_system_id);

    // set tech and danger levels
    generator.ReportLoadingPhase($x0004); // Calculating levels
    GenerateSystemLevels(args, start_coord, start_system_id);

    // misc
    generator.ReportLoadingPhase($x0005); // Generating system objects
    if (!GenerateSystemObjects(args)) {
        return false;
    }

    generator.ReportLoadingPhase($x0006); // Final preparations
    generator.ReportLoadingProgress(1);





    /*
       var sys_id0 = generator.AddSystem(5, 5, "Solar System", MathExt.RandInt());
       var sys_id1 = generator.AddSystem(5, 7, "player base", MathExt.RandInt());
   
   
       generator.AddSystemsLink(sys_id0, sys_id1);
       generator.FixConnectivity();
    
      
   
       args.jumpgates = AddJumpgates(args);
   
   
   
       //--------------------------
       // O R I G I N (pick)
       //
       var angle = MathExt.RandRangeDouble(0, Math.PI * 2);
       var start_coord = { x: 50 + 50 * Math.cos(angle), y: 50 + 50 * Math.sin(angle) };
   
   
       //var start_system_id = generator.GetClosestSystemToPoint(start_coord.x, start_coord.y);
       start_system_id = sys_id0;
   
   
   
   
       var start_system = generator.GetSystemByID(start_system_id);
       start_coord.x = start_system.coord_x;
       start_coord.y = start_system.coord_y;
       args.start_system_id = start_system_id;
       spawn.SetSystemID(start_system_id);
       //console.Print("start system: " + start_system_id);
   
       // generate factions
       generator.ReportLoadingPhase($x0003); // Generating factions
       if (!GenerateFactions(args, start_coord, start_system_id))
       {
           // failed to generate galaxy
           return false;
       }
   
       // fix some wrongly generated systems
       FixFactions(args, start_coord, start_system_id);
   
       // set tech and danger levels
       generator.ReportLoadingPhase($x0004); // Calculating levels
       GenerateSystemLevels(args, start_coord, start_system_id);
   
       // misc
       generator.ReportLoadingPhase($x0005); // Generating system objects
       if (!GenerateSystemObjects(args))
       {
           return false;
       }
   
       generator.ReportLoadingPhase($x0006); // Final preparations
       generator.ReportLoadingProgress(1);

    return true;
}*/



var sys_id0;
var sys_id1;
var sys_id2;

function GenerateGalaxy(args) {
    MathExt.RandSeedStr(args.seed);

    NumOfSystems = 3;//Math.floor(50 + 50 * (args.galaxy_size / 255));
    args.NumOfSystems = NumOfSystems; //NumOfSystems
    console.Print("Creating galaxy with " + NumOfSystems + " systems. Seed is: " + args.seed);

    ResetPresetsQuantity();

    // generate systems
    generator.ReportLoadingPhase($x0001); // Generating systems
    //GenerateSystems(args);

    // generate links
    //generator.ReportLoadingPhase($x0002); // Generating links
    //GenerateLinks(args);


    /*var Systems = generator.GetAllSystems();

    /*var index = 0;

    for (var i = 0; i < Systems.length; i++) {
        var inf = generator.GetSystemByID(Systems[i]);
        if (inf.name == "Solar System") {
            index = i;
        }
    } 

    Systems.splice(index, 1);
    


    generator.AddSystemsLink(Systems[0], Systems[1]);

    generator.ReportLoadingProgress(1);

    generator.FixConnectivity();*/









    //9sMODs
    sys_id0 = generator.AddSystem(5, 5, "player base", MathExt.RandInt()); // call it player clone resuscitation new technology base/station invented right on time for helping in the conflict...
    sys_id1 = generator.AddSystem(5, 7, "Solar System", MathExt.RandInt());
    sys_id2 = generator.AddSystem(5, 9, "Earth", MathExt.RandInt());






    //spawn.SetSystemID(sys_id1);

    /*sys_id2 = generator.AddSystem(5, 7, "earth population retreat system", MathExt.RandInt());
    sys_id2 = generator.AddSystem(5, 7, "system xengatarn", MathExt.RandInt());
    sys_id3 = generator.AddSystem(5, 7, "The Moon", MathExt.RandInt());
    sys_id4 = generator.AddSystem(5, 7, "Mars", MathExt.RandInt());
    sys_id5 = generator.AddSystem(5, 7, "Mars", MathExt.RandInt());*/









    generator.AddSystemsLink(sys_id0, sys_id1);
    generator.AddSystemsLink(sys_id1, sys_id0);
    generator.AddSystemsLink(sys_id0, sys_id2);
    generator.AddSystemsLink(sys_id1, sys_id2);
    generator.FixConnectivity();



    var curinf = generator.GetSystemByID(sys_id0);

    //if (curinf.name == "Solar System" || curinf.name == "player base") {

    //console.PrintError("testing 000");
    var randomIndexStationOutpost = Math.floor(Math.random() * arrayOfOutpostStations.length);

    var getSomeIndex = arrayOfOutpostStations[randomIndexStationOutpost].substring(11, arrayOfOutpostStations[randomIndexStationOutpost].length); //outpost_01_0

    //var index = indexOfStuff(getSomeIndex);
    var parsedAngle = parseInt(getSomeIndex);

    var tagClass = "";
    // assign class
    if (arrayOfOutpostStations[randomIndexStationOutpost].indexOf("mining") >= 0) {
        tagClass = "mining";
    }
    else if (arrayOfOutpostStations[randomIndexStationOutpost].indexOf("outpost") >= 0) {
        tagClass = "outpost";
    }
    else if (arrayOfOutpostStations[randomIndexStationOutpost].indexOf("science") >= 0) {
        tagClass = "science";
    }
    else if (arrayOfOutpostStations[randomIndexStationOutpost].indexOf("business") >= 0) {
        tagClass = "business";
    }
    else if (arrayOfOutpostStations[randomIndexStationOutpost].indexOf("military") >= 0) {
        tagClass = "military";
    }

    var system_vec = { x: curinf.coord_x, y: curinf.coord_y };
    var angle = -MathExt.Vector2Angle(system_vec, { x: curinf.coord_x, y: curinf.coord_y });
    var bounds = CSGen.GetWorldBounds();
    var radius = bounds.x * 0.75 * MathExt.RandRangeDouble(0.6, 1.2);
    //var coord_x = radius * Math.cos(angle);
    //var coord_y = radius * Math.sin(angle);
    //var id = generator.AddNPCShipToSystem("station test", "ai_station_interior", 100, "xml_station_interior", sys_id1, coord_x - 5, coord_y + 10, { someTag: "stationINT", class: "stationINT", greeting: "terminal", unique_id: "stationINT" }); //    
    //var radToDegAngle = angle;//SC_Utilities.RadianToDegree(angle);
    storage.SetGlobal("stationINT" + sys_id0, { id: null, x: null, y: null, sys_id: sys_id0, sys_id_link: sys_id1, xml_id: arrayOfOutpostStations[randomIndexStationOutpost], DegAngle: parsedAngle, class: tagClass, name: curinf.name, baseID: null });
    storage.SetGlobal("stationEXT" + sys_id1, { id: null, x: null, y: null, sys_id: sys_id1, sys_id_link: sys_id0, xml_id: "xml_station_exterior", DegAngle: parsedAngle, class: tagClass, name: curinf.name, baseID: null });

    var curinf = generator.GetSystemByID(sys_id2);
    var system_vec = { x: curinf.coord_x, y: curinf.coord_y };
    storage.SetGlobal("planet earth INT" + sys_id0, { id: null, x: null, y: null, sys_id: sys_id2, sys_id_link: sys_id0, xml_id: "planet_earth_night", DegAngle: parsedAngle, class: tagClass, name: curinf.name, baseID: null });
    storage.SetGlobal("planet earth EXT" + sys_id1, { id: null, x: null, y: null, sys_id: sys_id2, sys_id_link: sys_id1, xml_id: "planet_earth_night", DegAngle: parsedAngle, class: tagClass, name: curinf.name, baseID: null });
    storage.SetGlobal("planet earth before invasion", { id: null, x: null, y: null, sys_id: sys_id2, sys_id_link: sys_id1, xml_id: "planet_earth_night", DegAngle: parsedAngle, class: tagClass, name: curinf.name, baseID: null });
    storage.SetGlobal("planet earth after invasion", { id: null, x: null, y: null, sys_id: sys_id2, sys_id_link: sys_id1, xml_id: "planet_earth_night", DegAngle: parsedAngle, class: tagClass, name: curinf.name, baseID: null });





    //}





    //var angle = -MathExt.Vector2Angle(system_vec, { x: curinf.coord_x, y: curinf.coord_y });
    //var radius = bounds.x * 0.75 * MathExt.RandRangeDouble(0.6, 1.2);
    //var coord_x = radius * Math.cos(angle);
    //var coord_y = radius * Math.sin(angle);
    //var id = generator.AddNPCShipToSystem("station test", "ai_station_interior", 100, "xml_station_interior", sys_id1, coord_x - 5, coord_y + 10, { someTag: "stationINT", class: "stationINT", greeting: "terminal", unique_id: "stationINT" }); //    

    //var radToDegAngle = angle;//SC_Utilities.RadianToDegree(angle);
    //storage.SetGlobal("stationINT" + sys_id1, { id: id, coordsx: coord_x, coordsy: coord_y, sys_id: sys_id1, xml_id: "xml_station_interior", DegAngle: radToDegAngle });

    ////console.PrintError("station ship interior generated");













    //--------------------------
    // O R I G I N (pick)
    //
    var angle = MathExt.RandRangeDouble(0, Math.PI * 2);
    var start_coord = { x: 50 + 50 * Math.cos(angle), y: 50 + 50 * Math.sin(angle) };


    //var start_system_id = generator.GetClosestSystemToPoint(start_coord.x, start_coord.y);
    start_system_id = sys_id0;




    var start_system = generator.GetSystemByID(start_system_id);

    start_coord.x = start_system.coord_x;
    start_coord.y = start_system.coord_y;

    var startsysdata = { id: start_system_id, name: start_system};
    storage.SetGlobal("startingsystem", startsysdata);

    args.start_system_id = start_system_id;
    spawn.SetSystemID(start_system_id);
    //console.Print("start system: " + start_system_id);




    // generate factions
    generator.ReportLoadingPhase($x0003); // Generating factions
    if (!GenerateFactions(args, start_coord, start_system_id)) {
        // failed to generate galaxy
        return false;
    }

    // fix some wrongly generated systems
    FixFactions(args, start_coord, start_system_id);

    // set tech and danger levels
    generator.ReportLoadingPhase($x0004); // Calculating levels
    GenerateSystemLevels(args, start_coord, start_system_id);

    // misc
    generator.ReportLoadingPhase($x0005); // Generating system objects
    if (!GenerateSystemObjects(args)) {
        return false;
    }

    generator.ReportLoadingPhase($x0006); // Final preparations
    generator.ReportLoadingProgress(1);

    return true;
}










function GenerateSystems(args) {
    var center = { x: 50, y: 50 };
    var tries = 20; //max attempts to create one system
    var minDistanceBetweenStars = 1; //3

    curNumOfSystems = 0;

    SystemNameGenerator.Clear();
    StationNameGenerator.Clear();


    var currentCount = 0;
    while (curNumOfSystems < NumOfSystems) {
        generator.ReportLoadingProgress(curNumOfSystems / NumOfSystems);


        for (var i = 0; i < tries; i++) {
            var coord = { x: MathExt.RandRangeDouble(5, 7), y: MathExt.RandRangeDouble(5, 7) }; // 5-95
            var dist = MathExt.Vector2Length(coord, center);

            if (dist > 1) //test if coordinate in galaxy circle // was 50
            {
                i--;
            }
            else {
                var mindist = generator.GetMinDistanceToSystem(coord.x, coord.y);
                if (mindist > minDistanceBetweenStars) {
                    var sysName = SystemNameGenerator.GetName();
                    //var sys_id = generator.AddSystem(coord.x, coord.y, "Solar System", MathExt.RandInt());

                    if (currentCount < 1) {
                        var sys_id = generator.AddSystem(coord.x, coord.y, "Solar System", MathExt.RandInt());
                        //var sys_id = generator.AddSystem(coord.x, coord.y, sysName, MathExt.RandInt());
                    }
                    else {
                        var sys_id = generator.AddSystem(coord.x, coord.y, "player base", MathExt.RandInt());


                        /*var jg_id = generator.AddJumpgate(
                            sys_id,
                            null,
                            0,
                            0,
                            0,
                            "jumpgate_01");


                        var curinf = generator.GetSystemByID(sys_id);

                        curinf.jumpgates = jg_id;
                        
                        */
                        //var jumpgate_coords = { x: 0, y: 0 };
                        //res.push({ id: jg_id, coord: jumpgate_coords });
                        ////console.PrintError("player baseate");
                    }

                    curNumOfSystems++;
                    currentCount++;
                    break;
                }
            }
        }
        if (i == tries) {
            console.Print("Couldn't find a place for star in " + tries + " tries.");
            break;
        }
    }
}


//--------------------------------------------------------------------------
// Name:
// Desc: function generates links between created systems
//--------------------------------------------------------------------------
function GenerateLinks(args) {
    var GenerationSteps = 5;
    var AverageJumpsFromSystemToSystem = 4; //4

    var ByDist = {};
    var Systems = generator.GetAllSystems();

    for (var i = 0; i < GenerationSteps; i++) {
        generator.ReportLoadingProgress(i / GenerationSteps);

        for (var j = 0; j < Systems.length; j++) {
            // allocate if none
            var curSysID = Systems[j];
            if (typeof (ByDist[curSysID]) == "undefined") {
                ByDist[curSysID] = generator.GetSystemsByDistanceTo(curSysID);
            }

            // try to connect with current iteration
            var conWithID = ByDist[curSysID][i + 1];

            if (generator.GetJumpsBetweenSystems(curSysID, conWithID) > AverageJumpsFromSystemToSystem) {
                generator.AddSystemsLink(curSysID, conWithID);
            }
        }
    }

    // make system whole
    //generator.FixConnectivity();
}

// Generate tech and danger levels of systems regardless of factions
// (factions influence will be taken into account later)
function GenerateSystemLevels(args, start_coord, start_system_id) {
    var system,
        coord,
        dist,
        fraction,
        fraction2;
    var tech_level,
        danger_level;
    var factionInfo,
        capital_id,
        capital_info;

    // set systems default level according to how far it is from origin
    var systems = generator.GetAllSystems();
    for (var i = 0; i < systems.length; i++) {
        var system_id = systems[i];

        generator.ReportLoadingProgress(i / systems.length);
        system = generator.GetSystemByID(system_id);
        coord = { x: system.coord_x, y: system.coord_y };

        dist = MathExt.Vector2Length(coord, start_coord);
        fraction = dist / 100;

        tech_level = 5 + fraction * 50 + (MathExt.RandDouble() - 0.5) * fraction * 50;
        danger_level = 1 + Math.round(fraction * 70);

        // take faction into account
        if (typeof (system.faction) != "undefined" && system.faction != "none" && system.faction.length > 0) {
            factionInfo = relations.GetFactionInfoByID(system.faction);
            capital_id = relations.GetFactionCapital(system.faction);
            capital_info = generator.GetSystemByID(capital_id);

            dist = MathExt.Vector2Length(coord, { x: capital_info.coord_x, y: capital_info.coord_y });
            fraction2 = 1 - dist / factionInfo.capital_area_of_influence;
            if (fraction2 > 1) {
                fraction2 = 1;
            }
            else if (fraction2 <= 0) {
                fraction2 = 0;
            }

            tech_level += (factionInfo.capital_tech_level - tech_level) * fraction2;
            danger_level += (factionInfo.capital_danger_level - danger_level) * fraction2;
        }

        tech_level = utils.Clamp(tech_level, 1, 100);
        danger_level = utils.Clamp(danger_level, 3, 100);

        generator.SetSystemTechLevel(system_id, Math.round(tech_level));
        generator.SetSystemDangerLevel(system_id, Math.round(danger_level));
    }
}

// Function generates factions in the galaxy
function GenerateFactions(args, start_coord, start_system_id) {
    //----------------------------------------------
    // C A P I T A L S  of factions
    //

    // grab info about factions (from xmls)
    var factions = {};
    //var factions_ids = relations.GetFactions();


    var factionName = "none";
    var fact = relations.GetFactionInfoByID(factionName);
    if (fact.stars_count_value != 0 || fact.stars_count_percent != 0) {
        fact.total_num_of_stars = 1;//Math.ceil(fact.stars_count_value + NumOfSystems * (fact.stars_count_percent / 100));
        //factions[factionName] = fact;
        factions[0] = fact;
        factions.push(fact);
        console.Print("Faction " + factionName + " must have num of stars " + fact.total_num_of_stars);
    }


    /*for (var i = 0; i < factions_ids.length; i++)
    {
		var factionName = factions_ids[i];
		if (generator.IsFactionExcludedFromGeneration(factionName))	
		{
			continue;
		}
		
        var fact = relations.GetFactionInfoByID(factions_ids[i]);
        if (fact.stars_count_value != 0 || fact.stars_count_percent != 0)
        {
            fact.total_num_of_stars = Math.ceil(fact.stars_count_value + NumOfSystems * (fact.stars_count_percent / 100));
            factions[factions_ids[i]] = fact;
            console.Print("Faction " + factions_ids[i] + " must have num of stars " + fact.total_num_of_stars);
        }
    }*/

    var total_stars_needed_to_be_created = 1; //8
    for (var fctt in factions) {
        total_stars_needed_to_be_created += factions[fctt].total_num_of_stars;
    }

    if (NumOfSystems < total_stars_needed_to_be_created) {
        //console.PrintError("Error: total amount of stars (" + NumOfSystems + ") is not enough to fit factions. Fix factions' xmls, or increase total amount of stars (" + total_stars_needed_to_be_created + " at least needed)");
        console.WaitForUserInteraction();
        return;
    }

    var result;
    var tries = 500;
    var capitalPickFails = 0;
    for (var i = 0; i < tries; i++) {
        console.Print("GenerateFactions: Try " + i);
        // pick capitals
        result = GenerateFactions_PickCapitals(factions, start_system_id, start_coord);
        if (!result) {
            capitalPickFails++;
            if (capitalPickFails > 50) {
                MinJumpsBetweenCapitals--;
                capitalPickFails = 0;
                console.Print("GenerateFactions: failed to pick capitals, decreasing requirement of minimum jumps count between capitals to " + MinJumpsBetweenCapitals + ". Start over...");
            }
            else {
                console.Print("GenerateFactions: failed to pick capitals, start over");
            }

            relations.ClearAllInfoOnSystems();
        }
        else {
            // reinit capital pick fails
            capitalPickFails = 0;

            for (var k in factions) {
                relations.SetFactionCapital(k, factions[k].capital);
            }

            // expand factions area
            result = GenerateFactions_CreateFactionsAreas(factions, start_system_id);
            if (!result) {
                console.Print("GenerateFactions: Failed to create factions areas, start over");
                relations.ClearAllInfoOnSystems();
            }
            else {
                console.Print("GenerateFactions: Try " + i + " was successful");
                GenerateFactions_FactionsPostprocessing();
                break; // generated successfully
            }
        }
    }

    if (i == tries) {
        console.Print("GenerateFactions: Error: cannot generate factions areas with specified parameters. Try changing Xmls.");
        return false;
    }

    return true;
}

function FixFactions(args, start_coord, start_system_id) {
    // step 1
    // for all order, freedom and fanatics systems remove all adjacent xengatarn systems
    /*var arr = ["order", "fanatics", "freedom"];
     for(var i = 0; i < arr.length; i++)
     {
     var systems = relations.GetFactionSystems(arr[i]);
     for(var j = 0; j < systems.length; j++)
     {
     var linked = generator.GetLinkedSystems(systems[j]);
     for(var k = 0; k < linked.length; k++)
     {
     var faction = relations.GetSystemFaction(linked[k]);
     if(faction == "aliens")
     {
     //remove faction no matter what!
     relations.SetSystemFaction(linked[k], "none");
     }
     ////console.PrintError(linked[k] + " system " + faction);
     }
     }
     }*/

    // step 2
    // ensure, that passage between the start system and all non-alien systems is available
    var systems = generator.GetAllSystems();
    var startSystemID = spawn.GetSystemID(); //get starting system

    console.Print("Start opening blocked by alies systems");

    for (var i = 0; i < systems.length; i++) {
        var systemId = systems[i];
        if (systemId == startSystemID) {
            continue;
        }

        var system_faction = relations.GetSystemFaction(systemId);
        if (system_faction == "aliens") {
            continue;
        }

        var route = generator.GetRoute(startSystemID, systemId);
        for (var k = 0; k < route.length; k++) {
            var faction = relations.GetSystemFaction(route[k]);
            if (faction == "aliens") {
                //remove faction no matter what!
                relations.SetSystemFaction(route[k], "none");

                var inf = generator.GetSystemByID(route[k]);
                console.Print("Set system " + inf.name + " faction to none");
            }
        }
    }

    console.Print("End opening blocked by alies systems");
}


// Special function to pick capitals according to xml parameters
function GenerateFactions_PickCapitals(factions, start_system_id, start_coord) {
    // variables
    var i;
    var capitals = [];
    capitals.push(start_system_id);
    var maxAttempts = 100;

    for (i in factions) {
        for (var attempt = 0; attempt < maxAttempts; attempt++) {
            // pick by distance from origin (in percents)
            var systemId = generator.GetStarSystemByDistanceFromStarSystem(
                start_system_id,
                factions[i].distance_from_origin_min,
                factions[i].distance_from_origin_max);

            if (systemId == 0) {
                attempt++;
                continue;
            }

            var isOk = true;

            for (var j = 0; j < capitals.length; j++) {
                var anotherCapitalId = capitals[j];
                if (systemId == anotherCapitalId) {
                    console.Print("The same system ID is already used as capital system for another faction");
                    isOk = false;
                    break;
                }

                if (MinJumpsBetweenCapitals
                    > generator.GetJumpsBetweenSystems(anotherCapitalId, systemId)) {
                    console.Print("Too close to another capital system");
                    isOk = false;
                    break;
                }
            }

            if (isOk) {
                capitals.push(systemId);
                factions[i].capital = systemId;
                break;
            }
            else {
                attempt++;
            }
        }

        if (attempt == maxAttempts) {
            console.Print("PickCapitals error: Something went wrong (Capital too close to another?), couldn't pick a capital for " + i);
            // total repick!
            return false;
        }
    }

    return true;
}

// Special function to expand factions areas around picked capitals
function GenerateFactions_CreateFactionsAreas(factions, start_system_id) {
    var bGenerated;
    var bNeeded;
    var sti;
    var cursysfac;
    var i;
    var stars;

    // fill first system
    for (i in factions) {
        factions[i].stars = [factions[i].capital];
        relations.SetSystemFaction(factions[i].capital, i);
    }

    console.Print("At the beginning all factions has their capitals: ");
    for (var br in factions) {
        console.Print(br + " capital is " + factions[br].capital);
    }

    while (true) {
        bGenerated = false;
        bNeeded = false;

        // try to expand every faction by 1 unit (1 link)
        for (i in factions) {
            // lack of stars - needs to be generated
            if (factions[i].stars.length < factions[i].total_num_of_stars) {
                bNeeded = true;

                //console.Print("trying to expand");
                stars = generator.ExpandArea(factions[i].stars);

                //console.Print("expanded to: " + stars.length);
                for (sti = 0; sti < factions[i].total_num_of_stars && sti < stars.length; sti++) {
                    cursysfac = relations.GetSystemFaction(stars[sti]);
                    if (cursysfac.length == 0 && stars[sti] != start_system_id) {
                        relations.SetSystemFaction(stars[sti], i);
                        bGenerated = true;
                        //console.Print(stars[sti] + " faction set to " + i);
                    }
                    else if (cursysfac != i || stars[sti] == start_system_id) {
                        //console.Print(stars[sti] + " already taken");
                        stars.splice(sti, 1);
                        sti--;
                    }
                }
                for (sti = stars.length; sti > factions[i].total_num_of_stars; sti--) {
                    stars.pop();
                } //remove all extra stars

                factions[i].stars = stars;
            }
        }

        // if nothing was generated, it means either all systems properly generated,
        // or error occured, and nothing can be generated
        if (!bGenerated) {
            if (bNeeded) {
                return false;
            }
            else {
                return true;
            }
        }
    }
}


// Function sets faction of unfactioned systems to "none" - default faction
function GenerateFactions_FactionsPostprocessing() {
    // fix systems with no faction
    var systems = generator.GetAllSystems();
    for (var i = 0; i < systems.length; i++) {
        var inf = generator.GetSystemByID(systems[i]);

        // if no faction - set faction to "none"
        if (inf.faction.length == 0) {
            relations.SetSystemFaction(systems[i], "none");
        }
    }
}

// Function, initiating process of creating objects for each system
function GenerateSystemObjects(args) {
    var systems = generator.GetAllSystems();
    for (var i = 0; i < systems.length; i++) {
        generator.ReportLoadingProgress(i / systems.length);
        if (!GenSystemObjects(systems[i], args)) {
            ////console.PrintError("failed generation " + systems[i]);
            return false;
        }
    }

    return true;
}

function ResetPresetsQuantity() {
    for (var i in SystemsPresets) {
        var preset = SystemsPresets[i];
        preset.Quantity = 0;
    }
}
