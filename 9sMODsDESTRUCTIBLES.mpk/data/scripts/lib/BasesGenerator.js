//-------------------------------------------------------
// BasesGenerator
//
// extendable script to generate different types of bases:
// aliens-like, humans-like, etc
//
include(outpost01FloorData.js);
include(outpost01OuterWallData.js);
include(outpost01DoorData.js);
include(outpost01GarbageContainerData.js);
include(outpost01RoboticArmHighData.js);
include(outpost01RoboticArmLowData.js);
include(outpost01FuelReservoirData.js);
include(outpost01RoboticRepairPlatformData.js);

using(console);
using(storage);

include(SC_Station_Tiles_Outpost_01.js);
include(SC_Station_Tiles_Military_01.js);
include(SC_Station_Tiles_Science_01.js);
include(SC_Station_Tiles_Business_01.js);
include(SC_Station_Tiles_Mining_01.js);

var startSysSwtch = -1;
var theBase = [];

var arrayOfRefuelPlatforms =
    [
        "station_platform_refuel_0",
        "station_platform_refuel_45",
        "station_platform_refuel_90",
        "station_platform_refuel_135",
        "station_platform_refuel_180",
        "station_platform_refuel_225",
        "station_platform_refuel_270",
        "station_platform_refuel_315"
    ];

var arrayOfRepairPlatforms =
    [
        "station_platform_repair_0",
        "station_platform_repair_45",
        "station_platform_repair_90",
        "station_platform_repair_135",
        "station_platform_repair_180",
        "station_platform_repair_225",
        "station_platform_repair_270",
        "station_platform_repair_315"
    ];


var initStationInteriorDemo = 0;


var BasesGenerator = {
    /*
     ===================================================================
     Adds solid human bases, which are indestructible
     
     Has two modes:
     1 - "random" (or null):
     In this mode random bases from types are taken in random order. For example,
     from
     {types,quantity}:[a,b,c],5
     it produce
     [a,a,a,a,a] or [a,b,c,c,b].
     2 - "strict":
     In this mode you need to specify all bases you need to be generated. Quantity parameters
     will be ignored. For example, from
     {types,quantity}:[a,b,c],5
     it can give
     [a,b,c] or [b,c,a] or [c,a,b]	
     ===================================================================
     */


    




    AddBasesOfTypes: function (args, types, quantity, mode, customTagClass, switchForDebugTiles) {
        var sys_id = args.sys_id;
        var inf = args.system_info;
        var bases = [];
        var distanceToJumpgate = 50;

        var strictBaseNum = MathExt.RandRange(0, types.length);
        if (mode == "strict") {
            quantity = types.length;
        }

        // add base next to one of jumpgates
        var current_jumpgate = 0;
        var jumpgates = args.jumpgates;

        // create a number of bases
        var tries = 1;
        for (var i = 0; i < quantity; i++) {
            var pickedIndex = 0;
            if (mode == "strict") {
                //pick at least one of each
                pickedIndex = strictBaseNum % types.length;
            }
            else //suppose it's "random" or null
            {
                pickedIndex = MathExt.RandRange(0, types.length);
            }

            var base_xml = types[pickedIndex];

            var tagClass = customTagClass == null ? "none" : customTagClass;
            var bas_coord = 0;

            if (customTagClass == null) {
                // assign class
                if (base_xml.indexOf("mining") >= 0) {
                    tagClass = "mining";
                }
                else if (base_xml.indexOf("outpost") >= 0) {
                    tagClass = "outpost";
                }
                else if (base_xml.indexOf("science") >= 0) {
                    tagClass = "science";
                }
                else if (base_xml.indexOf("business") >= 0) {
                    tagClass = "business";
                }
                else if (base_xml.indexOf("military") >= 0) {
                    tagClass = "military";
                }
                ////console.PrintError("is customTagClass null? " + "BasesGenerator.js");
            }

            ////console.PrintError(tagClass);

            /*// assign class
            if (base_xml.indexOf("mining") >= 0) {
                tagClass = "mining";
            }
            else if (base_xml.indexOf("outpost") >= 0) {
                tagClass = "outpost";
            }
            else if (base_xml.indexOf("science") >= 0) {
                tagClass = "science";
            }
            else if (base_xml.indexOf("business") >= 0) {
                tagClass = "business";
            }
            else if (base_xml.indexOf("military") >= 0) {
                tagClass = "military";
            }*/




            // based on class, define position
            /*if (tagClass == "outpost")
            {
                // try to positionate next to jumpgate
                if (current_jumpgate < jumpgates.length - 1)
                {
                    //console.Print("Base " + i + " outpost - trying to pick near jumpgate " + current_jumpgate);
                    var jag = jumpgates[current_jumpgate];
                    var ang = MathExt.RandDouble() * Math.PI - Math.PI / 2;

                    bas_coord =
                    {
                        x: jag.coord.x + distanceToJumpgate * Math.cos(ang),
                        y: jag.coord.y + distanceToJumpgate * Math.sin(ang)
                    };

                    current_jumpgate++;
                }
            }*/





            if (bas_coord == 0) {
                //generic coordinates pick
                bas_coord = CSGen.GetRandomCoordWithinWorldBounds(args);
                //console.Print("Base " + i + " " + tagClass + " picking coordinates " + bas_coord.x + ";" + bas_coord.y);
            }

            //console.Print("Base " + i + " coordinates picked");

            // check if fits
            // we're checking distances to other bases and jumpgates
            var bFits = true;
            var dist = MathExt.Vector2Length(bas_coord);
            if (dist < CSGen.GetMinDistanceToSun()) {
                bFits = false;
            } //to close to sun

            for (var k = 0; k < bases.length; k++) {
                dist = MathExt.Vector2Length(bases[k].coord, bas_coord);
                if (dist < 70) {
                    bFits = false;
                } //should be no closer then 70 to any other base
            }


            if (jumpgates != null)     /////////////////////////////////////// FIX for 9sMODs ///////////////////////////////////////
            {
                for (var k = 0; k < jumpgates.length; k++) {
                    dist = MathExt.Vector2Length(jumpgates[k].coord, bas_coord);
                    if (dist < distanceToJumpgate - 10) {
                        bFits = false;
                    } //should be no closer then 30 to any jumpgate
                }
            }

            if (!bFits) {
                i--;
                //this try doesn't count
                //console.Print("too close! another try (now " + tries + ")");
                tries++;
                if (tries > 100) {
                    //console.PrintError("Cannot create bases. Tries of base creation are over 100, still cannot find room for it, restart generation process, or change generation parameters.");
                    console.WaitForUserInteraction();
                }
            }
            else {
                strictBaseNum++;
                tries = 1;



                var base_id;


                //in order it goes like this:
                //from GenerateGalaxy.js for the galaxys systems creation 
                //to SystemsPresets.js for the selection of the station type.
                //to BasesGenerator.js for the stations bases creation
                //to GenerateGalaxy.js for the jumpgates creation





                /*
                if (initStationInteriorDemo == 0) {



              







                    /*
                    //console.PrintError("testing 1");

                    if (storage.IsSetGlobal("stationINT" + sys_id))
                    {
                        var randomIndexStationOutpost = Math.floor(Math.random() * arrayOfOutpostStations.length);

                        var getSomeIndex = arrayOfOutpostStations[randomIndexStationOutpost].substring(11, arrayOfOutpostStations[randomIndexStationOutpost].length); //outpost_01_0

                        //var index = indexOfStuff(getSomeIndex);
                        var parsedAngle = parseInt(getSomeIndex);






                        //console.PrintError("testing 2");
                        var baseInteriorGlobalVariable = storage.GetGlobal("stationINT" + sys_id);
                        //containing this from GenerateGalaxy.js => storage.GetGlobal("stationINT" + sys_id1, { id: id, x: coord_x, y: coord_y, sys_id: sys_id1, xml_id: arrayOfOutpostStations[randomIndexStationOutpost], DegAngle: parsedAngle, class: tagClass });
                        //for the station interior. a station
                        base_id = generator.AddBase(
                            baseInteriorGlobalVariable.sys_id,
                            baseInteriorGlobalVariable.x, baseInteriorGlobalVariable.y,
                            baseInteriorGlobalVariable.xml_id,
                            StationNameGenerator.GetName(),
                            { class: tagClass });

                        storage.SetGlobal("stationINT" + sys_id, baseInteriorGlobalVariable);



                        relations.SetBaseFaction(base_id, inf.faction);
                        bases.push({
                            id: base_id,
                            coord: { x: baseInteriorGlobalVariable.x, y: baseInteriorGlobalVariable.y },
                            base_xml: baseInteriorGlobalVariable.xml_id
                        });



                        /*var stationData = storage.GetGlobal("stationINT" + systems[j]);

                        //startingStationID = generator.AddNPCShipToSystem("stationINT", "ai_station_interior", 1, "xml_station_interior", sys_idPlayer, coords.x + 50, coords.y, { class: "stationDialog", someTag: "stationINT", greeting: "terminal", stationID: "stationINT" }); //, unique_id: "stationDialog"
                        var id = generator.AddNPCShipToSystem("station test", "ai_station", 100, "xml_station_exterior", sys_idPlayer, coords.x + 20, coords.y, { someTag: "stationEXT", class: "stationEXT", greeting: "terminal", unique_id: "stationTest", stationID: "player base" });

                        //var radToDegAngle = ;//SC_Utilities.DegreeToRadian(angle);
                        ship.SetRotation(id, stationData.DegAngle);

                        var baseExteriorGlobalVariable = storage.GetGlobal("stationEXT" + sys_id);

                        //var systems = generator.GetAllSystems();

                        //var stationData = storage.GetGlobal("stationEXT" + baseExteriorGlobalVariable.id);

                        //startingStationID = generator.AddNPCShipToSystem("stationINT", "ai_station_interior", 1, "xml_station_interior", sys_idPlayer, coords.x + 50, coords.y, { class: "stationDialog", someTag: "stationINT", greeting: "terminal", stationID: "stationINT" }); //, unique_id: "stationDialog"
                        var id = generator.AddNPCShipToSystem("station test", "ai_station", 100, "xml_station_exterior", baseExteriorGlobalVariable.sys_id, baseInteriorGlobalVariable.x + 20, baseInteriorGlobalVariable.y, { someTag: "stationEXT", class: "stationEXT", greeting: "terminal", unique_id: "stationTest", stationID: "player base" });

                        //var radToDegAngle = ;//SC_Utilities.DegreeToRadian(angle);
                        ship.SetRotation(id, baseExteriorGlobalVariable.DegAngle);

                        baseExteriorGlobalVariable.id = id;

                        storage.SetGlobal("stationEXT" + sys_id, baseExteriorGlobalVariable );


                        /*
                        for (var j = 0; j < systems.length; j++) {
                            var inf = generator.GetSystemByID(systems[j]);
                            if (inf.name == "player base") {

                                var stationData = storage.GetGlobal("stationINT" + systems[j]);

                                //startingStationID = generator.AddNPCShipToSystem("stationINT", "ai_station_interior", 1, "xml_station_interior", sys_idPlayer, coords.x + 50, coords.y, { class: "stationDialog", someTag: "stationINT", greeting: "terminal", stationID: "stationINT" }); //, unique_id: "stationDialog"
                                var id = generator.AddNPCShipToSystem("station test", "ai_station", 100, "xml_station_exterior", sys_idPlayer, baseInteriorGlobalVariable.x + 20, baseInteriorGlobalVariable.y, { someTag: "stationEXT", class: "stationEXT", greeting: "terminal", unique_id: "stationTest", stationID: "player base" });

                                //var radToDegAngle = ;//SC_Utilities.DegreeToRadian(angle);
                                ship.SetRotation(id, stationData.DegAngle);

                                //console.PrintError("spawned station interior");
                                canStartCounter = 1;
                                spawnStation = 1;
                            }
                        }*/




                /*
                base_id = generator.AddBase(
                    sys_id,
                    baseInteriorGlobalVariable.x, baseInteriorGlobalVariable.y,
                    base_xml,
                    StationNameGenerator.GetName(),
                    { class: tagClass });
                relations.SetBaseFaction(base_id, inf.faction);

                bases.push({
                    id: base_id,
                    coord: { x: baseInteriorGlobalVariable.x, y: baseInteriorGlobalVariable.y },
                    base_xml: baseInteriorGlobalVariable.xml_id
                });




            }
            else {


            }*/

                /*if () {

                }*/


                //for the station interior. a station
                /*base_id = generator.AddBase(
                    sys_id,
                    bas_coord.x, bas_coord.y,
                    base_xml,
                    StationNameGenerator.GetName(),
                    { class: tagClass });
                relations.SetBaseFaction(base_id, inf.faction);

                bases.push({
                    id: base_id,
                    coord: bas_coord,
                    base_xml: base_xml
                });*/




                /*var angle = -MathExt.Vector2Angle(system_vec, { x: curinf.coord_x, y: curinf.coord_y });
                var radius = bounds.x * 0.75 * MathExt.RandRangeDouble(0.6, 1.2);
                var coord_x = radius * Math.cos(angle);
                var coord_y = radius * Math.sin(angle);
                var id = generator.AddNPCShipToSystem("station test", "ai_station_interior", 100, "xml_station_interior", sys_id1, coord_x - 5, coord_y + 10, { someTag: "stationINT", class: "stationINT", greeting: "terminal", unique_id: "stationINT" }); //    

                var radToDegAngle = angle;//SC_Utilities.RadianToDegree(angle);
                storage.SetGlobal("stationINT" + sys_id1, { id: id, coordsx: coord_x, coordsy: coord_y, sys_id: sys_id1, xml_id: "xml_station_interior", DegAngle: radToDegAngle });

                //console.PrintError("station ship interior generated");*/

                /*var systems = generator.GetAllSystems();

                for (var j = 0; j < systems.length; j++) {
                    var inf = generator.GetSystemByID(systems[j]);
                    if (inf.name == "player base") {

                        var stationData = storage.GetGlobal("stationINT" + systems[j]);

                        //startingStationID = generator.AddNPCShipToSystem("stationINT", "ai_station_interior", 1, "xml_station_interior", sys_idPlayer, coords.x + 50, coords.y, { class: "stationDialog", someTag: "stationINT", greeting: "terminal", stationID: "stationINT" }); //, unique_id: "stationDialog"
                        //var id = generator.AddNPCShipToSystem("station test", "ai_station", 100, "xml_station_exterior", sys_idPlayer, coords.x + 20, coords.y, { someTag: "stationEXT", class: "stationEXT", greeting: "terminal", unique_id: "stationTest", stationID: "player base" });
                        //var radToDegAngle = ;//SC_Utilities.DegreeToRadian(angle);
                        //ship.SetRotation(id, stationData.DegAngle);

                        /*base_id = generator.AddBase(
                            sys_id,
                            stationData.coordsx, stationData.coordsy,
                            stationData.xml_id,
                            StationNameGenerator.GetName(),
                            { class: tagClass });


                        relations.SetBaseFaction(base_id, inf.faction);

                        bases.push({
                            id: base_id,
                            coord: bas_coord,
                            base_xml: stationData.xml_id
                        });


                        //console.PrintError("spawned station interior");
                        canStartCounter = 1;
                        spawnStation = 1;
                        break;
                    }
                }

                initStationInteriorDemo = 1;
            }
            else {
                /*base_id = generator.AddBase(
                    sys_id,
                    bas_coord.x, bas_coord.y,
                    base_xml,
                    StationNameGenerator.GetName(),
                    { class: tagClass });
                relations.SetBaseFaction(base_id, inf.faction);

                bases.push({
                    id: base_id,
                    coord: bas_coord,
                    base_xml: base_xml
                });
            }*/



                //var someTester = "tester_01";
                ////console.PrintError(someTester.substring(0,1));

                //var theBase = { id: base_id, coord: bas_coord, base_xml: base_xml, sys_id: sys_id };            





                var parsedAngle;

                if (tagClass == "outpost" && base_xml.length >= 10) //base_xml.indexOf("outpost") >= 0)
                {

                    if (storage.IsSetGlobal("stationINT" + sys_id))
                    {
                     
                 


                        //console.PrintError("setting stationINT data");
                        var baseInteriorGlobalVariable = storage.GetGlobal("stationINT" + sys_id);

                        //console.PrintError("global variable is accessible");


                        var getSomeIndex = base_xml.substring(11, base_xml.length); //outpost_01_0

                        var index = indexOfStuff(getSomeIndex);

                        /*if (index == 0)
                        {
                            index = 0;
                        }
                        else if (index == 1)
                        {
                            index = 5;
                        }
                        else if (index == 2)
                        {
                            index = 6;
                        }
                        else if (index == 3)
                        {
                            index = 7;
                        }
                        else if (index == 4)
                        {
                            index = 4;
                        }
                        else if (index == 5)
                        {
                            index = 1;
                        }
                        else if (index == 6)
                        {
                            index = 2;
                        }
                        else if (index == 7)
                        {
                            index = 3;
                        }*/

                        base_id = generator.AddBase(
                            sys_id,
                            bas_coord.x, bas_coord.y,
                            base_xml,
                            StationNameGenerator.GetName(),
                            { class: tagClass });
                        relations.SetBaseFaction(base_id, inf.faction);

                        bases.push({
                            id: base_id,
                            coord: bas_coord,
                            base_xml: base_xml
                        });




                        //PLANET DECORATION SPAWNING
                        //PLANET DECORATION SPAWNING
                        //PLANET DECORATION SPAWNING
                        //PLANET DECORATION SPAWNING
                        var someCoordsEarth = { x: bas_coord.x + 85, y: bas_coord.y };
                        someCoordsEarth.y += 0;//40

                        var decoID1 = generator.AddDecoration(
                            baseInteriorGlobalVariable.sys_id_link,
                            "planet_earth_night",
                            someCoordsEarth,
                            200, //175
                            { X: 0, Y: 0, Z: 0 },
                            { X: 0, Y: 0, Z: -1 },
                            10); //7
                        //PLANET DECORATION SPAWNING
                        //PLANET DECORATION SPAWNING
                        //PLANET DECORATION SPAWNING
                        //PLANET DECORATION SPAWNING


                        var decoID1 = generator.AddDecoration(
                            baseInteriorGlobalVariable.sys_id_link,
                            "planet_earth_grid",
                            someCoordsEarth,
                            200, //175
                            { X: 0, Y: 0, Z: 0 },
                            { X: 0, Y: 0, Z: -1 },
                            11); //7


                        /*//var distance = MathExt.RandRangeDouble(current.distance_from, current.distance_to);
                        var someCoordsEarth = { x: bas_coord.x + 85, y: bas_coord.y };
                        someCoordsEarth.y += 40;
                        var decoID0 = generator.AddDecoration(
                            sys_id,
                            "planet_earth_night",
                            someCoordsEarth,
                            175,
                            { X: 0, Y: 0, Z: 0 },
                            { X: 0, Y: 0, Z: 1 },
                            7);*/

                       //storage.SetGlobal("planet earth INT" + sys_id2, { id: null, x: system_vec.x, y: system_vec.y, sys_id: sys_id2, sys_id_link: sys_id0, xml_id: "planet_earth_night", DegAngle: parsedAngle, class: tagClass });
                       // storage.SetGlobal("planet earth EXT" + sys_id2, { id: null, x: system_vec.x, y: system_vec.y, sys_id: sys_id2, sys_id_link: sys_id1, xml_id: "planet_earth_night", DegAngle: parsedAngle, class: tagClass });
                 
                        var baseExteriorGlobalVariable = storage.GetGlobal("stationEXT" + baseInteriorGlobalVariable.sys_id_link);

                        var glob = storage.GetGlobal("planet earth INT" + baseInteriorGlobalVariable.sys_id);
                        glob.x = someCoordsEarth.x;
                        glob.y = someCoordsEarth.y;
                        glob.id = null;//decoID0;
                        storage.SetGlobal("planet earth INT" + baseInteriorGlobalVariable.sys_id, glob);

                        glob = storage.GetGlobal("planet earth EXT" + baseInteriorGlobalVariable.sys_id_link);
                        glob.x = someCoordsEarth.x;
                        glob.y = someCoordsEarth.y;
                        glob.id = null;//decoID1;
                        storage.SetGlobal("planet earth EXT" + baseInteriorGlobalVariable.sys_id_link, glob);
                        //PLANET DECORATION SPAWNING
                        //PLANET DECORATION SPAWNING

                        baseInteriorGlobalVariable.id = base_id;
                        baseInteriorGlobalVariable.x = bas_coord.x;
                        baseInteriorGlobalVariable.y = bas_coord.y;
                        baseInteriorGlobalVariable.baseID = base_id;
                        storage.SetGlobal("stationINT" + sys_id, baseInteriorGlobalVariable);


                        parsedAngle = parseInt(getSomeIndex);

                        var tempCoordRefuelX = bas_coord.x - 3.5;
                        var tempCoordRefuelY = bas_coord.y - 23;
                        var tempCoordRefuel = { x: tempCoordRefuelX, y: tempCoordRefuelY };

                        var tempCoordRepairX = bas_coord.x + 3;
                        var tempCoordRepairY = bas_coord.y - 23;
                        var tempCoordRepair = { x: tempCoordRepairX, y: tempCoordRepairY };

                        var rotatedCoordRefuel = RotatePoint(tempCoordRefuel, bas_coord, parsedAngle);
                        var rotatedCoordRepair = RotatePoint(tempCoordRepair, bas_coord, parsedAngle);

                        //TO READD
                        //var StationRefuelPlatform = generator.AddSpecialObject(sys_id, rotatedCoordRefuel.x, rotatedCoordRefuel.y, arrayOfRefuelPlatforms[index], 0);
                        //var StationRepairPlatform = generator.AddSpecialObject(sys_id, rotatedCoordRepair.x, rotatedCoordRepair.y, arrayOfRepairPlatforms[index], 0);
                        //TO READD

                        //// add turrets
                        var turret_radius = 35;
                        var level = inf.danger_level + Math.round(MathExt.RandDouble() * 4 - 2);
                        level = utils.Clamp(level, 1, 100);
                        //var angle_offset = 0.0;

                        //TO READD
                        //TO READD
                        //TO READD
                        //NpcGenerator.SpawnTurretsOfType(args, bas_coord, 4, level, turret_radius, "Turret", "special_human_turret", inf.faction, { class: "turret", non_talkable: true }, parsedAngle, base_xml, sys_id, base_id);					
                        theBase = { id: base_id, coord: bas_coord, base_xml: base_xml, sys_id: sys_id, rot: parsedAngle };// widthL: 21, widthR: 21, heightT: 19, heightB: 42, 
                        storage.SetGlobal("system_" + sys_id + "_base_" + base_id + "_xmlStationType", theBase);
                        //TO READD
                        //TO READD
                        //TO READD

                        //if(switchForDebugTiles == 1)
                        //{
                        //	SC_Station_Tiles_Outpost_01.buildTiles(theBase);
                        //}

                        if (theBase == null) {
                            //console.PrintError("theBase null00: " + scriptCounter);
                        }
                        else {

                            if (theBase.base_xml == null) {
                                //console.PrintError("theBase.base_xml null00: " + scriptCounter);
                            }
                            else if (theBase.id == null) {
                                //console.PrintError("string00 theBase.id null BasesGenerator.js " + theBase);
                            }
                            else {
                                //console.PrintError("theBase BasesGenerator.js !null");
                            }
                        }

                        /*if (switchForDebugTiles == -1)
                        {
                            ///BUILD TILES////////////////////////
                            //////////////////////////////////////
                            SC_Station_Tiles_Outpost_01.buildTiles(theBase);
                            //////////////////////////////////////
                            //////////////////////////////////////
                            storage.SetGlobal("player_starting_system_base_", base_id);
                        }*/

                        //var stationData = { baseID: theBase.id, sys_id: theBase.sys_id, coord: theBase.coord, xml_id: theBase.base_xml, widthL: -1, widthR: -1, heightT: -1, heightB: -1, grid: [], rot: 0, visualTiles: -1 };
                        //storage.SetGlobal("station_tiles" + theBase.id, stationData);



                        if (switchForDebugTiles == -1) {
                            if (theBase == null) {
                                //console.PrintError("theBase null: " + scriptCounter);
                            }
                            else {
                                if (theBase.base_xml == null) {
                                    //console.PrintError("theBase.base_xml null: " + scriptCounter);
                                }
                                else {

                                    //console.PrintError("theBase.base_xml !null: " + theBase.base_xml + " id: " + theBase.id);
                                    ////console.PrintError("player_starting_system_base_: " + base_id);
                                }

                                var stationData = SC_Station_Tiles_Outpost_01.buildTiles(theBase, theBase.id);
                                storage.SetGlobal("station_tiles" + theBase.id, stationData);

                                if (!storage.IsSetGlobal("player_starting_system_base_")) {

                                    storage.SetGlobal("player_starting_system_base_", base_id);
                                    //console.PrintError("setting player_starting_system_base_: " + base_id);
                                }
                            }
                            switchForDebugTiles = 1;
                        }






                        //console.PrintError("setting stationEXT data");
                        var baseExteriorGlobalVariable = storage.GetGlobal("stationEXT" + baseInteriorGlobalVariable.sys_id_link);

                        //startingStationID = generator.AddNPCShipToSystem("stationINT", "ai_station_interior", 1, "xml_station_interior", sys_idPlayer, coords.x + 50, coords.y, { class: "stationDialog", someTag: "stationINT", greeting: "terminal", stationID: "stationINT" }); //, unique_id: "stationDialog"
                        var id = generator.AddNPCShipToSystem("station test", "ai_station", 100, "xml_station_exterior", baseInteriorGlobalVariable.sys_id_link, bas_coord.x, bas_coord.y, { someTag: "stationEXT", class: "stationEXT", greeting: "terminal", unique_id: "stationTest", stationID: "player base", non_talkable: true  });

                        //var radToDegAngle = ;//SC_Utilities.DegreeToRadian(angle);
                        ship.SetRotation(id, baseExteriorGlobalVariable.DegAngle);

                        baseExteriorGlobalVariable.id = id;
                        baseExteriorGlobalVariable.x = bas_coord.x;
                        baseExteriorGlobalVariable.y = bas_coord.y;

                        storage.SetGlobal("stationEXT" + baseInteriorGlobalVariable.sys_id_link, baseExteriorGlobalVariable);

                        //var id = generator.AddNPCShipToSystem("station test", "ai_station", 100, "outpost_01_roof", sys_id, bas_coord.x, bas_coord.y, { someTag: "stationEXT", class: "stationEXT", greeting: "terminal", unique_id: "stationTest", stationID: "player base" });
          
                        //var startsysdata = { id: start_system_id, name: start_system };
                        //var startsysdata = storage.GetGlobal("startingsystem");

                    



                        //spawn.SetCoordinates(bas_coord.x, bas_coord.y);
                        //args.start_system_id = start_system_id;
                        //spawn.SetSystemID(baseInteriorGlobalVariable.sys_id);

                        //SETTING STARTING SYSTEM GLOBAL VARIABLE A SECOND TIME, BUT WITH DIFFERENT DATA. setglobal first called in GenerateGalaxy.js
                        var startsysdata = storage.GetGlobal("startingsystem");
                        if (startsysdata.id == sys_id || startsysdata.id == baseInteriorGlobalVariable.sys_id_link) {

                            //STATION ROOF SPAWN
                            var item_list = "droplist_empty";
                            generator.AddContainer(sys_id, bas_coord.x, bas_coord.y, "outpost_01_roof", item_list, { itemlist: item_list, class: "outpost_01_roof" });
                            storage.SetGlobal("stationINT" + sys_id, baseInteriorGlobalVariable);

                            if (startsysdata.id == sys_id) {
                                var someStartingBase = { sys_id: startsysdata.id, baseID: base_id, sys_name: startsysdata.sys_name, coords: bas_coord };
                                storage.SetGlobal("startingsystem", someStartingBase);
                            }
                            else if (startsysdata.id == baseInteriorGlobalVariable.sys_id_link) {
                                var someStartingBase = { sys_id: baseInteriorGlobalVariable.sys_id_link, baseID: base_id, sys_name: inf.name, coords: bas_coord };
                                storage.SetGlobal("startingsystem", someStartingBase);
                            }
                        }
                        //SETTING STARTING SYSTEM GLOBAL VARIABLE A SECOND TIME, BUT WITH DIFFERENT DATA. setglobal first called in GenerateGalaxy.js









                        //DESTRUCTIBLE STATIONS MOD
                        //DESTRUCTIBLE STATIONS MOD
                        //DESTRUCTIBLE STATIONS MOD

                        //CREATING THE DECORATIONS FLOOR TILES THAT WILL BE DESTRUCTIBLE LATER WITH SCRIPT DISTANCE TO PROJECTILES EXPLOSIONS OR WALLS OR SHIPS EXPLOSIONS... TODO.
                        //middle platform decorations floor tiles. the pivot point is set to the pivot point of the station for each one of those floor tiles compared to the ships/hulls that have their pivot point right on them.
                        var outpostFloor = outpost01FloorData.getOutpost01Floor();
                        var newBasCoord = { x: bas_coord.x, y: bas_coord.y }; // - 20 
                        for (var c = 0; c < outpostFloor.length; c++) {
                            var middlePlatform = generator.AddDecoration(sys_id, outpostFloor[c], { x: newBasCoord.x, y: newBasCoord.y }, 0, { x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 }, 1);
                        }
                        //middle platform decorations floor tiles. the pivot point is set to the pivot point of the station for each one of those floor tiles compared to the ships/hulls that have their pivot point right on them.
                        //CREATING THE DECORATIONS FLOOR TILES THAT WILL BE DESTRUCTIBLE LATER WITH SCRIPT DISTANCE TO PROJECTILES EXPLOSIONS OR WALLS OR SHIPS EXPLOSIONS... TODO.









                        var optionStopOrInstantStop = 1;
                        var optionString = "";

                        if (optionStopOrInstantStop == 0)
                        {
                            optionString = "SC_AI_stop";
                        }
                        else if (optionStopOrInstantStop == 1) {
                            optionString = "SC_AI_instantstop";
                        }












                        //SPAWNING DOORS OF THE STATION.
                        //SPAWNING DOORS OF THE STATION.
                        //SPAWNING DOORS OF THE STATION.
                        var starter = 0;
                        var offsetY = 0;
                        var outpostDoor = outpost01DoorData.getOutpost01Door();
                        for (var c = 0; c < outpostDoor.length; c++) {

                            var currentYIndex = outpostDoor[c].indexOf("posY");
                            var getSomeOriPosForY = outpostDoor[c].substring(currentYIndex + 4, outpostDoor[c].length - 1);
                            getSomeOriPosForY = getSomeOriPosForY.replace("_", ".");
                            /*if (starter == 1) {
                                offsetY = 2.25;
                            }*/
                            offsetY = -0.75; //0.75
                            //var doors = generator.AddDecoration(sys_id, outpostDoor[c], { x: newBasCoord.x, y: newBasCoord.y }, 0, { x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 }, 1);
                            var indexX = outpostDoor[c].indexOf("posX");
                            var indexY = outpostDoor[c].indexOf("posY");

                            var diff = indexY - indexX;

                            var getSomePosForX = outpostDoor[c].substring(indexX + 4, indexX + diff);

                            //var getSomePosForY = outpostOuterWall[c].substring(indexY + 4, outpostOuterWall[c].length - 1);

                            getSomePosForX = getSomePosForX.replace("_", ".");
                            //getSomePosForY = getSomePosForY.replace("_", ".");

                            var somePosX = newBasCoord.x + parseInt(getSomePosForX);
                            var somePosY = newBasCoord.y + parseInt(getSomeOriPosForY) + (offsetY); //parseInt(getSomePosForY); // + (offsetY * c)

                            //console.PrintError("x: " + getSomePosForX + " y: " + getSomeOriPosForY);

                            var somePos = { x: somePosX, y: somePosY };

                            var rotatedCoordWall = RotatePoint(somePos, newBasCoord, parsedAngle);


                            var angleN = SC_Utilities.normalizedegrees(parsedAngle); //????????

                            var degToRad = SC_Utilities.DegreeToRadian(angleN);

                            var someOuterWall = generator.AddNPCShipToSystem(
                                "", optionString, 1, outpostDoor[c],
                                sys_id,
                                rotatedCoordWall.x,
                                rotatedCoordWall.y,
                                { class: "turret", non_talkable: true });
                            ship.SetRotation(someOuterWall, degToRad);
                            starter = 1;
                        }
                        //SPAWNING DOORS OF THE STATION.
                        //SPAWNING DOORS OF THE STATION.
                        //SPAWNING DOORS OF THE STATION.



                        //SPAWNING GARBAGE CONTAINERS OF THE STATION.
                        //SPAWNING GARBAGE CONTAINERS OF THE STATION.
                        //SPAWNING GARBAGE CONTAINERS OF THE STATION.
                        var starter = 0;
                        var offsetY = 0;
                        var outpostGarbageContainerData = outpost01GarbageContainerData.getOutpost01GarbageContainerData();
                        for (var c = 0; c < outpostGarbageContainerData.length; c++) {

                            var currentYIndex = outpostGarbageContainerData[c].indexOf("posY");
                            var getSomeOriPosForY = outpostGarbageContainerData[c].substring(currentYIndex + 4, outpostGarbageContainerData[c].length - 1);
                            getSomeOriPosForY = getSomeOriPosForY.replace("_", ".");
                            /*if (starter == 1) {
                                offsetY = 2.25;
                            }*/
                            offsetY = -0.75; //0.75
                            //var doors = generator.AddDecoration(sys_id, outpostGarbageContainerData[c], { x: newBasCoord.x, y: newBasCoord.y }, 0, { x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 }, 1);
                            var indexX = outpostGarbageContainerData[c].indexOf("posX");
                            var indexY = outpostGarbageContainerData[c].indexOf("posY");

                            var diff = indexY - indexX;

                            var getSomePosForX = outpostGarbageContainerData[c].substring(indexX + 4, indexX + diff);

                            //var getSomePosForY = outpostOuterWall[c].substring(indexY + 4, outpostOuterWall[c].length - 1);

                            getSomePosForX = getSomePosForX.replace("_", ".");
                            //getSomePosForY = getSomePosForY.replace("_", ".");

                            var somePosX = newBasCoord.x + parseInt(getSomePosForX);
                            var somePosY = newBasCoord.y + parseInt(getSomeOriPosForY) + (offsetY); //parseInt(getSomePosForY); // + (offsetY * c)

                            //console.PrintError("x: " + getSomePosForX + " y: " + getSomeOriPosForY);

                            var somePos = { x: somePosX, y: somePosY };

                            var rotatedCoordWall = RotatePoint(somePos, newBasCoord, parsedAngle);


                            var angleN = SC_Utilities.normalizedegrees(parsedAngle); //????????

                            var degToRad = SC_Utilities.DegreeToRadian(angleN);

                            var someOuterWall = generator.AddNPCShipToSystem(
                                "", optionString, 1, outpostGarbageContainerData[c],
                                sys_id,
                                rotatedCoordWall.x,
                                rotatedCoordWall.y,
                                { class: "robotic arm", non_talkable: true });
                            ship.SetRotation(someOuterWall, degToRad);
                            starter = 1;
                        }
                        //SPAWNING GARBAGE CONTAINERS OF THE STATION.
                        //SPAWNING GARBAGE CONTAINERS OF THE STATION.
                        //SPAWNING GARBAGE CONTAINERS OF THE STATION.


                        //SPAWNING REPAIR ROBOTIC ARM HIGH OF THE STATION.
                        //SPAWNING REPAIR ROBOTIC ARM HIGH OF THE STATION.
                        //SPAWNING REPAIR ROBOTIC ARM HIGH OF THE STATION.
                        var starter = 0;
                        var offsetY = 0;
                        var outpostRoboticArmHighData = outpost01RoboticArmHighData.getOutpost01RoboticArmHighData();
                        for (var c = 0; c < outpostRoboticArmHighData.length; c++) {

                            var currentYIndex = outpostRoboticArmHighData[c].indexOf("posY");
                            var getSomeOriPosForY = outpostRoboticArmHighData[c].substring(currentYIndex + 4, outpostRoboticArmHighData[c].length - 1);
                            getSomeOriPosForY = getSomeOriPosForY.replace("_", ".");
                            /*if (starter == 1) {
                                offsetY = 2.25;
                            }*/
                            offsetY = -0.75; //0.75
                            //var doors = generator.AddDecoration(sys_id, outpostRoboticArmHighData[c], { x: newBasCoord.x, y: newBasCoord.y }, 0, { x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 }, 1);
                            var indexX = outpostRoboticArmHighData[c].indexOf("posX");
                            var indexY = outpostRoboticArmHighData[c].indexOf("posY");

                            var diff = indexY - indexX;

                            var getSomePosForX = outpostRoboticArmHighData[c].substring(indexX + 4, indexX + diff);

                            //var getSomePosForY = outpostOuterWall[c].substring(indexY + 4, outpostOuterWall[c].length - 1);

                            getSomePosForX = getSomePosForX.replace("_", ".");
                            //getSomePosForY = getSomePosForY.replace("_", ".");

                            var somePosX = newBasCoord.x + parseInt(getSomePosForX);
                            var somePosY = newBasCoord.y + parseInt(getSomeOriPosForY) + (offsetY); //parseInt(getSomePosForY); // + (offsetY * c)

                            //console.PrintError("x: " + getSomePosForX + " y: " + getSomeOriPosForY);

                            var somePos = { x: somePosX, y: somePosY };

                            var rotatedCoordWall = RotatePoint(somePos, newBasCoord, parsedAngle);


                            var angleN = SC_Utilities.normalizedegrees(parsedAngle); //????????

                            var degToRad = SC_Utilities.DegreeToRadian(angleN);

                            var someOuterWall = generator.AddNPCShipToSystem(
                                "", optionString, 1, outpostRoboticArmHighData[c],
                                sys_id,
                                rotatedCoordWall.x,
                                rotatedCoordWall.y,
                                { class: "robotic arm", non_talkable: true });
                            ship.SetRotation(someOuterWall, degToRad);
                            starter = 1;
                        }
                        //SPAWNING REPAIR ROBOTIC ARM HIGH OF THE STATION.
                        //SPAWNING REPAIR ROBOTIC ARM HIGH OF THE STATION.
                        //SPAWNING REPAIR ROBOTIC ARM HIGH OF THE STATION.






                        //SPAWNING REPAIR ROBOTIC ARM LOW OF THE STATION.
                        //SPAWNING REPAIR ROBOTIC ARM LOW OF THE STATION.
                        //SPAWNING REPAIR ROBOTIC ARM LOW OF THE STATION.
                        var starter = 0;
                        var offsetY = 0;
                        var outpostRoboticArmLowData = outpost01RoboticArmLowData.getOutpost01RoboticArmLowData();
                        for (var c = 0; c < outpostRoboticArmLowData.length; c++)
                        {
                            var currentYIndex = outpostRoboticArmLowData[c].indexOf("posY");
                            var getSomeOriPosForY = outpostRoboticArmLowData[c].substring(currentYIndex + 4, outpostRoboticArmLowData[c].length - 1);
                            getSomeOriPosForY = getSomeOriPosForY.replace("_", ".");
                            /*if (starter == 1) {
                                offsetY = 2.25;
                            }*/
                            //offsetY = -0.75; //0.75
                            //var doors = generator.AddDecoration(sys_id, outpostRoboticArmLowData[c], { x: newBasCoord.x, y: newBasCoord.y }, 0, { x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 }, 1);
                            var indexX = outpostRoboticArmLowData[c].indexOf("posX");
                            var indexY = outpostRoboticArmLowData[c].indexOf("posY");

                            var diff = indexY - indexX;

                            var getSomePosForX = outpostRoboticArmLowData[c].substring(indexX + 4, indexX + diff);

                            //var getSomePosForY = outpostOuterWall[c].substring(indexY + 4, outpostOuterWall[c].length - 1);

                            getSomePosForX = getSomePosForX.replace("_", ".");
                            //getSomePosForY = getSomePosForY.replace("_", ".");

                            var somePosX = newBasCoord.x + parseInt(getSomePosForX);
                            var somePosY = newBasCoord.y + parseInt(getSomeOriPosForY) + (offsetY); //parseInt(getSomePosForY); // + (offsetY * c)

                            //console.PrintError("x: " + getSomePosForX + " y: " + getSomeOriPosForY);

                            var somePos = { x: somePosX, y: somePosY };

                            var rotatedCoordWall = RotatePoint(somePos, newBasCoord, parsedAngle);


                            var angleN = SC_Utilities.normalizedegrees(parsedAngle); //????????

                            var degToRad = SC_Utilities.DegreeToRadian(angleN);

                            var someOuterWall = generator.AddNPCShipToSystem(
                                "", optionString, 1, outpostRoboticArmLowData[c],
                                sys_id,
                                rotatedCoordWall.x,
                                rotatedCoordWall.y,
                                { class: "robotic arm", non_talkable: true });
                            ship.SetRotation(someOuterWall, degToRad);
                            starter = 1;
                        }
                        //SPAWNING REPAIR ROBOTIC ARM LOW OF THE STATION.
                        //SPAWNING REPAIR ROBOTIC ARM LOW OF THE STATION.
                        //SPAWNING REPAIR ROBOTIC ARM LOW OF THE STATION.




                        //SPAWNING FUEL RESERVOIR OF THE STATION.
                        //SPAWNING FUEL RESERVOIR OF THE STATION.
                        //SPAWNING FUEL RESERVOIR OF THE STATION.
                        var starter = 0;
                        var offsetY = 0;
                        var outpostFuelReservoir = outpost01FuelReservoirData.getOutpost01FuelReservoirData();
                        for (var c = 0; c < outpostFuelReservoir.length; c++) {

                            var currentYIndex = outpostFuelReservoir[c].indexOf("posY");
                            var getSomeOriPosForY = outpostFuelReservoir[c].substring(currentYIndex + 4, outpostFuelReservoir[c].length - 1);
                            getSomeOriPosForY = getSomeOriPosForY.replace("_", ".");
                            /*if (starter == 1) {
                                offsetY = 2.25;
                            }*/
                            offsetY = -0.75; //0.75
                            //var doors = generator.AddDecoration(sys_id, outpostFuelReservoir[c], { x: newBasCoord.x, y: newBasCoord.y }, 0, { x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 }, 1);
                            var indexX = outpostFuelReservoir[c].indexOf("posX");
                            var indexY = outpostFuelReservoir[c].indexOf("posY");

                            var diff = indexY - indexX;

                            var getSomePosForX = outpostFuelReservoir[c].substring(indexX + 4, indexX + diff);

                            //var getSomePosForY = outpostOuterWall[c].substring(indexY + 4, outpostOuterWall[c].length - 1);

                            getSomePosForX = getSomePosForX.replace("_", ".");
                            //getSomePosForY = getSomePosForY.replace("_", ".");

                            var somePosX = newBasCoord.x + parseInt(getSomePosForX);
                            var somePosY = newBasCoord.y + parseInt(getSomeOriPosForY) + (offsetY); //parseInt(getSomePosForY); // + (offsetY * c)

                            //console.PrintError("x: " + getSomePosForX + " y: " + getSomeOriPosForY);

                            var somePos = { x: somePosX, y: somePosY };

                            var rotatedCoordWall = RotatePoint(somePos, newBasCoord, parsedAngle);


                            var angleN = SC_Utilities.normalizedegrees(parsedAngle); //????????

                            var degToRad = SC_Utilities.DegreeToRadian(angleN);

                            var someOuterWall = generator.AddNPCShipToSystem(
                                "", optionString, 1, outpostFuelReservoir[c],
                                sys_id,
                                rotatedCoordWall.x,
                                rotatedCoordWall.y,
                                { class: "robotic arm", non_talkable: true });
                            ship.SetRotation(someOuterWall, degToRad);
                            starter = 1;
                        }
                        //SPAWNING FUEL RESERVOIR OF THE STATION.
                        //SPAWNING FUEL RESERVOIR OF THE STATION.
                        //SPAWNING FUEL RESERVOIR OF THE STATION.


                        
                        //SPAWNING ROBOTIC REPAIR STATION OF THE STATION.
                        //SPAWNING ROBOTIC REPAIR STATION OF THE STATION.
                        //SPAWNING ROBOTIC REPAIR STATION OF THE STATION.
                        var starter = 0;
                        var offsetY = 0;
                        var outpostRoboticRepairPlatform = outpost01RoboticRepairPlatformData.getOutpost01RoboticRepairPlatformData();
                        for (var c = 0; c < outpostRoboticRepairPlatform.length; c++) {

                            var currentYIndex = outpostRoboticRepairPlatform[c].indexOf("posY");
                            var getSomeOriPosForY = outpostRoboticRepairPlatform[c].substring(currentYIndex + 4, outpostRoboticRepairPlatform[c].length - 1);
                            getSomeOriPosForY = getSomeOriPosForY.replace("_", ".");
                            /*if (starter == 1) {
                                offsetY = 2.25;
                            }*/
                            offsetY = -0.75; //0.75
                            //var doors = generator.AddDecoration(sys_id, outpostRoboticRepairPlatform[c], { x: newBasCoord.x, y: newBasCoord.y }, 0, { x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 }, 1);
                            var indexX = outpostRoboticRepairPlatform[c].indexOf("posX");
                            var indexY = outpostRoboticRepairPlatform[c].indexOf("posY");

                            var diff = indexY - indexX;

                            var getSomePosForX = outpostRoboticRepairPlatform[c].substring(indexX + 4, indexX + diff);

                            //var getSomePosForY = outpostOuterWall[c].substring(indexY + 4, outpostOuterWall[c].length - 1);

                            getSomePosForX = getSomePosForX.replace("_", ".");
                            //getSomePosForY = getSomePosForY.replace("_", ".");

                            var somePosX = newBasCoord.x + parseInt(getSomePosForX);
                            var somePosY = newBasCoord.y + parseInt(getSomeOriPosForY) + (offsetY); //parseInt(getSomePosForY); // + (offsetY * c)

                            //console.PrintError("x: " + getSomePosForX + " y: " + getSomeOriPosForY);

                            var somePos = { x: somePosX, y: somePosY };

                            var rotatedCoordWall = RotatePoint(somePos, newBasCoord, parsedAngle);


                            var angleN = SC_Utilities.normalizedegrees(parsedAngle); //????????

                            var degToRad = SC_Utilities.DegreeToRadian(angleN);

                            var someOuterWall = generator.AddNPCShipToSystem(
                                "", optionString, 1, outpostRoboticRepairPlatform[c],
                                sys_id,
                                rotatedCoordWall.x,
                                rotatedCoordWall.y,
                                { class: "turret", non_talkable: true });
                            ship.SetRotation(someOuterWall, degToRad);
                            starter = 1;
                        }
                        //SPAWNING ROBOTIC REPAIR STATION OF THE STATION.
                        //SPAWNING ROBOTIC REPAIR STATION OF THE STATION.
                        //SPAWNING ROBOTIC REPAIR STATION OF THE STATION.

                        









                    /*var outpostDoor = outpost01DoorData.getOutpost01Door();

                    var starter = 0;
                    var offsetY = 0;

                    var currentYIndex = outpostDoor[0].indexOf("posY");
                    var getSomeOriPosForY = outpostDoor[0].substring(currentYIndex + 4, outpostDoor[0].length - 1);
                    getSomeOriPosForY = getSomeOriPosForY.replace("_", ".");

                    for (var c = 0; c < outpostDoor.length; c++) {
                        if (starter == 1) {
                            offsetY = 2.25;
                        }


                        var indexX = outpostDoor[c].indexOf("posX");
                        var indexY = outpostDoor[c].indexOf("posY");

                        var diff = indexY - indexX;

                        var getSomePosForX = outpostDoor[c].substring(indexX + 4, indexX + diff);
                        //console.PrintError("xpos: " + getSomePosForX);
                        //var getSomePosForY = outpostOuterWall[c].substring(indexY + 4, outpostOuterWall[c].length - 1);

                        getSomePosForX = getSomePosForX.replace("_", ".");
                        //getSomePosForY = getSomePosForY.replace("_", ".");

                        var somePosX = newBasCoord.x + parseInt(getSomePosForX);
                        var somePosY = newBasCoord.y + (parseInt(getSomeOriPosForY)); //parseInt(getSomePosForY); // + (offsetY * c)

                        var somePos = { x: somePosX, y: somePosY };

                        var rotatedCoordWall = RotatePoint(somePos, newBasCoord, 90);

                        var someOuterWall = generator.AddNPCShipToSystem(
                            "", "SC_NO_AI", 1, outpostDoor[c],
                            sys_id,
                            somePos.x,
                            somePos.y,
                            { class: "turret", non_talkable: true });

                        ship.SetRotation(someOuterWall, 0);
                        ////console.PrintError(coords.x + "__" + coords.y);
                        starter = 1;
                    }*/










                    /*var outpostOuterWallRightBottom = outpost01OuterWallData.getOutpost01OuterWallRightBottom();
                    var outpostOuterWallLeftBottom = outpost01OuterWallData.getOutpost01OuterWallLeftBottom();
                    var outpostOuterWallLeftTop = outpost01OuterWallData.getOutpost01OuterWallLeftTop();
                    var outpostOuterWallRightTop = outpost01OuterWallData.getOutpost01OuterWallRightTop();
 
 
                    var starter = 0;
                    var offsetY = 0;
 
                    var currentYIndex = outpostOuterWallRightBottom[0].indexOf("posY")
                    var getSomeOriPosForY = outpostOuterWallRightBottom[0].substring(currentYIndex + 4, outpostOuterWallRightBottom[0].length - 1);
                    getSomeOriPosForY = getSomeOriPosForY.replace("_", ".");
 
                    for (var c = 0; c < outpostOuterWallRightBottom.length; c++) {
                        if (starter == 1) {
                            offsetY = 2.25;
                        }
 
                        var indexX = outpostOuterWallRightBottom[c].indexOf("posX")
                        var indexY = outpostOuterWallRightBottom[c].indexOf("posY")
 
                        var diff = indexY - indexX;
 
                        var getSomePosForX = outpostOuterWallRightBottom[c].substring(indexX + 4, indexX + diff);
                        //var getSomePosForY = outpostOuterWall[c].substring(indexY + 4, outpostOuterWall[c].length - 1);
 
                        getSomePosForX = getSomePosForX.replace("_", ".");
                        //getSomePosForY = getSomePosForY.replace("_", ".");
 
                        var somePosX = newBasCoord.x + parseInt(getSomePosForX);
                        var somePosY = newBasCoord.y + (parseInt(getSomeOriPosForY) + (offsetY * c)); //parseInt(getSomePosForY);
 
                        var somePos = { x: somePosX, y: somePosY };
 
                        var rotatedCoordWall = RotatePoint(somePos, newBasCoord, 90);
 
                        var someOuterWall = generator.AddNPCShipToSystem(
                            "", "SC_NO_AI", 1, outpostOuterWallRightBottom[c],
                            sys_id,
                            rotatedCoordWall.x,
                            rotatedCoordWall.y,
                            { class: "turret", non_talkable: true });
 
                        ship.SetRotation(someOuterWall, 0);
                        ////console.PrintError(coords.x + "__" + coords.y);
                        starter = 1;
                    }
 
 
                    var starter = 0;
                    var offsetY = 0;
 
                    var currentYIndex = outpostOuterWallLeftBottom[0].indexOf("posY")
                    var getSomeOriPosForY = outpostOuterWallLeftBottom[0].substring(currentYIndex + 4, outpostOuterWallLeftBottom[0].length - 1);
                    getSomeOriPosForY = getSomeOriPosForY.replace("_", ".");
 
                    for (var c = 0; c < outpostOuterWallLeftBottom.length; c++) {
                        if (starter == 1) {
                            offsetY = -2.25;
                        }
 
                        var indexX = outpostOuterWallLeftBottom[c].indexOf("posX")
                        var indexY = outpostOuterWallLeftBottom[c].indexOf("posY")
 
                        var diff = indexY - indexX;
 
                        var getSomePosForX = outpostOuterWallLeftBottom[c].substring(indexX + 4, indexX + diff);
                        //var getSomePosForY = outpostOuterWall[c].substring(indexY + 4, outpostOuterWall[c].length - 1);
 
                        getSomePosForX = getSomePosForX.replace("_", ".");
                        //getSomePosForY = getSomePosForY.replace("_", ".");
 
                        var somePosX = newBasCoord.x + parseInt(getSomePosForX);
                        var somePosY = newBasCoord.y + (parseInt(getSomeOriPosForY) + (offsetY * c)); //parseInt(getSomePosForY);
 
                        var somePos = { x: somePosX, y: somePosY };
 
                        var rotatedCoordWall = RotatePoint(somePos, newBasCoord, 90);
 
                        var someOuterWall = generator.AddNPCShipToSystem(
                            "", "SC_NO_AI", 1, outpostOuterWallLeftBottom[c],
                            sys_id,
                            rotatedCoordWall.x,
                            rotatedCoordWall.y,
                            { class: "turret", non_talkable: true });
 
                        ship.SetRotation(someOuterWall, 0);
                        ////console.PrintError(coords.x + "__" + coords.y);
                        starter = 1;
                    }
 
 
                    var starter = 0;
                    var offsetY = 0;
 
                    var currentYIndex = outpostOuterWallLeftTop[0].indexOf("posY")
                    var getSomeOriPosForY = outpostOuterWallLeftTop[0].substring(currentYIndex + 4, outpostOuterWallLeftTop[0].length - 1);
                    getSomeOriPosForY = getSomeOriPosForY.replace("_", ".");
 
                    for (var c = 0; c < outpostOuterWallLeftTop.length; c++) {
                        if (starter == 1) {
                            offsetY = -2.25;
                        }
 
                        var indexX = outpostOuterWallLeftTop[c].indexOf("posX")
                        var indexY = outpostOuterWallLeftTop[c].indexOf("posY")
 
                        var diff = indexY - indexX;
 
                        var getSomePosForX = outpostOuterWallLeftTop[c].substring(indexX + 4, indexX + diff);
                        //var getSomePosForY = outpostOuterWall[c].substring(indexY + 4, outpostOuterWall[c].length - 1);
 
                        getSomePosForX = getSomePosForX.replace("_", ".");
                        //getSomePosForY = getSomePosForY.replace("_", ".");
 
                        var somePosX = newBasCoord.x + parseInt(getSomePosForX);
                        var somePosY = newBasCoord.y + (parseInt(getSomeOriPosForY) + (offsetY * c)); //parseInt(getSomePosForY);
 
                        var somePos = { x: somePosX, y: somePosY };
 
                        var rotatedCoordWall = RotatePoint(somePos, newBasCoord, 90);
 
                        var someOuterWall = generator.AddNPCShipToSystem(
                            "", "SC_NO_AI", 1, outpostOuterWallLeftTop[c],
                            sys_id,
                            rotatedCoordWall.x,
                            rotatedCoordWall.y,
                            { class: "turret", non_talkable: true });
 
                        ship.SetRotation(someOuterWall, 0);
                        ////console.PrintError(coords.x + "__" + coords.y);
                        starter = 1;
                    }
 
 
 
                    var starter = 0;
                    var offsetY = 0;
 
                    var currentYIndex = outpostOuterWallRightTop[0].indexOf("posY")
                    var getSomeOriPosForY = outpostOuterWallRightTop[0].substring(currentYIndex + 4, outpostOuterWallRightTop[0].length - 1);
                    getSomeOriPosForY = getSomeOriPosForY.replace("_", ".");
 
                    for (var c = 0; c < outpostOuterWallRightTop.length; c++) {
                        if (starter == 1) {
                            offsetY = 2.25;
                        }
 
                        var indexX = outpostOuterWallRightTop[c].indexOf("posX")
                        var indexY = outpostOuterWallRightTop[c].indexOf("posY")
 
                        var diff = indexY - indexX;
 
                        var getSomePosForX = outpostOuterWallRightTop[c].substring(indexX + 4, indexX + diff);
                        //var getSomePosForY = outpostOuterWall[c].substring(indexY + 4, outpostOuterWall[c].length - 1);
 
                        getSomePosForX = getSomePosForX.replace("_", ".");
                        //getSomePosForY = getSomePosForY.replace("_", ".");
 
                        var somePosX = newBasCoord.x + parseInt(getSomePosForX);
                        var somePosY = newBasCoord.y + (parseInt(getSomeOriPosForY) + (offsetY * c)); //parseInt(getSomePosForY);
 
                        var somePos = { x: somePosX, y: somePosY };
 
                        var rotatedCoordWall = RotatePoint(somePos, newBasCoord, 90);
 
                        var someOuterWall = generator.AddNPCShipToSystem(
                            "", "SC_NO_AI", 1, outpostOuterWallRightTop[c],
                            sys_id,
                            rotatedCoordWall.x,
                            rotatedCoordWall.y,
                            { class: "turret", non_talkable: true });
 
                        ship.SetRotation(someOuterWall, 0);
                        ////console.PrintError(coords.x + "__" + coords.y);
                        starter = 1;
                    }*/
                    //DESTRUCTIBLE STATIONS MOD
                    //DESTRUCTIBLE STATIONS MOD
                    //DESTRUCTIBLE STATIONS MOD


















                        /*if (storage.IsSetGlobal("stationEXT" + baseInteriorGlobalVariable.sys_id_link) && base_xml.length < 10)
                        {
                    
                        }*/





                        //TODO GLOBAL VARIABLE ADDON FOR DRONE ARRIVING TO A DIFFERENT STATION INSTEAD OF JUST THE STARTING STATION.
                        //TODO GLOBAL VARIABLE ADDON FOR DRONE ARRIVING TO A DIFFERENT STATION INSTEAD OF JUST THE STARTING STATION.
                        //TODO GLOBAL VARIABLE ADDON FOR DRONE ARRIVING TO A DIFFERENT STATION INSTEAD OF JUST THE STARTING STATION.
                        //storage.SetGlobal("system_" + sys_id + "_base_" + base_id + "_xmlStationType", theBase);
                        //TODO GLOBAL VARIABLE ADDON FOR DRONE ARRIVING TO A DIFFERENT STATION INSTEAD OF JUST THE STARTING STATION.
                        //TODO GLOBAL VARIABLE ADDON FOR DRONE ARRIVING TO A DIFFERENT STATION INSTEAD OF JUST THE STARTING STATION.
                        //TODO GLOBAL VARIABLE ADDON FOR DRONE ARRIVING TO A DIFFERENT STATION INSTEAD OF JUST THE STARTING STATION.
                    }



                    /*var baseInteriorGlobalVariable = storage.GetGlobal("stationINT" + sys_id);
                    //containing this from GenerateGalaxy.js => storage.GetGlobal("stationINT" + sys_id1, { id: id, x: coord_x, y: coord_y, sys_id: sys_id1, xml_id: arrayOfOutpostStations[randomIndexStationOutpost], DegAngle: parsedAngle, class: tagClass });
                    //for the station interior. a station
                    base_id = generator.AddBase(
                        baseInteriorGlobalVariable.sys_id,
                        baseInteriorGlobalVariable.x, baseInteriorGlobalVariable.y,
                        baseInteriorGlobalVariable.xml_id,
                        StationNameGenerator.GetName(),
                        { class: tagClass });

                    storage.SetGlobal("stationINT" + sys_id, baseInteriorGlobalVariable);*/







                }
                else if (tagClass == "science") // (base_xml.indexOf("science") >= 0)
                {
                    base_id = generator.AddBase(
                        sys_id,
                        bas_coord.x, bas_coord.y,
                        base_xml,
                        StationNameGenerator.GetName(),
                        { class: tagClass });
                    relations.SetBaseFaction(base_id, inf.faction);

                    bases.push({
                        id: base_id,
                        coord: bas_coord,
                        base_xml: base_xml
                    });


                    var getSomeIndex = base_xml.substring(11, base_xml.length); //science_01_0
                    var index = indexOfStuff(getSomeIndex);

                    if (index == 0) {
                        index = 4;
                    }
                    else if (index == 1) {
                        index = 5;
                    }
                    else if (index == 2) {
                        index = 6;
                    }
                    else if (index == 3) {
                        index = 7;
                    }
                    else if (index == 4) {
                        index = 0;
                    }
                    else if (index == 5) {
                        index = 1;
                    }
                    else if (index == 6) {
                        index = 2;
                    }
                    else if (index == 7) {
                        index = 3;
                    }

                    parsedAngle = parseInt(getSomeIndex);

                    var tempCoordRefuelX = bas_coord.x + 3.5;
                    var tempCoordRefuelY = bas_coord.y + 14.5;
                    var tempCoordRefuel = { x: tempCoordRefuelX, y: tempCoordRefuelY };

                    var tempCoordRepairX = bas_coord.x - 3;
                    var tempCoordRepairY = bas_coord.y + 14.5;
                    var tempCoordRepair = { x: tempCoordRepairX, y: tempCoordRepairY };

                    var rotatedCoordRefuel = RotatePoint(tempCoordRefuel, bas_coord, parsedAngle);
                    var rotatedCoordRepair = RotatePoint(tempCoordRepair, bas_coord, parsedAngle);

                    var StationRefuelPlatform = generator.AddSpecialObject(sys_id, rotatedCoordRefuel.x, rotatedCoordRefuel.y, arrayOfRefuelPlatforms[index], 0);
                    var StationRepairPlatform = generator.AddSpecialObject(sys_id, rotatedCoordRepair.x, rotatedCoordRepair.y, arrayOfRepairPlatforms[index], 0);


                    var turret_radius = 35;
                    var level = inf.danger_level + Math.round(MathExt.RandDouble() * 4 - 2);
                    level = utils.Clamp(level, 1, 100);
                    //var angle_offset = 0.0;

                    NpcGenerator.SpawnTurretsOfType(args, bas_coord, 4, level, turret_radius, "Turret", "special_human_turret", inf.faction, { class: "turret", non_talkable: true }, parsedAngle, base_xml, sys_id, base_id);
                    var theBase = { id: base_id, coord: bas_coord, base_xml: base_xml, sys_id: sys_id, rot: parsedAngle };// widthL: 21, widthR: 21, heightT: 19, heightB: 42, 
                    storage.SetGlobal("system_" + sys_id + "_base_" + base_id + "_xmlStationType", theBase);


                    var stationData = SC_Station_Tiles_Science_01.buildTiles(theBase, theBase.id);
                    storage.SetGlobal("station_tiles" + theBase.id, stationData);


                    /*SC_Station_Tiles_Science_01.buildTiles(theBase);
 
                    if (switchForDebugTiles == -1) {
                        if (theBase == null) {
                            //console.PrintError("theBase null: " + scriptCounter);
                        }
                        else {
                            if (theBase.base_xml == null) {
                                //console.PrintError("theBase.base_xml null: " + scriptCounter);
                            }
                            else
                            {
                                //console.PrintError("theBase.base_xml !null: " + theBase.base_xml + " id: " + theBase.id);
                                //SC_Station_Tiles_Science_01.buildTiles(theBase);
                                //console.PrintError("player_starting_system_base_: " + scriptCounter);
                                storage.SetGlobal("player_starting_system_base_", theBase);
                            }
                        }
                    }*/


                }
                else if (tagClass == "military") //(base_xml.indexOf("military") >= 0)
                {
                    base_id = generator.AddBase(
                        sys_id,
                        bas_coord.x, bas_coord.y,
                        base_xml,
                        StationNameGenerator.GetName(),
                        { class: tagClass });
                    relations.SetBaseFaction(base_id, inf.faction);

                    bases.push({
                        id: base_id,
                        coord: bas_coord,
                        base_xml: base_xml
                    });


                    var getSomeIndex = base_xml.substring(12, base_xml.length); //military_01_0
                    var index = indexOfStuff(getSomeIndex);

                    if (index == 0) {
                        index = 4;
                    }
                    else if (index == 1) {
                        index = 5;
                    }
                    else if (index == 2) {
                        index = 6;
                    }
                    else if (index == 3) {
                        index = 7;
                    }
                    else if (index == 4) {
                        index = 0;
                    }
                    else if (index == 5) {
                        index = 1;
                    }
                    else if (index == 6) {
                        index = 2;
                    }
                    else if (index == 7) {
                        index = 3;
                    }

                    parsedAngle = parseInt(getSomeIndex);

                    var tempCoordRefuelX = bas_coord.x + 3.5;
                    var tempCoordRefuelY = bas_coord.y + 14.5;
                    var tempCoordRefuel = { x: tempCoordRefuelX, y: tempCoordRefuelY };

                    var tempCoordRepairX = bas_coord.x - 3;
                    var tempCoordRepairY = bas_coord.y + 14.5;
                    var tempCoordRepair = { x: tempCoordRepairX, y: tempCoordRepairY };

                    var rotatedCoordRefuel = RotatePoint(tempCoordRefuel, bas_coord, parsedAngle);
                    var rotatedCoordRepair = RotatePoint(tempCoordRepair, bas_coord, parsedAngle);

                    var StationRefuelPlatform = generator.AddSpecialObject(sys_id, rotatedCoordRefuel.x, rotatedCoordRefuel.y, arrayOfRefuelPlatforms[index], 0);
                    var StationRepairPlatform = generator.AddSpecialObject(sys_id, rotatedCoordRepair.x, rotatedCoordRepair.y, arrayOfRepairPlatforms[index], 0);



                    var turret_radius = 35;
                    var level = inf.danger_level + Math.round(MathExt.RandDouble() * 4 - 2);
                    level = utils.Clamp(level, 1, 100);
                    //var angle_offset = 0.0;

                    NpcGenerator.SpawnTurretsOfType(args, bas_coord, 4, level, turret_radius, "Turret", "special_human_turret", inf.faction, { class: "turret", non_talkable: true }, parsedAngle, base_xml, sys_id, base_id);
                    var theBase = { id: base_id, coord: bas_coord, base_xml: base_xml, sys_id: sys_id, rot: parsedAngle };// widthL: 21, widthR: 21, heightT: 19, heightB: 42, 
                    storage.SetGlobal("system_" + sys_id + "_base_" + base_id + "_xmlStationType", theBase);




                    var stationData = SC_Station_Tiles_Military_01.buildTiles(theBase, theBase.id);
                    storage.SetGlobal("station_tiles" + theBase.id, stationData);


                    /*if (switchForDebugTiles == -1) {
                        if (theBase == null) {
                            //console.PrintError("theBase null: " + scriptCounter);
                        }
                        else {
                            if (theBase.base_xml == null) {
                                //console.PrintError("theBase.base_xml null: " + scriptCounter);
                            }
                            else {
                                //console.PrintError("theBase.base_xml !null: " + theBase.base_xml + " id: " + theBase.id);
                                SC_Station_Tiles_Military_01.buildTiles(theBase);
                                //console.PrintError("player_starting_system_base_: " + scriptCounter);
                                storage.SetGlobal("player_starting_system_base_", theBase);
                            }
                        }
                    }*/


                    /*SC_Station_Tiles_Military_01.buildTiles(theBase);
 
                    if (switchForDebugTiles == -1) {
                        if (theBase == null) {
                            //console.PrintError("theBase null: " + scriptCounter);
                        }
                        else {
                            if (theBase.base_xml == null) {
                                //console.PrintError("theBase.base_xml null: " + scriptCounter);
                            }
                            else {
                                //console.PrintError("theBase.base_xml !null: " + theBase.base_xml + " id: " + theBase.id);
                                //SC_Station_Tiles_Military_01.buildTiles(theBase);
                                //console.PrintError("player_starting_system_base_: " + scriptCounter);
                                storage.SetGlobal("player_starting_system_base_", theBase);
                            }
                        }
                    }*/
                }
                else if (tagClass == "business") //(base_xml.indexOf("business") >= 0)
                {
                    base_id = generator.AddBase(
                        sys_id,
                        bas_coord.x, bas_coord.y,
                        base_xml,
                        StationNameGenerator.GetName(),
                        { class: tagClass });
                    relations.SetBaseFaction(base_id, inf.faction);

                    bases.push({
                        id: base_id,
                        coord: bas_coord,
                        base_xml: base_xml
                    });


                    var getSomeIndex = base_xml.substring(12, base_xml.length); //business_01_0
                    var index = indexOfStuff(getSomeIndex);

                    /*if (index == 0)
                    {
                        index = 0;
                    }
                    else if (index == 1)
                    {
                        index = 5;
                    }
                    else if (index == 2)
                    {
                        index = 6;
                    }
                    else if (index == 3)
                    {
                        index = 7;
                    }
                    else if (index == 4)
                    {
                        index = 4;
                    }
                    else if (index == 5)
                    {
                        index = 1;
                    }
                    else if (index == 6)
                    {
                        index = 2;
                    }
                    else if (index == 7)
                    {
                        index = 3;
                    }*/

                    parsedAngle = parseInt(getSomeIndex);

                    var tempCoordRefuelX = bas_coord.x - 3.5;
                    var tempCoordRefuelY = bas_coord.y - 22;
                    var tempCoordRefuel = { x: tempCoordRefuelX, y: tempCoordRefuelY };

                    var tempCoordRepairX = bas_coord.x + 3;
                    var tempCoordRepairY = bas_coord.y - 22;
                    var tempCoordRepair = { x: tempCoordRepairX, y: tempCoordRepairY };

                    var rotatedCoordRefuel = RotatePoint(tempCoordRefuel, bas_coord, parsedAngle);
                    var rotatedCoordRepair = RotatePoint(tempCoordRepair, bas_coord, parsedAngle);

                    var StationRefuelPlatform = generator.AddSpecialObject(sys_id, rotatedCoordRefuel.x, rotatedCoordRefuel.y, arrayOfRefuelPlatforms[index], 0);
                    var StationRepairPlatform = generator.AddSpecialObject(sys_id, rotatedCoordRepair.x, rotatedCoordRepair.y, arrayOfRepairPlatforms[index], 0);

                    var turret_radius = 35;
                    var level = inf.danger_level + Math.round(MathExt.RandDouble() * 4 - 2);
                    level = utils.Clamp(level, 1, 100);
                    //var angle_offset = 0.0;

                    NpcGenerator.SpawnTurretsOfType(args, bas_coord, 4, level, turret_radius, "Turret", "special_human_turret", inf.faction, { class: "turret", non_talkable: true }, parsedAngle, base_xml, sys_id, base_id);
                    var theBase = { id: base_id, coord: bas_coord, base_xml: base_xml, sys_id: sys_id, rot: parsedAngle };// widthL: 21, widthR: 21, heightT: 19, heightB: 42, 
                    storage.SetGlobal("system_" + sys_id + "_base_" + base_id + "_xmlStationType", theBase);



                    var stationData = SC_Station_Tiles_Business_01.buildTiles(theBase, theBase.id);
                    storage.SetGlobal("station_tiles" + theBase.id, stationData);


                    //if(switchForDebugTiles == 1)
                    //{
                    //	    SC_Station_Tiles_Business_01.buildTiles(theBase);
                    //}

                    /*SC_Station_Tiles_Business_01.buildTiles(theBase);
 
                    if (switchForDebugTiles == -1) {
                        if (theBase == null) {
                            //console.PrintError("theBase null: " + scriptCounter);
                        }
                        else {
                            if (theBase.base_xml == null) {
                                //console.PrintError("theBase.base_xml null: " + scriptCounter);
                            }
                            else {
                                //console.PrintError("theBase.base_xml !null: " + theBase.base_xml + " id: " + theBase.id);
                                //SC_Station_Tiles_Business_01.buildTiles(theBase);
                                //console.PrintError("player_starting_system_base_: " + scriptCounter);
                                storage.SetGlobal("player_starting_system_base_", theBase);
                            }
                        }
                    }*/
                }

                else if (tagClass == "mining") //(base_xml.indexOf("mining") >= 0)
                {
                    base_id = generator.AddBase(
                        sys_id,
                        bas_coord.x, bas_coord.y,
                        base_xml,
                        StationNameGenerator.GetName(),
                        { class: tagClass });
                    relations.SetBaseFaction(base_id, inf.faction);

                    bases.push({
                        id: base_id,
                        coord: bas_coord,
                        base_xml: base_xml
                    });


                    var getSomeIndex = base_xml.substring(10, base_xml.length); //mining_01_0
                    var index = indexOfStuff(getSomeIndex);
                    parsedAngle = parseInt(getSomeIndex);

                    if (index == 0) {
                        index = 4;
                    }
                    else if (index == 1) {
                        index = 5;
                    }
                    else if (index == 2) {
                        index = 6;
                    }
                    else if (index == 3) {
                        index = 7;
                    }
                    else if (index == 4) {
                        index = 0;
                    }
                    else if (index == 5) {
                        index = 1;
                    }
                    else if (index == 6) {
                        index = 2;
                    }
                    else if (index == 7) {
                        index = 3;
                    }

                    var tempCoordRepairX = bas_coord.x + 14.75;
                    var tempCoordRepairY = bas_coord.y + 17;
                    var tempCoordRepair = { x: tempCoordRepairX, y: tempCoordRepairY };

                    var rotatedCoordRepair = RotatePoint(tempCoordRepair, bas_coord, parsedAngle);
                    var StationRepairPlatform = generator.AddSpecialObject(sys_id, rotatedCoordRepair.x, rotatedCoordRepair.y, arrayOfRepairPlatforms[index], 0);


                    if (index == 0) {
                        index = 4;
                    }
                    else if (index == 1) {
                        index = 5;
                    }
                    else if (index == 2) {
                        index = 6;
                    }
                    else if (index == 3) {
                        index = 7;
                    }
                    else if (index == 4) {
                        index = 0;
                    }
                    else if (index == 5) {
                        index = 1;
                    }
                    else if (index == 6) {
                        index = 2;
                    }
                    else if (index == 7) {
                        index = 3;
                    }

                    var tempCoordRefuelX = bas_coord.x + 14.75;
                    var tempCoordRefuelY = bas_coord.y + 0;
                    var tempCoordRefuel = { x: tempCoordRefuelX, y: tempCoordRefuelY };

                    var rotatedCoordRefuel = RotatePoint(tempCoordRefuel, bas_coord, parsedAngle);
                    var StationRefuelPlatform = generator.AddSpecialObject(sys_id, rotatedCoordRefuel.x, rotatedCoordRefuel.y, arrayOfRefuelPlatforms[index], 0);

                    var turret_radius = 35;
                    var level = inf.danger_level + Math.round(MathExt.RandDouble() * 4 - 2);
                    level = utils.Clamp(level, 1, 100);
                    //var angle_offset = 0.0;

                    NpcGenerator.SpawnTurretsOfType(args, bas_coord, 4, level, turret_radius, "Turret", "special_human_turret", inf.faction, { class: "turret", non_talkable: true }, parsedAngle, base_xml, sys_id, base_id);
                    var theBase = { id: base_id, coord: bas_coord, base_xml: base_xml, sys_id: sys_id, rot: parsedAngle };// widthL: 21, widthR: 21, heightT: 19, heightB: 42, 
                    storage.SetGlobal("system_" + sys_id + "_base_" + base_id + "_xmlStationType", theBase);

                    /*SC_Station_Tiles_Mining_01.buildTiles(theBase);
 
                    if (switchForDebugTiles == -1)
                    {
                        if (theBase == null)
                        {
                            //console.PrintError("theBase null: " + scriptCounter);
                        }
                        else
                        {
                            if (theBase.base_xml == null)
                            {
                                //console.PrintError("theBase.base_xml null: " + scriptCounter);
                            }
                            else {
                                //console.PrintError("theBase.base_xml !null: " + theBase.base_xml + " id: " + theBase.id);
                                //SC_Station_Tiles_Mining_01.buildTiles(theBase);
                                //console.PrintError("player_starting_system_base_: " + scriptCounter);
                                storage.SetGlobal("player_starting_system_base_", theBase);
                            }
                        }
                    }*/
                }


                /*if (base_xml.indexOf("mining") >= 0)
                {

                }
                else if (base_xml.indexOf("outpost") >= 0)
                {
                    // add turrets
                    var turret_radius = 60;
                    var level = inf.danger_level + Math.round(MathExt.RandDouble() * 4 - 2);
                    level = utils.Clamp(level, 1, 100);
                    //var angle_offset = 0.0;

                    NpcGenerator.SpawnTurretsOfType(args, bas_coord, 4, level, turret_radius, "Turret", "special_human_turret", inf.faction, { class: "turret", non_talkable: true }, parsedAngle,base_xml);
                }
                else if (base_xml.indexOf("science") >= 0)
                {

                }
                else if (base_xml.indexOf("business") >= 0)
                {

                }
                else if (base_xml.indexOf("military") >= 0)
                {

                }*/

                // add satellite
                /*if(base_xml == "outpost_01")
                 {
                 this.AddSatellite(args, bas_coord, "satellite_01");
                 }*/

                /*if (base_xml == "outpost_01")
                {
                    // add turrets
                    var turret_radius = 20;
                    var level = inf.danger_level + Math.round(MathExt.RandDouble() * 4 - 2);
                    level = utils.Clamp(level, 1, 100);
                    var angle_offset = 0.0;
                    NpcGenerator.SpawnTurretsOfType(args, bas_coord, 4, level, turret_radius, "Turret", "special_human_turret", inf.faction, { class: "turret", non_talkable: true }, angle_offset, sys_id, base_id);
                }*/
            }
        }

        this.AddDecorations(
            args,
            [
                {
                    type: "satellite_01_rust",
                    count_from: 1,
                    count_to: 2,
                    rotation: this.GenSatelliteRotation(),
                    rotationSpeed: this.GenSatelliteRotationSpeed(),
                    distance_from: 45,
                    distance_to: 80,
                    scale_from: 0.7,
                    scale_to: 0.7,
                },
                {
                    type: "broken_asteroid_01",
                    count_from: 3,
                    count_to: 10,
                    rotation: "random",
                    rotationSpeed: { x: 0, y: 0, z: 0 },
                    distance_from: 45,
                    distance_to: 80,
                    scale_from: 2,
                    scale_to: 3,
                },
                {
                    type: "station_wreck_01",
                    count_from: 1,
                    count_to: 3,
                    rotation: "random",
                    rotationSpeed: { x: 0, y: 0, z: 0 },
                    distance_from: 45,
                    distance_to: 80,
                    scale_from: 2,
                    scale_to: 3,
                },
            ]);


        return bases;
    },
    //---------------------------------------------------
    // Adds alien hives of specified types (ship-like destructable alien bases)
    //
    //---------------------------------------------------
    AddAlienHives: function (args, types, quantity) {
        var system_id = args.sys_id;
        var inf = args.system_info;
        var bases = [];

        // add base
        var tries = 0;
        for (var i = 0; i < quantity; i++) {
            var ang = MathExt.RandDouble() * Math.PI * 2.0;
            var dist = MathExt.RandRangeDouble(150, 250);
            var coords = { x: dist * Math.cos(ang), y: dist * Math.sin(ang) };

            var is_fits = true;

            if (generator.AreThereAnyObjects(system_id, coords.x, coords.y, 25)) {
                is_fits = false;
            }

            if (is_fits) {
                for (var k = 0; k < bases.length; k++) {
                    var dist = MathExt.Vector2Length(bases[k].coord, coords);
                    if (dist < 70) {
                        is_fits = false;
                    } //should be no closer then 70 to any other base
                }
            }

            if (!is_fits) {
                i--;
                tries++;
                if (tries > 1500) {
                    //console.PrintError("Add alien hives: cannot add " + quantity + " of bases.");
                }
            }
            else {
                var level = inf.danger_level + Math.round(MathExt.RandDouble() * 4 - 2);
                level = utils.Clamp(level, 1, 100);
                var base_xml = utils.SelectRandom(types);
                var id = generator.AddNPCShipToSystem($i0001, "Turret", level, base_xml, system_id, coords.x, coords.y, { class: "alien_hive", sex: "male" }); // Hive
                relations.SetShipFaction(id, "aliens");

                bases.push({
                    id: id,
                    coord: coords,
                });

                // add turrets	
                var turret_radius = 20;
                var angle_offset = 0.0;
                NpcGenerator.SpawnTurretsOfType(args, coords, 4, level, turret_radius, "Turret", "special_xengatarn_turret", "aliens", { class: "turret", non_talkable: true }, angle_offset, base_xml, system_id, id);
            }
        }

        this.AddDecorations(args,
            [
                {
                    type: "satellite_01_rust",
                    count_from: 1,
                    count_to: 2,
                    rotation: this.GenSatelliteRotation(),
                    rotationSpeed: this.GenSatelliteRotationSpeed(),
                    distance_from: 45,
                    distance_to: 80,
                    scale_from: 0.7,
                    scale_to: 0.7,
                },
                {
                    type: "broken_asteroid_01",
                    count_from: 3,
                    count_to: 10,
                    rotation: "random",
                    rotationSpeed: { x: 0, y: 0, z: 0 },
                    distance_from: 45,
                    distance_to: 80,
                    scale_from: 2,
                    scale_to: 3,
                },
                {
                    type: "station_wreck_01",
                    count_from: 1,
                    count_to: 3,
                    rotation: "random",
                    rotationSpeed: { x: 0, y: 0, z: 0 },
                    distance_from: 45,
                    distance_to: 80,
                    scale_from: 2,
                    scale_to: 3,
                },
            ]);


        return bases;
    },
    /*
     ================================
     AddDecorations
     
     used to add abstract decorations
     
     type format:
     {
     type: string,
     count_from: number,
     count_to: number,
     rotation: vector3,
     rotationSpeed: vector3,
     distance_from: number,
     distance_to: number,
     scale_from: number
     scale_to: number
     }
     ================================
     */
    AddDecorations: function (args, types) {
        var points = [];

        for (var k = 0; k < types.length; k++) {
            var current = types[k];
            var count = MathExt.RandRange(current.count_from, current.count_to + 1);

            var pass;
            for (var i = 0; i < count;) {
                var pass = false;
                var coord = CSGen.GetRandomCoordWithinWorldBounds(args);
                for (var j = 0; j < points.length; j++) {
                    var dist = MathExt.Vector2Length(points, coord);
                    if (dist < 50) {
                        pass = true;
                        break;
                    }
                }

                if (!pass) {
                    //generate
                    i++;

                    var distance = MathExt.RandRangeDouble(current.distance_from, current.distance_to);
                    var scale = MathExt.RandRangeDouble(current.scale_from, current.scale_to);
                    this.AddDecoration(
                        args,
                        coord,
                        current.type,
                        current.rotation,
                        current.rotationSpeed,
                        distance,
                        scale);
                }
            }
        }
    },
    AddDecoration: function (args, coords, type, rotation, rotationSpeed, distance, scale) {
        if (rotation == "random") {
            rotation = this.GenRandomRotation();
        }

        generator.AddDecoration(
            args.sys_id,
            type,
            coords,
            distance,
            rotation,
            rotationSpeed,
            scale);
    },
    GenSatelliteRotation: function () {
        return { X: (MathExt.RandDouble()) * 90, Y: (MathExt.RandDouble()) * 90, Z: 0.0 };
    },
    GenSatelliteRotationSpeed: function () {
        return { X: (MathExt.RandDouble() * 0.5 - 1.0) * 0.025 * 90, Y: 0.0, Z: 0.0 };
    },
    GenRandomRotation: function () {
        return {
            X: MathExt.RandDouble() * 360,
            Y: MathExt.RandDouble() * 360,
            Z: MathExt.RandDouble() * 360
        };
    },
    //-------------------------------
    // LEGACY METHODS

    /*
     AddSatellite : function(args, coords, type)
     {
     var rotation = { X: (MathExt.RandDouble()) * 90, Y: (MathExt.RandDouble()) * 90, Z: 0.0 };
     var rotationSpeed = { X: (MathExt.RandDouble() * 0.5 - 1.0) * 0.025 * 90, Y: 0.0, Z: 0.0 };
     var distance = MathExt.RandRangeDouble(45, 80);
     
     generator.AddDecoration(args.sys_id,
     type,
     coords,
     distance,
     rotation,
     rotationSpeed,
     1);
     },
     
     AddBrokenAsteroid : function(args, coords, type)
     {
     var rotation = { X: 0, Y: 0, Z: 0.0 };
     var rotationSpeed = { X: 0, Y: 0.0, Z: 0.0 };
     var distance = MathExt.RandRangeDouble(55, 80);
     
     generator.AddDecoration(args.sys_id,
     type,
     coords,
     distance,
     rotation,
     rotationSpeed,
     1);
     },*/

};


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



