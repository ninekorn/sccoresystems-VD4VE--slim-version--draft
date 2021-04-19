//--------------------------------------------------------
// Desc:
//	This is a preset XML-like JS library, which is designed to give
//	an easy way to create new systems and modify existing ones
//--------------------------------------------------------
include(CSGen.js);

using(relations);
using(generator);
using(console);
using(storage);
using(spawn);


include(NamesGen.js);
include(CSGen.js);
include(NpcGenerator.js);
include(BasesGenerator.js);

/*
 structure of preset:
 
 Name,
 CheckIfFits - function, analyzes the system, and returns true, if it fits
 Chance - relative number, chance this system to be generated amound other candidates
 QuantityCheck,
 CreateSystemBases - system creating routine
 CreateSystemObjects
 NpcGenerationStep
 */

var arrayOfMiningStations =
    [
        //"outpost_01_0",
        "mining_01_0",
        /*"mining_01_45",
        "mining_01_90",
        "mining_01_135",
        "mining_01_180",
        "mining_01_225",
        "mining_01_270",
        "mining_01_315"*/
    ];

var arrayOfMilitaryStations =
    [
        //"outpost_01_0",
        "military_01_0",
        /*"military_01_45",
        "military_01_90",
        "military_01_135",
        "military_01_180",
        "military_01_225",
        "military_01_270",
        "military_01_315"*/
    ];

var arrayOfScienceStations =
    [
        "science_01_0",
        /*"science_01_45",
        "science_01_90",
        "science_01_135",
        "science_01_180",
        "science_01_225",
        "science_01_270",
        "science_01_315"*/
    ];

var arrayOfOutpostStations =
    [
        "outpost_01_0",
        /*"outpost_01_45",
        "outpost_01_90",
        "outpost_01_135",
        "outpost_01_180",
        "outpost_01_225",
        "outpost_01_270",
        "outpost_01_315"*/
    ];


var arrayOfBusinessStations =
    [
        "business_01_0",
        /*"business_01_45",
        "business_01_90",
        "business_01_135",
        "business_01_180",
        "business_01_225",
        "business_01_270",
        "business_01_315"*/
    ];

var SystemsPresets =
{
    //-------------------------------------------------------------
    // S T A R T I N G    S Y S T E M
    //-------------------------------------------------------------
    StartingSystem: {
        Name: "StartingSystem",
        CheckIfFits: function (args) {
            var res = args.sys_id == spawn.GetSystemID();
            return res;
        },
        Chance: 100,

        QuantityCheck: function (args) {
            return this.Quantity == 0;
        },
        CreateSystemBases: function (args) {
            var indexOutpost = Math.floor(Math.random() * arrayOfOutpostStations.length);

            //console.PrintError(arrayOfOutpostStations[indexOutpost]);
            //storage.SetGlobal("system_" + args.sys_id + "_base_" + base_id + "_xml" + base_xml, base_xml);
            return BasesGenerator.AddBasesOfTypes(args, [arrayOfOutpostStations[indexOutpost]], 1, null, null, -1); //[arrayOfOutpostStations[indexOutpost] //[arrayOfOutpostStations[indexOutpost]]
        },
        CreateSystemObjects: function (args) {
            // set techlevel
            generator.SetSystemTechLevel(args.sys_id, 1);

            // Set playlist
            generator.SetSystemPlaylists(args.sys_id,
                {
                    "explore": "playlist_explore_starting",
                    "combat": "playlist_combat_starting"
                }
            );


            // objects
            CSGen.CreateGenericStars(args);
            CSGen.CreateGenericPlanets(args, { allow_belts: false, min_planets: 1 });
            CSGen.CreateGenericObjects(args, 4);
            CSGen.CreateAsteroidsByPrefixes(args, { rock: 1 }, 4, 1, 3);

            CSGen.CreatePlanetsWithType(args, "habitable_01");


            var ergrek_terminal_id = CSGen.CreateMiniStation(args, "ergrek_terminal");
            storage.SetGlobal("starting_system_ergrek_terminal_id", ergrek_terminal_id);

            NpcGenerator.SpawnMinersSquad(args);
            for (var i = 0; i < args.bases.length; i++) {
                NpcGenerator.SpawnStationPopulation(args, args.bases[i].id, this.Name);
            }

            // do not spawn treasures in the start system!
            // NpcGenerator.SpawnNpcsPointsWithTreasure(args, 1);

            NpcGenerator.SpawnPirateBlockposts(args, MathExt.RandRange(1, 3));
            //removed TEMP 9sMODs
            //removed TEMP 9sMODs


            //removed 9sMODs
            // additional
            /*if (args.bases.length == 0) {
                console.PrintError("In the starting system there's no bases. That's not right, fix the generation.js script");
                console.WaitForUserInteraction();
            }*/
            //removed 9sMODs



            var systems = generator.GetAllSystems();
            for (var j = 0; j < systems.length; j++)
            {
                var inf = generator.GetSystemByID(systems[j]);



                if (inf.name == "player base") {




                    //spawn.SetCoordinates(bas_coord.x, bas_coord.y);
                    //args.start_system_id = start_system_id;
                    //spawn.SetSystemID(baseInteriorGlobalVariable.sys_id);

                    //SETTING STARTING SYSTEM GLOBAL VARIABLE A SECOND TIME, BUT WITH DIFFERENT DATA. setglobal first called in GenerateGalaxy.js
                    /*var startsysdata = storage.GetGlobal("startingsystem");
                    if (startsysdata.id == systems[j]) {


                        spawn.SetCoordinates(startsysdata.coords.x, startsysdata.coords.y);

                        /*if (startsysdata.id == sys_id) {
                            var someStartingBase = { sys_id: startsysdata.id, baseID: base_id, sys_name: startsysdata.sys_name, coords: bas_coord };
                            storage.SetGlobal("startingsystem", someStartingBase);
                        }
                        else if (startsysdata.id == baseInteriorGlobalVariable.sys_id_link) {
                            var someStartingBase = { sys_id: baseInteriorGlobalVariable.sys_id_link, baseID: base_id, sys_name: inf.name, coords: bas_coord };
                            storage.SetGlobal("startingsystem", someStartingBase);
                        }
                    }*/
                        //SETTING STARTING SYSTEM GLOBAL VARIABLE A SECOND TIME, BUT WITH DIFFERENT DATA. setglobal first called in GenerateGalaxy.js






                    //too tired to code. coming back to this tomorrow.
                    if (!storage.IsSetGlobal("stationEXT" + systems[j])) {

                        /*console.PrintError("testing 0");
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
                        /*var angle = -MathExt.Vector2Angle(system_vec, { x: curinf.coord_x, y: curinf.coord_y });
                        var radius = bounds.x * 0.75 * MathExt.RandRangeDouble(0.6, 1.2);
                        var coord_x = radius * Math.cos(angle);
                        var coord_y = radius * Math.sin(angle);
                        var id = generator.AddNPCShipToSystem("station test", "ai_station_interior", 100, "xml_station_interior", sys_id1, coord_x - 5, coord_y + 10, { someTag: "stationINT", class: "stationINT", greeting: "terminal", unique_id: "stationINT" }); //    
                        var radToDegAngle = angle;//SC_Utilities.RadianToDegree(angle);
                        var bounds = CSGen.GetWorldBounds();
                        var system_vec = { x: inf.coord_x, y: inf.coord_y };
                        var angle = -MathExt.Vector2Angle(system_vec, { x: inf.coord_x, y: inf.coord_y });
                        var radius = bounds.x * 0.75 * MathExt.RandRangeDouble(0.6, 1.2);
                        var coord_x = radius * Math.cos(angle);
                        var coord_y = radius * Math.sin(angle);

                        storage.SetGlobal("stationINT" + sys_id1, { id: null, x: coord_x, y: coord_y, sys_id: sys_id1, xml_id: arrayOfOutpostStations[randomIndexStationOutpost], DegAngle: parsedAngle, class: tagClass });
                        storage.SetGlobal("stationEXT" + sys_id1, { id: null, x: coord_x, y: coord_y, sys_id: sys_id0, xml_id: arrayOfOutpostStations[randomIndexStationOutpost], DegAngle: parsedAngle, class: tagClass });
                        */
      
                        /*var bas_coord = { x: system_vec.x, y: system_vec.y };// args.bases[0].coord;
                        var ang = MathExt.RandDouble() * Math.PI * 2;
                        spawn.SetCoordinates(
                            bas_coord.x + 50 * Math.cos(ang),
                            bas_coord.y + 50 * Math.sin(ang));


                        var baseExteriorGlobalVariable = storage.GetGlobal("stationEXT" + systems[j]);
                        var bas_coord = { x: baseExteriorGlobalVariable.x, y: baseExteriorGlobalVariable.y };// args.bases[0].coord;
                        var ang = MathExt.RandDouble() * Math.PI * 2;
                        spawn.SetCoordinates(
                            bas_coord.x + 50 * Math.cos(ang),
                            bas_coord.y + 50 * Math.sin(ang));*/
                 
                    }
                    else {
                        //console.PrintError("SystemsPreset.js loads before the GenerateGalaxy.js script.");
                        
                    }

















                    






                }
                else {
                    if (args.bases[0] != null) {
                        var bas_coord = args.bases[0].coord;
                        var ang = MathExt.RandDouble() * Math.PI * 2;
                        spawn.SetCoordinates(
                            bas_coord.x + 50 * Math.cos(ang),
                            bas_coord.y + 50 * Math.sin(ang));

                    }                
                }
            }


            /*var bas_coord = args.bases[0].coord;
            var ang = MathExt.RandDouble() * Math.PI * 2;
            spawn.SetCoordinates(
                bas_coord.x + 50 * Math.cos(ang),
                bas_coord.y + 50 * Math.sin(ang));*/



        },
        NpcGenerationStep: function (args) {
            return NpcGenerator.SpawnNpcsStep(args, 40, ["pirate"], "pirates", $i0002, [
                { name: "PirateInWaiting", chance: 5 },
                { name: "RoutePatrolling", chance: 2 },
            ], { class: "pirate" }); // Pirate
        },
    },
    //-------------------------------------------------------------
    // A L I E N S
    //-------------------------------------------------------------
    AliensSystem: {
        Name: "AliensSystem",
        CheckIfFits: function (args) {
            var res = args.system_info.faction == "aliens";
            return res;
        },
        Chance: 100,

        QuantityCheck: function (args) {
            return true;
        },
        CreateSystemBases: function (args) {
            var isExodus = false;
            if (!storage.IsSetGlobal("exodus")) {
                var capital = relations.GetFactionCapital("aliens");
                var linked = generator.GetLinkedSystems(args.sys_id);
                if (linked.indexOf(capital) >= 0) {
                    // this is neighboring system to alien capital - make it the Exodus system
                    storage.SetGlobal("exodus",
                        {
                            sys_id: args.sys_id
                        });
                    isExodus = true;
                }
            }

            var isCapital = args.sys_id == relations.GetFactionCapital("aliens");
            var level = args.system_info.danger_level;

            if (isExodus || isCapital) {
                // if its exodus or capital - then  add more hives
                // more hives = more fun =)
                return BasesGenerator.AddAlienHives(args, ["special_xengatarn_hive"], MathExt.RandRange(5, 8));
            }
            else if (level < 90) {
                //periphery alien system
                return BasesGenerator.AddAlienHives(args, ["special_xengatarn_hive"], MathExt.RandRange(2, 3));
            }
            else {
                //core alien system
                return BasesGenerator.AddAlienHives(args, ["special_xengatarn_hive"], MathExt.RandRange(3, 4));
            }
        },
        CreateSystemObjects: function (args) {
            // Set playlist
            var system_id = args.sys_id;
            generator.SetSystemPlaylists(system_id,
                {
                    "explore": "playlist_explore_alien",
                    "combat": "playlist_combat_alien"
                }
            );

            var exodus = storage.GetGlobal("exodus");
            var isExodus = exodus != null && exodus.sys_id == system_id;
            if (isExodus) {
                //spawn secret ancient jumpgate outside system borders
                var rad = 400;
                var ang = MathExt.RandRangeDouble(0, Math.PI * 2);
                var coords = {
                    x: rad * Math.cos(ang),
                    y: rad * Math.sin(ang)
                };
                var jg_id = generator.AddJumpgate(
                    system_id, system_id,
                    coords.x,
                    coords.y,
                    ang,
                    "jumpgate_01");
                exodus.jumpgate_id = jg_id;
                exodus.jumpgate_coords = coords;
                storage.SetGlobal("exodus", exodus);
            }

            var isCapital = system_id == relations.GetFactionCapital("aliens");
            if (isCapital) {
                // Set playlist
                generator.SetSystemPlaylists(system_id,
                    {
                        "explore": "playlist_special_alien_capital",
                        "combat": "playlist_special_alien_capital"
                    }
                );

                //add Xengatarn control device

                //pick coordinates
                // in 2k tries
                var isCreated = false;
                for (var i = 0; i < 2000; i++) {
                    var coords = CSGen.GetRandomCoordWithinWorldBounds(args);
                    if (CSGen.CoordInSafeZones(args, coords)
                        || generator.AreThereAnyObjects(system_id, coords.x, coords.y, 25)) {
                        continue;
                    }

                    var id = generator.AddNPCShipToSystem($i0001, "Base", 100, "special_xengatarn_cwaveemitter", system_id, coords.x, coords.y, { class: "cwaveemitter", sex: "male" }); // Unknown Construction
                    relations.SetShipFaction(id, "aliens");

                    storage.SetGlobal("mind_control_device", { id: id });
                    isCreated = true;

                    break;
                }

                if (!isCreated) {
                    console.PrintError("Critical error: couldn't create C-Field generator.");
                }
            }

            CSGen.CreateGenericStars(args);
            //CSGen.CreateGenericPlanets(args);
            CSGen.CreatePlanetsWithType(args, "alien_01");
            CSGen.CreateGenericObjects(args, 4);

            if (isCapital || isExodus) {
                CSGen.CreateMiniStation(args, "platform_refuel");
                // two repair platforms
                CSGen.CreateMiniStation(args, "platform_repair");
                CSGen.CreateMiniStation(args, "platform_repair");
            }
            else {
                //CSGen.CreateMiniStationsRandom(args, "ergrek_terminal", 1, 3);
                //CSGen.CreateMiniStationsRandom(args, "platform_refuel", 0, 1);
                //CSGen.CreateMiniStationsRandom(args, "platform_repair", 0, 1);
            }

            // objects
            var level = args.system_info.danger_level;

            if (level < 90) {
                //periphery alien system
                CSGen.CreateAsteroidsByPrefixes(args, { rock: 5, ice: 1 }, 4, 1, 3);

                NpcGenerator.SpawnNpcsPointsWithTreasure(args, 5);
            }
            else {
                //core alien system
                CSGen.CreateAsteroidsByPrefixes(args, { rock: 5, ice: 1 }, 4, 1, 3);
            }
        },
        NpcGenerationStep: function (args) {
            var level = args.system_info.danger_level;
            var npcsCount = level < 90 ? 70 : 90;

            return NpcGenerator.SpawnNpcsStep(args, npcsCount, ["alien"], "aliens", $i0003, [
                { name: "PirateInWaiting", chance: 5 },
                { name: "RoutePatrolling", chance: 2 },], { class: "alien" }); // Alien
        },
    },
    //-------------------------------------------------------------
    // G E N E R I C
    //-------------------------------------------------------------
    GenericSystem: {
        Name: "GenericSystem",
        CheckIfFits: function (args) {
            var faction = args.system_info.faction;

            //todo: replace to faction == "none"
            var res =
                args.sys_id != spawn.GetSystemID() &&
                faction == "none";
            return res;
        },
        Chance: 2,

        QuantityCheck: function (args) {
            return true;
        },
        CreateSystemBases: function (args) {
            //// special trick - we need at least one low-level neutral generic station,
            //// so we need to guarantee that	
            // FIXED by ai_enabled - previously if world too small, neutral stations may be unsufficient
            // and game will not generate required quest npcs for them.

            var station_level = generator.GetSystemByID(args.sys_id).tech_level;

            var neutral_stations_special = storage.Get("_system_generation", "neutral_stations_special");
            if (neutral_stations_special == null) {
                neutral_stations_special = [];
            }

            if (neutral_stations_special.length < 3) {
                // need more neutral stations

                do {
                    var indexOutpost = Math.floor(Math.random() * arrayOfOutpostStations.length);
                    var indexMining = Math.floor(Math.random() * arrayOfMiningStations.length);
                    var indexBusiness = Math.floor(Math.random() * arrayOfBusinessStations.length);


                    var res = BasesGenerator.AddBasesOfTypes(args, [arrayOfMiningStations[indexMining], arrayOfOutpostStations[indexOutpost], arrayOfBusinessStations[indexBusiness]], 1, null, null, 0); //"mining_01", "outpost_01", "business_01"
                }
                while (res.length == 0);

                neutral_stations_special.push(res[0].id);
                storage.Set("_system_generation", "neutral_stations_special", neutral_stations_special);

                return res;
            }
            else {
                var pick = MathExt.RandRange(0, 100);
                if (pick < 20) {
                    var indexOutpost = Math.floor(Math.random() * arrayOfOutpostStations.length);
                    var indexMining = Math.floor(Math.random() * arrayOfMiningStations.length);
                    var indexBusiness = Math.floor(Math.random() * arrayOfBusinessStations.length);

                    var res = BasesGenerator.AddBasesOfTypes(args, [arrayOfMiningStations[indexMining], arrayOfOutpostStations[indexOutpost], arrayOfBusinessStations[indexBusiness]], 1, null, null, 0); //["mining_01", "outpost_01", "business_01"]
                    return res;
                }
                else {
                    return [];
                }
            }
        },
        CreateSystemObjects: function (args) {
            // Set playlist
            var system_id = args.sys_id;
            generator.SetSystemPlaylists(system_id,
                {
                    "explore": "playlist_explore_generic",
                    "combat": "playlist_combat_generic"
                }
            );

            var level = args.system_info.danger_level;

            // OBJECTS
            CSGen.CreateGenericStars(args);
            CSGen.CreateGenericPlanets(args);
            CSGen.CreateGenericObjects(args, 4);
            CSGen.CreateAsteroidsByPrefixes(args, { rock: 1 }, 4, 1, 3);

            //CSGen.CreateMiniStationsRandom(args, "ergrek_terminal", 1, 3);
            //CSGen.CreateMiniStationsRandom(args, "platform_refuel", 0, 1);
            //CSGen.CreateMiniStationsRandom(args, "platform_repair", 0, 1);

            // NPCS						
            NpcGenerator.SpawnMinersSquad(args);
            for (var i = 0; i < args.bases.length; i++) {
                NpcGenerator.SpawnStationPopulation(args, args.bases[i].id, this.Name);
            }

            var num = MathExt.RandRange(3, 5);
            NpcGenerator.SpawnNpcsPointsWithTreasure(args, num);
            NpcGenerator.SpawnPirateBlockposts(args, MathExt.RandRange(1, 3));
        },
        NpcGenerationStep: function (args) {
            return NpcGenerator.SpawnNpcsStep(args, 70, ["pirate"], "pirates", $i0002, [
                { name: "PirateInWaiting", chance: 5 },
                { name: "RoutePatrolling", chance: 2 },
            ], { class: "pirate" }); // Pirate
        },
    },
    //-------------------------------------------------------------
    // O R D E R
    //-------------------------------------------------------------
    OrderSystem: {
        Name: "OrderSystem",
        CheckIfFits: function (args) {
            var faction = args.system_info.faction;

            if (faction == "order") {
                return true;
            }

            return false;
        },
        Chance: 100,

        QuantityCheck: function (args) {
            return true;
        },
        CreateSystemBases: function (args) {
            var capital_system_id = relations.GetFactionCapital("order");
            var system_id = args.sys_id;
            var isCapital = system_id == capital_system_id;

            var gen_params = storage.Get("_system_generation", "generation_order");
            if (gen_params == null) {
                gen_params = { step: 0 };
            }

            if (isCapital) {
                var indexOutpost = Math.floor(Math.random() * arrayOfOutpostStations.length);
                var indexScience = Math.floor(Math.random() * arrayOfScienceStations.length);
                var indexMilitary = Math.floor(Math.random() * arrayOfMilitaryStations.length);
                //[arrayOfMiningStations[indexMining], arrayOfOutpostStations[indexOutpost], arrayOfBusinessStations[indexBusiness]]


                return BasesGenerator.AddBasesOfTypes(args, [arrayOfOutpostStations[indexOutpost], arrayOfScienceStations[indexScience], arrayOfMilitaryStations[indexMilitary]], 3, "strict", null, 0); // ["outpost_01", "science_01", "military_01"]
            }
            else {
                if (gen_params.step == 0) {
                    var indexBusiness = Math.floor(Math.random() * arrayOfBusinessStations.length);
                    //generate mining station
                    gen_params.step++;
                    storage.Set("_system_generation", "generation_order", gen_params);
                    return BasesGenerator.AddBasesOfTypes(args, [arrayOfBusinessStations[indexBusiness]], 1, "strict", null, 0); //["business_01"]
                }
                else if (gen_params.step == 1) {
                    var indexMining = Math.floor(Math.random() * arrayOfMiningStations.length);
                    gen_params.step++;
                    storage.Set("_system_generation", "generation_order", gen_params);
                    return BasesGenerator.AddBasesOfTypes(args, [arrayOfMiningStations[indexMining]], 1, "strict", null, 0); //["mining_01"]
                }
                else {
                    //random


                    var indexMining = Math.floor(Math.random() * arrayOfMiningStations.length);
                    var indexBusiness = Math.floor(Math.random() * arrayOfBusinessStations.length);
                    var indexOutpost = Math.floor(Math.random() * arrayOfOutpostStations.length);
                    var indexScience = Math.floor(Math.random() * arrayOfScienceStations.length);
                    var indexMilitary = Math.floor(Math.random() * arrayOfMilitaryStations.length);

                    return BasesGenerator.AddBasesOfTypes(args, [arrayOfOutpostStations[indexOutpost], arrayOfScienceStations[indexScience], arrayOfBusinessStations[indexBusiness], arrayOfMilitaryStations[indexMilitary], arrayOfMiningStations[indexMining]], 1, null, null, 0); // ["outpost_01", "science_01", "business_01", "military_01", "mining_01"]
                }
            }
        },
        CreateSystemObjects: function (args) {
            // Set playlist
            var system_id = args.sys_id;
            generator.SetSystemPlaylists(system_id,
                {
                    "explore": "playlist_explore_generic",
                    "combat": "playlist_combat_generic"
                }
            );

            var level = args.system_info.danger_level;
            var isCapital = (relations.GetFactionCapital("order") == system_id);

            // OBJECTS
            CSGen.CreateGenericStars(args);
            CSGen.CreateGenericPlanets(args);

            if (isCapital) {
                CSGen.CreateGenericObjects(args, 4);
                CSGen.CreateAsteroidsByPrefixes(args, { rock: 10, ice: 1 }, 6, 1, 0);
                NpcGenerator.SpawnNpcsPointsWithTreasure(args, 1);
                //CSGen.CreateMiniStationsRandom(args, "ergrek_terminal", 2, 3, 300, 400);
            }
            else {
                CSGen.CreateGenericObjects(args, 4);
                CSGen.CreateAsteroidsByPrefixes(args, { rock: 10, ice: 1 }, 4, 1, 3);
                NpcGenerator.SpawnNpcsPointsWithTreasure(args, 1);
                NpcGenerator.SpawnPirateBlockposts(args, MathExt.RandRange(1, 3));
                //CSGen.CreateMiniStationsRandom(args, "ergrek_terminal", 1, 3);
                //CSGen.CreateMiniStationsRandom(args, "platform_refuel", 0, 1);
                //CSGen.CreateMiniStationsRandom(args, "platform_repair", 0, 1);
            }

            // friendly NPCS					
            NpcGenerator.SpawnMinersSquad(args);
            for (var i = 0; i < args.bases.length; i++) {
                NpcGenerator.SpawnStationPopulation(args, args.bases[i].id, this.Name);
            }
        },
        NpcGenerationStep: function (args) {
            var isCapital = (relations.GetFactionCapital("order") == args.sys_id);
            var npcsCount = isCapital ? 50 : 70;
            return NpcGenerator.SpawnNpcsStep(args, npcsCount, ["pirate"], "pirates", $i0002, [
                { name: "PirateInWaiting", chance: 5 },
                { name: "RoutePatrolling", chance: 2 },
            ], { class: "pirate" }); // Pirate
        }

    },
    //-------------------------------------------------------------
    // F R E E D O M
    //-------------------------------------------------------------
    FreedomSystem: {
        Name: "FreedomSystem",
        CheckIfFits: function (args) {
            var faction = args.system_info.faction;

            if (faction == "freedom") {
                return true;
            }

            return false;
        },
        Chance: 100,

        QuantityCheck: function (args) {
            return true;
        },
        CreateSystemBases: function (args) {
            var capital = relations.GetFactionCapital("freedom");
            var isCapital = (capital == args.sys_id);

            var gen_params = storage.Get("_system_generation", "generation_freedom");
            if (gen_params == null) {
                gen_params = { step: 0 };
            }

            if (isCapital) {
                var indexMining = Math.floor(Math.random() * arrayOfMiningStations.length);
                var indexBusiness = Math.floor(Math.random() * arrayOfBusinessStations.length);
                var indexOutpost = Math.floor(Math.random() * arrayOfOutpostStations.length);

                return BasesGenerator.AddBasesOfTypes(args, [arrayOfOutpostStations[indexOutpost], arrayOfBusinessStations[indexBusiness], arrayOfMiningStations[indexMining]], 3, "strict", null, 0); //["outpost_01", "business_01", "mining_01"]
            }
            else {
                if (gen_params.step == 0) {
                    var indexMilitary = Math.floor(Math.random() * arrayOfMilitaryStations.length);
                    //generate mining station
                    gen_params.step++;
                    storage.Set("_system_generation", "generation_freedom", gen_params);
                    return BasesGenerator.AddBasesOfTypes(args, [arrayOfMilitaryStations[indexMilitary]], 1, "strict", null, 0); //["military_01"]
                }
                else if (gen_params.step == 1) {
                    var indexScience = Math.floor(Math.random() * arrayOfScienceStations.length);
                    gen_params.step++;
                    storage.Set("_system_generation", "generation_freedom", gen_params);
                    return BasesGenerator.AddBasesOfTypes(args, [arrayOfScienceStations[indexScience]], 1, "strict", null, 0); //["science_01"]
                }
                else {
                    var indexMining = Math.floor(Math.random() * arrayOfMiningStations.length);
                    var indexBusiness = Math.floor(Math.random() * arrayOfBusinessStations.length);
                    var indexOutpost = Math.floor(Math.random() * arrayOfOutpostStations.length);
                    var indexScience = Math.floor(Math.random() * arrayOfScienceStations.length);
                    var indexMilitary = Math.floor(Math.random() * arrayOfMilitaryStations.length);
                    //random
                    return BasesGenerator.AddBasesOfTypes(args, [arrayOfOutpostStations[indexOutpost], arrayOfScienceStations[indexScience], arrayOfBusinessStations[indexBusiness], arrayOfMilitaryStations[indexMilitary], arrayOfMiningStations[indexMining]], 1, null, null, 0); // ["outpost_01", "science_01", "business_01", "military_01", "mining_01"]
                }
            }
        },
        CreateSystemObjects: function (args) {
            // Set playlist
            generator.SetSystemPlaylists(args.sys_id,
                {
                    "explore": "playlist_explore_generic",
                    "combat": "playlist_combat_generic"
                }
            );

            var level = args.system_info.danger_level;
            var isCapital = (relations.GetFactionCapital("freedom") == args.sys_id);

            // OBJECTS
            CSGen.CreateGenericStars(args);
            CSGen.CreateGenericPlanets(args);

            if (isCapital) {
                CSGen.CreateGenericObjects(args, 4);
                CSGen.CreateAsteroidsByPrefixes(args, { rock: 10, ice: 1 }, 6, 1, 0);
                //CSGen.CreateMiniStationsRandom(args, "ergrek_terminal", 2, 3, 300, 400);
            }
            else {
                CSGen.CreateGenericObjects(args, 4);
                CSGen.CreateAsteroidsByPrefixes(args, { rock: 10, ice: 1 }, 4, 1, 3);
                NpcGenerator.SpawnNpcsPointsWithTreasure(args, 1);
                NpcGenerator.SpawnPirateBlockposts(args, MathExt.RandRange(1, 3));
                //CSGen.CreateMiniStationsRandom(args, "ergrek_terminal", 1, 3);
                //CSGen.CreateMiniStationsRandom(args, "platform_refuel", 0, 1);
                //CSGen.CreateMiniStationsRandom(args, "platform_repair", 0, 1);
            }

            // friendly NPCS					
            NpcGenerator.SpawnMinersSquad(args);
            for (var i = 0; i < args.bases.length; i++) {
                NpcGenerator.SpawnStationPopulation(args, args.bases[i].id, this.Name);
            }
        },
        NpcGenerationStep: function (args) {
            var isCapital = (relations.GetFactionCapital("freedom") == args.sys_id);
            var npcsCount = isCapital ? 50 : 70;
            return NpcGenerator.SpawnNpcsStep(args, npcsCount, ["pirate"], "pirates", $i0002, [
                { name: "PirateInWaiting", chance: 5 },
                { name: "RoutePatrolling", chance: 2 },
            ], { class: "pirate" }); // Pirate
        },
    },
    //-------------------------------------------------------------
    // F A N A T I C S
    //-------------------------------------------------------------
    FanaticsSystem: {
        Name: "FanaticsSystem",
        CheckIfFits: function (args) {
            var faction = args.system_info.faction;

            if (faction == "fanatics") {
                return true;
            }

            return false;
        },
        Chance: 100,

        QuantityCheck: function (args) {
            return true;
        },
        CreateSystemBases: function (args) {
            var capital = relations.GetFactionCapital("fanatics");
            var isCapital = (capital == args.sys_id);

            var gen_params = storage.Get("_system_generation", "generation_fanatics");
            if (gen_params == null) {
                gen_params = { step: 0 };
            }

            if (isCapital) {
                var indexMining = Math.floor(Math.random() * arrayOfMiningStations.length);
                var indexOutpost = Math.floor(Math.random() * arrayOfOutpostStations.length);
                var indexScience = Math.floor(Math.random() * arrayOfScienceStations.length);

                //console.PrintError(indexMining + " _ " + indexOutpost + " _ " + indexScience);

                return BasesGenerator.AddBasesOfTypes(args, [arrayOfOutpostStations[indexOutpost], arrayOfScienceStations[indexScience], arrayOfMiningStations[indexMining]], 3, "strict", null, 0); // ["outpost_01", "science_01", "mining_01"]
            }
            else {
                if (gen_params.step == 0) {
                    var indexMilitary = Math.floor(Math.random() * arrayOfMilitaryStations.length);
                    //generate military station
                    gen_params.step++;
                    storage.Set("_system_generation", "generation_fanatics", gen_params);
                    return BasesGenerator.AddBasesOfTypes(args, [arrayOfMilitaryStations[indexMilitary]], 1, "strict", null, 0); //["military_01"]
                }
                else if (gen_params.step == 1) {
                    var indexMining = Math.floor(Math.random() * arrayOfMiningStations.length);

                    //generate mining station
                    gen_params.step++;
                    storage.Set("_system_generation", "generation_fanatics", gen_params);
                    return BasesGenerator.AddBasesOfTypes(args, [arrayOfMiningStations[indexMining]], 1, "strict", null, 0); ///["mining_01"]
                }
                else {
                    var indexMining = Math.floor(Math.random() * arrayOfMiningStations.length);
                    var indexBusiness = Math.floor(Math.random() * arrayOfBusinessStations.length);
                    var indexOutpost = Math.floor(Math.random() * arrayOfOutpostStations.length);
                    var indexScience = Math.floor(Math.random() * arrayOfScienceStations.length);
                    var indexMilitary = Math.floor(Math.random() * arrayOfMilitaryStations.length);
                    //random
                    return BasesGenerator.AddBasesOfTypes(args, [arrayOfOutpostStations[indexOutpost], arrayOfScienceStations[indexScience], arrayOfBusinessStations[indexBusiness], arrayOfMilitaryStations[indexMilitary], arrayOfMiningStations[indexMining]], 1, null, null, 0); //["outpost_01", "science_01", "business_01", "military_01", "mining_01"]
                }
            }
        },
        CreateSystemObjects: function (args) {
            // Set playlist
            generator.SetSystemPlaylists(args.sys_id,
                {
                    "explore": "playlist_explore_generic",
                    "combat": "playlist_combat_generic"
                }
            );

            var level = args.system_info.danger_level;
            var isCapital = (relations.GetFactionCapital("fanatics") == args.sys_id);

            // OBJECTS
            CSGen.CreateGenericStars(args);
            CSGen.CreateGenericPlanets(args);

            if (isCapital) {
                CSGen.CreateGenericObjects(args, 4);
                CSGen.CreateAsteroidsByPrefixes(args, { rock: 10, ice: 1 }, 6, 1, 0);
                //CSGen.CreateMiniStationsRandom(args, "ergrek_terminal", 2, 3, 300, 400);
            }
            else {
                CSGen.CreateGenericObjects(args, 4);
                CSGen.CreateAsteroidsByPrefixes(args, { rock: 10, ice: 1 }, 4, 1, 3);
                NpcGenerator.SpawnNpcsPointsWithTreasure(args, 1);
                NpcGenerator.SpawnPirateBlockposts(args, MathExt.RandRange(1, 3));
                //CSGen.CreateMiniStationsRandom(args, "ergrek_terminal", 1, 3);
                //CSGen.CreateMiniStationsRandom(args, "platform_refuel", 0, 1);
                //CSGen.CreateMiniStationsRandom(args, "platform_repair", 0, 1);
            }

            // friendly NPCS					
            NpcGenerator.SpawnMinersSquad(args);
            for (var i = 0; i < args.bases.length; i++) {
                NpcGenerator.SpawnStationPopulation(args, args.bases[i].id, this.Name);
            }
        },
        NpcGenerationStep: function (args) {
            var isCapital = (relations.GetFactionCapital("fanatics") == args.sys_id);
            var npcsCount = isCapital ? 50 : 70;
            return NpcGenerator.SpawnNpcsStep(args, npcsCount, ["pirate"], "pirates", $i0002, [
                { name: "PirateInWaiting", chance: 5 },
                { name: "RoutePatrolling", chance: 2 },
            ], { class: "pirate" }); // Pirate
        },
    },
    //-------------------------------------------------------------
    // P I R A T E S
    //-------------------------------------------------------------
    PiratesSystem: {
        Name: "PiratesSystem",
        CheckIfFits: function (args) {
            var faction = args.system_info.faction;

            var res = (faction == "pirates");
            return res;
        },
        Chance: 100,

        QuantityCheck: function (args) {
            return true;
        },
        CreateSystemBases: function (args) {
            var isCapital = (relations.GetFactionCapital("pirates") == args.sys_id);

            if (isCapital) {
                var indexOutpost = Math.floor(Math.random() * arrayOfOutpostStations.length);
                var indexMilitary = Math.floor(Math.random() * arrayOfMilitaryStations.length);

                return BasesGenerator.AddBasesOfTypes(args, [arrayOfMilitaryStations[indexMilitary], arrayOfOutpostStations[indexOutpost]], 2, "strict", null, 0); //["military_01", "outpost_01"]
            }
            else {
                var indexOutpost = Math.floor(Math.random() * arrayOfOutpostStations.length);
                var indexMilitary = Math.floor(Math.random() * arrayOfMilitaryStations.length);
                return BasesGenerator.AddBasesOfTypes(args, [arrayOfMilitaryStations[indexMilitary], arrayOfOutpostStations[indexOutpost]], 1, null, null, 0); //["military_01", "outpost_01"]
            }
        },
        CreateSystemObjects: function (args) {
            // Set playlist
            generator.SetSystemPlaylists(args.sys_id,
                {
                    "explore": "playlist_explore_generic",
                    "combat": "playlist_combat_generic"
                }
            );

            var level = args.system_info.danger_level;
            var isCapital = (relations.GetFactionCapital("pirates") == args.sys_id);

            // OBJECTS
            CSGen.CreateGenericStars(args);
            CSGen.CreateGenericPlanets(args);

            if (isCapital) {
                CSGen.CreateGenericObjects(args, 4);
                CSGen.CreateAsteroidsByPrefixes(args, { rock: 10, ice: 1 }, 6, 1, 0);
                NpcGenerator.SpawnNpcsPointsWithTreasure(args, 5);
                NpcGenerator.SpawnPirateBlockposts(args, MathExt.RandRange(3, 5));
                //CSGen.CreateMiniStationsRandom(args, "ergrek_terminal", 2, 3, 300, 400);
            }
            else {
                CSGen.CreateGenericObjects(args, 4);
                CSGen.CreateAsteroidsByPrefixes(args, { rock: 10, ice: 1 }, 4, 1, 3);
                NpcGenerator.SpawnNpcsPointsWithTreasure(args, 5);
                NpcGenerator.SpawnPirateBlockposts(args, MathExt.RandRange(3, 5));
                //CSGen.CreateMiniStationsRandom(args, "ergrek_terminal", 1, 3);
                //CSGen.CreateMiniStationsRandom(args, "platform_refuel", 0, 1);
                //CSGen.CreateMiniStationsRandom(args, "platform_repair", 0, 1);
            }

            // friendly NPCS					
            for (var i = 0; i < args.bases.length; i++) {
                NpcGenerator.SpawnStationPopulation(args, args.bases[i].id, this.Name);
            }
        },
        NpcGenerationStep: function (args) {
            var isCapital = (relations.GetFactionCapital("pirates") == args.sys_id);
            var npcsCount = isCapital ? 90 : 70; // even more pirates in the capital system!
            return NpcGenerator.SpawnNpcsStep(args, npcsCount, ["pirate"], "pirates", $i0002, [
                { name: "PirateInWaiting", chance: 5 },
                { name: "RoutePatrolling", chance: 2 },
            ], { class: "pirate" }); // Pirate
        },
    },
    //-------------------------------------------------------------
    // G A R B A G E
    //-------------------------------------------------------------
    GarbageSystemPreset:
    {
        Name: "GarbageSystemPreset",
        CheckIfFits: function (args) {
            var faction = args.system_info.faction;

            var res =
                args.sys_id != spawn.GetSystemID() &&
                faction == "none";
            return res;
        },
        Chance: 100,

        QuantityCheck: function (args) {
            if (this.Quantity >= 0.04 * args.NumOfSystems) {
                return false;
            }
            return true;
        },
        CreateSystemBases: function (args) {
            var pick = MathExt.RandRange(0, 100);
            if (pick < 10) {
                var indexMining = Math.floor(Math.random() * arrayOfMiningStations.length);
                var indexBusiness = Math.floor(Math.random() * arrayOfBusinessStations.length);
                var indexOutpost = Math.floor(Math.random() * arrayOfOutpostStations.length);
                var indexScience = Math.floor(Math.random() * arrayOfScienceStations.length);

                return BasesGenerator.AddBasesOfTypes(args, [arrayOfOutpostStations[indexOutpost], arrayOfScienceStations[indexScience], arrayOfBusinessStations[indexBusiness], arrayOfMiningStations[indexMining]], 1, null, null, 0); //["mining_01", "outpost_01", "science_01", "business_01"]
            }
            else {
                return [];
            }
        },
        CreateSystemObjects: function (args) {
            // Set playlist
            generator.SetSystemPlaylists(args.sys_id,
                {
                    "explore": "playlist_explore_garbage",
                    "combat": "playlist_combat_garbage"
                }
            );

            var level = args.system_info.danger_level;

            // OBJECTS
            CSGen.CreateGenericStars(args);
            //CSGen.CreateGenericPlanets(args); // no planets
            CSGen.CreateGenericObjects(args, 1); // a lot of objects
            CSGen.CreateAsteroidsByPrefixes(args, { rock: 4, ice: 1 }, 4, 1, 3);

            //CSGen.CreateMiniStationsRandom(args, "ergrek_terminal", 1, 3);
            //CSGen.CreateMiniStationsRandom(args, "platform_refuel", 0, 1);
            //CSGen.CreateMiniStationsRandom(args, "platform_repair", 0, 1);

            // NPCS
            for (var i = 0; i < args.bases.length; i++) {
                NpcGenerator.SpawnStationPopulation(args, args.bases[i].id, this.Name);
            }

            NpcGenerator.SpawnNpcsPointsWithTreasure(args, 3);

            NpcGenerator.SpawnPirateBlockposts(args, MathExt.RandRange(2, 4));

            // TODO: add something cool, like random treasure crates or mines			
        },
        NpcGenerationStep: function (args) {
            return NpcGenerator.SpawnNpcsStep(args, 55, ["pirate"], "pirates", $i0002, [
                { name: "PirateInWaiting", chance: 5 },
                { name: "RoutePatrolling", chance: 2 },
            ], { class: "pirate" }); // Pirate
        },
    },
    //-------------------------------------------------------------
    // B I G  B A T T L E
    //-------------------------------------------------------------
    // Info: single instance system, needed for quest for barrich device
    // basically, this is the system where a big battle between humand and xengatarn was
    BigBattleSystemPreset:
    {
        Name: "BigBattleSystemPreset",
        CheckIfFits: function (args) {
            var faction = args.system_info.faction;

            var res =
                args.sys_id != spawn.GetSystemID() &&
                faction == "none" &&
                args.system_info.danger_level > 30;
            return res;
        },
        Chance: 100,

        QuantityCheck: function (args) {
            if (this.Quantity >= 1) {
                return false;
            }
            else {
                return true;
            }
        },
        CreateSystemBases: function (args) {
            //no bases
            return [];
        },
        CreateSystemObjects: function (args) {
            // Set playlist
            generator.SetSystemPlaylists(args.sys_id,
                {
                    // uses playlists from garbage system
                    "explore": "playlist_explore_garbage",
                    "combat": "playlist_combat_garbage"
                }
            );

            storage.Set("_system_generation", "big_battle_system", args.sys_id);
            //console.PrintError("Big battle system generated: " + args.sys_id);

            var level = args.system_info.danger_level;

            // OBJECTS
            CSGen.CreateGenericStars(args);
            CSGen.CreateGenericPlanets(args);
            CSGen.CreateGenericObjects(args, 2);
            CSGen.CreateAsteroidsByPrefixes(args, { rock: 4, ice: 1 }, 4, 1, 3);

            //CSGen.CreateMiniStationsRandom(args, "ergrek_terminal", 1, 3);
            //CSGen.CreateMiniStationsRandom(args, "platform_refuel", 0, 1);
            //CSGen.CreateMiniStationsRandom(args, "platform_repair", 0, 1);

            // no npcs

            // treasures and pirate blockposts
            NpcGenerator.SpawnNpcsPointsWithTreasure(args, 6);

            NpcGenerator.SpawnPirateBlockposts(args, MathExt.RandRange(2, 4));

            // TODO: add ships wreckages
        },
        NpcGenerationStep: function (args) {
            return NpcGenerator.SpawnNpcsStep(args, 55, ["pirate"], "pirates", $i0002, [
                { name: "PirateInWaiting", chance: 5 },
                { name: "RoutePatrolling", chance: 2 },
            ], { class: "pirate" }); // Pirate
        },
    },
    //-------------------------------------------------------------
    // G A S  A N O M A L Y
    //-------------------------------------------------------------
    GasSystemPresets: {
        Name: "GasSystemPresets",
        CheckIfFits: function (args) {
            var faction = args.system_info.faction;

            var res =
                args.sys_id != spawn.GetSystemID() &&
                faction == "none" &&
                args.system_info.danger_level > 20;
            return res;
        },
        Chance: 100,

        QuantityCheck: function (args) {
            if (this.Quantity >= 0.04 * args.NumOfSystems) {
                return false;
            }
            return true;
        },
        CreateSystemBases: function (args) {
            var indexScience = Math.floor(Math.random() * arrayOfScienceStations.length);

            return BasesGenerator.AddBasesOfTypes(args, [arrayOfScienceStations[indexScience]], 1, null, null, 0); //["science_01"]
        },
        CreateSystemObjects: function (args) {
            // Set playlist
            generator.SetSystemPlaylists(args.sys_id,
                {
                    "explore": "playlist_explore_gas",
                    "combat": "playlist_combat_gas"
                }
            );

            var level = args.system_info.danger_level;

            // OBJECTS
            CSGen.CreateGenericStars(args);
            CSGen.CreateGenericPlanets(args); // no planets
            CSGen.CreateGenericObjects(args, 2); // a lot of objects
            CSGen.CreateAsteroidsByPrefixes(args, { rock: 2, ice: 3 }, 4, 1, 3);

            //CSGen.CreateMiniStationsRandom(args, "ergrek_terminal", 1, 3);
            //CSGen.CreateMiniStationsRandom(args, "platform_refuel", 0, 1);
            //CSGen.CreateMiniStationsRandom(args, "platform_repair", 0, 1);

            //generator.AddAnomaly(args.sys_id, "radar_disable", {});
            generator.AddSystemTag(args.sys_id, "show_clouds", "yes");

            for (var i = 0; i < args.bases.length; i++) {
                NpcGenerator.SpawnStationPopulation(args, args.bases[i].id, this.Name);
            }

            NpcGenerator.SpawnNpcsPointsWithTreasure(args, 5);

            NpcGenerator.SpawnPirateBlockposts(args, MathExt.RandRange(3, 5));

            // TODO: add something cool, like random treasure crates or mines
        },
        NpcGenerationStep: function (args) {
            return NpcGenerator.SpawnNpcsStep(args, 55, ["pirate"], "pirates", $i0002, [
                { name: "PirateInWaiting", chance: 5 },
                { name: "RoutePatrolling", chance: 2 },
            ], { class: "pirate" }); // Pirate
        },
    },
    //-------------------------------------------------------------
    // R A D I A T I O N
    //-------------------------------------------------------------
    RadiationSystemPreset: {
        Name: "RadiationSystemPreset",
        CheckIfFits: function (args) {
            var faction = args.system_info.faction;

            var res =
                args.sys_id != spawn.GetSystemID() &&
                faction == "none" &&
                args.system_info.danger_level > 40;
            return res;
        },
        Chance: 100,

        QuantityCheck: function (args) {
            if (this.Quantity >= 0.04 * args.NumOfSystems) {
                return false;
            }
            return true;
        },
        CreateSystemBases: function (args) {
            return [];
        },
        CreateSystemObjects: function (args) {
            // Set playlist
            generator.SetSystemPlaylists(args.sys_id,
                {
                    "explore": "playlist_explore_radiation",
                    "combat": "playlist_combat_radiation"
                }
            );

            var level = args.system_info.danger_level;

            // OBJECTS
            CSGen.CreateGenericStars(args);
            CSGen.CreateGenericPlanets(args); // no planets
            CSGen.CreateGenericObjects(args, 2); // a lot of objects
            CSGen.CreateAsteroidsByPrefixes(args, { rock: 2, lava: 3 }, 4, 1, 3);

            //generator.AddAnomaly(args.sys_id, "radiation", { radiation_level: 0.01 });

            NpcGenerator.SpawnNpcsPointsWithTreasure(args, 6);

            // TODO: add something cool, like random treasure crates or mines
        },
        NpcGenerationStep: function (args) {
            return NpcGenerator.SpawnNpcsStep(args, 50, ["alien"], "aliens", $i0003, [
                { name: "PirateInWaiting", chance: 5 },
                { name: "RoutePatrolling", chance: 2 },
            ], { class: "alien" }); // Alien
        },
    },
    //-------------------------------------------------------------
    // M I N E F I E L D
    //-------------------------------------------------------------
    MinefieldPreset: {
        Name: "MinefieldPreset",
        CheckIfFits: function (args) {
            var faction = args.system_info.faction;

            var res =
                args.sys_id != spawn.GetSystemID() &&
                faction == "none" &&
                args.system_info.danger_level > 15;
            return res;
        },
        Chance: 100,

        QuantityCheck: function (args) {
            if (this.Quantity >= 0.05 * args.NumOfSystems) {
                return false;
            }
            return true;
        },
        CreateSystemBases: function (args) {
            return [];
        },
        CreateSystemObjects: function (args) {
            // Set playlist
            generator.SetSystemPlaylists(args.sys_id,
                {
                    "explore": "playlist_explore_minefield",
                    "combat": "playlist_combat_minefield"
                }
            );

            var level = args.system_info.danger_level;

            // OBJECTS
            CSGen.CreateGenericStars(args);
            CSGen.CreateGenericPlanets(args);
            CSGen.CreateGenericObjects(args, 2); // a lot of objects
            CSGen.CreateMinefields(args, 5, 5); // a lot of objects
            CSGen.CreateAsteroidsByPrefixes(args, { rock: 10, lava: 1 }, 4, 1, 3);

            //CSGen.CreateMiniStationsRandom(args, "ergrek_terminal", 1, 3);
            //CSGen.CreateMiniStationsRandom(args, "platform_refuel", 0, 1);
            //CSGen.CreateMiniStationsRandom(args, "platform_repair", 0, 1);

            NpcGenerator.SpawnNpcsPointsWithTreasure(args, 5);

            NpcGenerator.SpawnPirateBlockposts(args, MathExt.RandRange(3, 5));
        },
        NpcGenerationStep: function (args) {
            return NpcGenerator.SpawnNpcsStep(args, 50, ["alien"], "aliens", $i0003, "PirateInWaiting", { class: "alien" }); // Alien
        },
    },
    //-------------------------------------------------------------
    // B L A C K   H O L E
    //-------------------------------------------------------------

    BlackholePreset: {
        Name: "BlackholePreset",
        CheckIfFits: function (args) {
            var faction = args.system_info.faction;

            var res =
                args.sys_id != spawn.GetSystemID() &&
                faction == "none" &&
                args.system_info.danger_level > 30;
            return res;
        },
        Chance: 100,

        QuantityCheck: function (args) {
            if (this.Quantity >= 2) {
                return false;
            }
            return true;
        },
        CreateSystemBases: function (args) {
            return [];
        },
        CreateSystemObjects: function (args) {
            // Set playlist
            generator.SetSystemPlaylists(args.sys_id,
                {
                    "explore": "playlist_explore_blackhole",
                    "combat": "playlist_combat_blackhole"
                }
            );

            var level = args.system_info.danger_level;

            // OBJECTS
            CSGen.CreateBlackHole(args);
            //CSGen.CreateGenericPlanets(args); // no planets
            CSGen.CreateGenericObjects(args, 3); // a lot of objects
            CSGen.CreateAsteroidsByPrefixes(args, { lava: 1 }, 4, 1, 3);

            generator.AddAnomaly(args.sys_id, "gravity_field",
                {
                    gravity_center: "0;0",
                    magnitude: 0.14,
                    gravity_radius: 250,
                    magnitude_on_border: 0.003,
                    gravity_center_damage_radius: 10,
                    gravity_center_damage: 250,
                });

            NpcGenerator.SpawnNpcsPointsWithTreasure(args, 2);

            NpcGenerator.SpawnPirateBlockposts(args, MathExt.RandRange(3, 5));
        },
        NpcGenerationStep: function (args) {
            return NpcGenerator.SpawnNpcsStep(args, 40, ["alien", "blackhole"], "aliens", $i0003, [
                { name: "PirateInWaiting", chance: 5 },
                { name: "RoutePatrolling", chance: 2 },
            ], { class: "alien" }); // Alien
        },
    },
}; //eof SystemsPresets