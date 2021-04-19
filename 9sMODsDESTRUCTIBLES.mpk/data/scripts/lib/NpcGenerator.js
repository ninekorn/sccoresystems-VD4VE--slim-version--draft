

/*
 ======================================================================
 Special library for npc generation
 ======================================================================
 */

include(pickByChance.js);

using(game);
using(generator);
using(console);
using(ship);
using(storage);
using(relations);
using(npc);

var NpcGenerator = {
    //-------------------------------------------------------------------
    // Spawn standard npcs
    //-------------------------------------------------------------------
    SpawnMinersSquad: function (args) {

        /*// add neutral npc's to system
        var MinersNum = MathExt.RandInt() % 3 + 2;
        var ang;
        var inf = args.system_info;
        var new_coord;
        var bases = args.bases;
        var sys_id = args.sys_id;

        if (bases.length == 0) {
            return;
        }

        for (var i = 0; i < MinersNum; i++) {
            ang = MathExt.RandDouble() * Math.PI * 2.0;
            new_coord = {
                x: bases[0].coord.x + 25 * Math.cos(ang),
                y: bases[0].coord.y + 25 * Math.sin(ang)
            };

            //spawn miner
            var level = inf.danger_level + Math.round(MathExt.RandDouble() * 4 - 2);
            level = utils.Clamp(level, 1, 100);
            var id = generator.AddNPCShipToSystem($i0001, "TechMiner", level, "special_human_techship", sys_id, new_coord.x + MathExt.RandInt() % 3, new_coord.y + MathExt.RandInt() % 3, { class: "miner", meta: "human" }); // Miner
            relations.SetShipFaction(id, inf.faction);
        }*/
    },
    //-------------------------------------------------------------------
    // Spawn diablo-like chest, surrounded by enemies
    //-------------------------------------------------------------------
    SpawnNpcsPointsWithTreasure: function (args, numOfPoints) {
        /*var inf = generator.GetSystemByID(args.sys_id);
        var attempt = 0;
        for (var i = 0; i < numOfPoints; i++) {
            // pick coordinates
            var coords = CSGen.GetRandomCoordWithinWorldBounds(args);
            if (!CSGen.CoordInSafeZonesRad(args, coords, 150)
                && !generator.AreThereAnyObjects(args.sys_id, coords.x, coords.y, 20)) {
                var item_list = "droplist_generic_relic_crate_01";
                if (inf.faction == "aliens") {
                    item_list = "droplist_generic_relic_crate_xengatarn";
                }

                // add treasure chest
                var cont_id = generator.AddContainer(
                    args.sys_id,
                    coords.x,
                    coords.y,
                    "crate_02",
                    item_list,
                    { itemlist: item_list });

                // add mighty guardian
                var possibleNpcs = generator.GetNpcsForLevelByTags(["treasure_guard"], inf.danger_level);
                var l = pickByChance(possibleNpcs);
                var random_type = possibleNpcs[l].xml_type;

                var id = generator.AddNPCShipToSystem($i0002, "TreasureGuard", inf.danger_level, random_type, args.sys_id, coords.x + MathExt.RandInt() % 3, coords.y + MathExt.RandInt() % 3, { class: "pirate", crate: cont_id }); // Pirate
                relations.SetShipFaction(id, "pirates");
            }
            else {
                attempt++;
                i--;
                if (attempt > 200) {
                    return;
                }
            }
        }*/
        return;
    },
    //-------------------------------------------------------------------
    // Spawn pirate blockpost
    //-------------------------------------------------------------------
    SpawnPirateBlockposts: function (args, numOfPoints) {
        /*var system_id = args.sys_id;
        var inf = generator.GetSystemByID(system_id);
        var tries = 0;
        for (var i = 0; i < numOfPoints; i++) {
            // pick coordinates
            var coords = CSGen.GetRandomCoordWithinWorldBounds(args);
            if (CSGen.CoordInSafeZonesRad(args, coords, 150)
                || generator.AreThereAnyObjects(system_id, coords.x, coords.y, 20)) {
                tries++;
                i--;
                if (tries > 200) {
                    return;
                }

                continue;
            }

            // add pirate den	
            var rescoord = { x: coords.x + MathExt.RandInt() % 3, y: coords.y + MathExt.RandInt() % 3 };
            var id = generator.AddNPCShipToSystem($i0003, "Turret", inf.danger_level + 10, "special_pirates_platform", system_id, rescoord.x, rescoord.y, { class: "pirate" }); // Pirate Den
            relations.SetShipFaction(id, "pirates");

            // add a couple of minions
            var minionsCount = MathExt.RandRange(1, 3);
            for (var j = 0; j < minionsCount; j++) {
                var ang = MathExt.RandRangeDouble(0, Math.PI * 2);
                var id = generator.AddNPCShipToSystem($i0004, "PirateInWaiting", inf.danger_level + 10, "generic_pirates_fang_01", system_id, rescoord.x + 19 * Math.cos(ang), rescoord.y + 19 * Math.sin(ang), { class: "pirate" }); // Pirate Guard
                relations.SetShipFaction(id, "pirates");
            }
        }*/
    },



    /*
    =======================================================
    SpawnTurretsOfType   
    spawns turrets of specified parameters. ACtually, can be used to spawn any NPCs
    in circle (with random start offset)
    =======================================================
    */

    SpawnTurretsOfType: function (args, coords, quantity, level, turret_radius, ai_behavior, xml_ship, faction, tags, parsedAngle, base_xml, sys_id, base_id) {
        var randomXML = Math.floor(Math.random() * ((arrayOfTurretsXML.length) - 0) + 0);
        tags.xmlnpcship = arrayOfTurretsXML[randomXML];//"generic_pirates_shuttle_01";//shipByRandom.xml_type;

        var arrayOfTurretsAtStation = [];

        /*arrayOfTurretsAtStation.push({ loc: 0, id: -1 });
        arrayOfTurretsAtStation.push({ loc: 1, id: -1 });
        arrayOfTurretsAtStation.push({ loc: 2, id: -1 });
        arrayOfTurretsAtStation.push({ loc: 3, id: -1 });
        arrayOfTurretsAtStation.push({ loc: 4, id: -1 });
        arrayOfTurretsAtStation.push({ loc: 5, id: -1 });
        arrayOfTurretsAtStation.push({ loc: 6, id: -1 });
        arrayOfTurretsAtStation.push({ loc: 7, id: -1 });*/

        storage.SetGlobal("system_" + sys_id + "_base_" + base_id + "_turrets", arrayOfTurretsAtStation);
        arrayOfTurretsAtStation = [];


        var pointFrontX = (1 * Math.cos(parsedAngle * Math.PI / 180)) + coords.x; // coords.x + 30;
        var pointFrontY = (1 * Math.sin(parsedAngle * Math.PI / 180)) + coords.y; // coords.y - 5; //- 5

        var playerDirX = pointFrontX - coords.x;
        var playerDirY = pointFrontY - coords.y;

        var frontDirPlayer = { x: playerDirX, y: playerDirY };
        var backDirPlayer = { x: -playerDirX, y: -playerDirY };
        var rightDirPlayer = { x: playerDirY, y: -playerDirX };
        var leftDirPlayer = { x: -playerDirY, y: playerDirX };

        if (base_xml.substring(0, 7) == "outpost") {
            //OUTTER TURRET RIGHT
            //OUTTER TURRET RIGHT
            offsetX = 18;
            offsetY = 18;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 9;
            offsetY = 9;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id0 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id0, 0);
            relations.SetShipFaction(id0, faction);
            arrayOfTurretsAtStation.push({ loc: 0, id: id0, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });



            offsetX = 18;
            offsetY = 18;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = -11;
            offsetY = -11;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);


            var id1 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id1, 0);
            relations.SetShipFaction(id1, faction);
            //OUTTER TURRET RIGHT
            //OUTTER TURRET RIGHT
            arrayOfTurretsAtStation.push({ loc: 1, id: id1, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });




            //OUTTER TURRET LEFT
            //OUTTER TURRET LEFT
            offsetX = 4;
            offsetY = 4;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 9;
            offsetY = 9;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);


            var id2 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id2, 0);
            relations.SetShipFaction(id2, faction);
            arrayOfTurretsAtStation.push({ loc: 2, id: id2, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });



            offsetX = 4;
            offsetY = 4;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = -11;
            offsetY = -11;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);


            var id3 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id3, 0);
            relations.SetShipFaction(id3, faction);
            //OUTTER TURRET LEFT
            //OUTTER TURRET LEFT
            arrayOfTurretsAtStation.push({ loc: 3, id: id3, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });


            //INNER TURRET RIGHT
            //INNER TURRET RIGHT
            offsetX = 14;
            offsetY = 14;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));  //y
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));  //y

            offsetX = 5;
            offsetY = 5;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1)); //x
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1)); //x

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);


            var id4 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id4, 0);
            relations.SetShipFaction(id4, faction);
            arrayOfTurretsAtStation.push({ loc: 4, id: id4, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });


            offsetX = 14;
            offsetY = 14;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = -7;
            offsetY = -7;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);


            var id5 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id5, 0);
            relations.SetShipFaction(id5, faction);
            //INNER TURRET RIGHT
            //INNER TURRET RIGHT
            arrayOfTurretsAtStation.push({ loc: 5, id: id5, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });


            //INNER TURRET LEFT
            //INNER TURRET LEFT
            offsetX = 8;
            offsetY = 8;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 5;
            offsetY = 5;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);


            var id6 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id6, 0);
            relations.SetShipFaction(id6, faction);
            arrayOfTurretsAtStation.push({ loc: 6, id: id6, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });


            offsetX = 8;
            offsetY = 8;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = -7;
            offsetY = -7;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);


            var id7 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id7, 0);
            relations.SetShipFaction(id7, faction);
            arrayOfTurretsAtStation.push({ loc: 7, id: id7, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });



            //INNER TURRET LEFT
            //INNER TURRET LEFT

            //for missions of damaged bases to help repair against invading other factions and to currently test the repair option of the station manager.

            /*var turretHP = ship.GetCurrentValue(id0, "structure") * 0.15;
            ship.SetCurrentValue(id0, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id1, "structure") * 0.15;
            ship.SetCurrentValue(id1, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id2, "structure") * 0.15;
            ship.SetCurrentValue(id2, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id3, "structure") * 0.15;
            ship.SetCurrentValue(id3, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id4, "structure") * 0.15;
            ship.SetCurrentValue(id4, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id5, "structure") * 0.15;
            ship.SetCurrentValue(id5, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id6, "structure") * 0.15;
            ship.SetCurrentValue(id6, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id7, "structure") * 0.15;
            ship.SetCurrentValue(id7, "structure", turretHP);*/





            storage.SetGlobal("system_" + sys_id + "_base_" + base_id + "_turrets", arrayOfTurretsAtStation);
        }
        else if (base_xml.substring(0, 6) == "mining") {



            arrayOfTurretsAtStation = [];
            //INNER TURRET RIGHT
            //INNER TURRET RIGHT
            var offsetX = -10;
            var offsetY = -10;
            var pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            var pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 15;
            offsetY = 15;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            var turretformationWaypoint = { x: pointRightX, y: pointRightY };
            var turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id0 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id0, 0);
            relations.SetShipFaction(id0, faction);
            arrayOfTurretsAtStation.push({ loc: 0, id: id0, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });



            offsetX = -8;
            offsetY = -8;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 15;
            offsetY = 15;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id1 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id1, 0);
            relations.SetShipFaction(id1, faction);
            //INNER TURRET RIGHT
            //INNER TURRET RIGHT
            arrayOfTurretsAtStation.push({ loc: 1, id: id1, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });


            //INNER TURRET RIGHT
            //INNER TURRET RIGHT
            var offsetX = -10;
            var offsetY = -10;
            var pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            var pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));
            offsetX = 17;
            offsetY = 17;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));
            var turretformationWaypoint = { x: pointRightX, y: pointRightY };
            var turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);
            var id2 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id2, 0);
            relations.SetShipFaction(id2, faction);
            arrayOfTurretsAtStation.push({ loc: 2, id: id2, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });


            offsetX = -8;
            offsetY = -8;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 17;
            offsetY = 17;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id3 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id3, 0);
            relations.SetShipFaction(id3, faction);
            arrayOfTurretsAtStation.push({ loc: 3, id: id3, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });


            offsetX = -9;
            offsetY = -9;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = -10;
            offsetY = -10;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id4 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id4, 0);
            relations.SetShipFaction(id4, faction);
            //INNER TURRET RIGHT
            //INNER TURRET RIGHT
            arrayOfTurretsAtStation.push({ loc: 4, id: id4, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });


            /*var turretHP = ship.GetCurrentValue(id0, "structure") * 0.15;
            ship.SetCurrentValue(id0, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id1, "structure") * 0.15;
            ship.SetCurrentValue(id1, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id2, "structure") * 0.15;
            ship.SetCurrentValue(id2, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id3, "structure") * 0.15;
            ship.SetCurrentValue(id3, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id4, "structure") * 0.15;
            ship.SetCurrentValue(id4, "structure", turretHP);*/



            storage.SetGlobal("system_" + sys_id + "_base_" + base_id + "_turrets", arrayOfTurretsAtStation);
        }
        else if (base_xml.substring(0, 8) == "military") //will be 4 turrets
        {
            arrayOfTurretsAtStation = [];

            //INNER TURRET RIGHT
            //INNER TURRET RIGHT
            var offsetX = -10;
            var offsetY = -10;
            var pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            var pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 15;
            offsetY = 15;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            var turretformationWaypoint = { x: pointRightX, y: pointRightY };
            var turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id0 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id0, 0);
            relations.SetShipFaction(id0, faction);
            arrayOfTurretsAtStation.push({ loc: 0, id: id0, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });




            offsetX = -8;
            offsetY = -8;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 15;
            offsetY = 15;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id1 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id1, 0);
            relations.SetShipFaction(id1, faction);
            //INNER TURRET RIGHT
            //INNER TURRET RIGHT
            arrayOfTurretsAtStation.push({ loc: 1, id: id1, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });


            //INNER TURRET RIGHT
            //INNER TURRET RIGHT
            var offsetX = -10;
            var offsetY = -10;
            var pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            var pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 17;
            offsetY = 17;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            var turretformationWaypoint = { x: pointRightX, y: pointRightY };
            var turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id2 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id2, 0);
            relations.SetShipFaction(id2, faction);
            arrayOfTurretsAtStation.push({ loc: 2, id: id2, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });


            offsetX = -8;
            offsetY = -8;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 17;
            offsetY = 17;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id3 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id3, 0);
            relations.SetShipFaction(id3, faction);
            //INNER TURRET RIGHT
            //INNER TURRET RIGHT
            arrayOfTurretsAtStation.push({ loc: 3, id: id3, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });




            //INNER WALL TURRET RIGHT
            //INNER WALL TURRET RIGHT
            var offsetX = -12;
            var offsetY = -12;
            var pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            var pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 9;
            offsetY = 9;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            var turretformationWaypoint = { x: pointRightX, y: pointRightY };
            var turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id4 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id4, 0);
            relations.SetShipFaction(id4, faction);
            arrayOfTurretsAtStation.push({ loc: 4, id: id4, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });


            offsetX = -6;
            offsetY = -6;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 9;
            offsetY = 9;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id5 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id5, 0);
            relations.SetShipFaction(id5, faction);
            //INNER WALL TURRET RIGHT
            //INNER WALL TURRET RIGHT





            arrayOfTurretsAtStation.push({ loc: 5, id: id5, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });

            //OUTER TURRET DOWN
            //OUTER TURRET DOWN

            var offsetX = -31;
            var offsetY = -31;
            var pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            var pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 24;
            offsetY = 24;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            var turretformationWaypoint = { x: pointRightX, y: pointRightY };
            var turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id6 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id6, 0);
            relations.SetShipFaction(id6, faction);

            arrayOfTurretsAtStation.push({ loc: 6, id: id6, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });

            var offsetX = -28;
            var offsetY = -28;
            var pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            var pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 20;
            offsetY = 20;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            var turretformationWaypoint = { x: pointRightX, y: pointRightY };
            var turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id7 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id7, 0);
            relations.SetShipFaction(id7, faction);
            arrayOfTurretsAtStation.push({ loc: 7, id: id7, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });

            var offsetX = -19;
            var offsetY = -19;
            var pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            var pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 21;
            offsetY = 21;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            var turretformationWaypoint = { x: pointRightX, y: pointRightY };
            var turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id8 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id8, 0);
            relations.SetShipFaction(id8, faction);
            //OUTER TURRET DOWN
            //OUTER TURRET DOWN
            arrayOfTurretsAtStation.push({ loc: 8, id: id8, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });



            //OUTER TURRET UP
            //OUTER TURRET UP

            var offsetX = 13;
            var offsetY = 13;
            var pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            var pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 24;
            offsetY = 24;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            var turretformationWaypoint = { x: pointRightX, y: pointRightY };
            var turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id9 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id9, 0);
            relations.SetShipFaction(id9, faction);
            arrayOfTurretsAtStation.push({ loc: 9, id: id9, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });


            var offsetX = 9;
            var offsetY = 9;
            var pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            var pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 20;
            offsetY = 20;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            var turretformationWaypoint = { x: pointRightX, y: pointRightY };
            var turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id10 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id10, 0);
            relations.SetShipFaction(id10, faction);
            arrayOfTurretsAtStation.push({ loc: 10, id: id10, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });

            var offsetX = 1;
            var offsetY = 1;
            var pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            var pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 21;
            offsetY = 21;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            var turretformationWaypoint = { x: pointRightX, y: pointRightY };
            var turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id11 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id11, 0);
            relations.SetShipFaction(id11, faction);
            //OUTER TURRET UP
            //OUTER TURRET UP
            arrayOfTurretsAtStation.push({ loc: 11, id: id11, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });


            //BACK OUTER TURRET UP
            //BACK OUTER TURRET UP
            var offsetX = 12;
            var offsetY = 12;

            var pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            var pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = -25;
            offsetY = -25;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            var turretformationWaypoint = { x: pointRightX, y: pointRightY };
            var turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id12 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id12, 0);
            relations.SetShipFaction(id12, faction);
            arrayOfTurretsAtStation.push({ loc: 12, id: id12, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });

            var offsetX = 8;
            var offsetY = 8;

            var pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            var pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = -27;
            offsetY = -27;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            var turretformationWaypoint = { x: pointRightX, y: pointRightY };
            var turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id13 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id13, 0);
            relations.SetShipFaction(id13, faction);
            //BACK OUTER TURRET UP
            //BACK OUTER TURRET UP

            arrayOfTurretsAtStation.push({ loc: 13, id: id13, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });


            //BACK OUTER TURRET DOWN
            //BACK OUTER TURRET DOWN
            var offsetX = -30;
            var offsetY = -30;

            var pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            var pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = -25;
            offsetY = -25;

            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            var turretformationWaypoint = { x: pointRightX, y: pointRightY };
            var turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id14 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id14, 0);
            relations.SetShipFaction(id14, faction);
            arrayOfTurretsAtStation.push({ loc: 14, id: id14, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });



            var offsetX = -26;
            var offsetY = -26;
            var pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            var pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = -27;
            offsetY = -27;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            var turretformationWaypoint = { x: pointRightX, y: pointRightY };
            var turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id15 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id15, 0);
            relations.SetShipFaction(id15, faction);
            //BACK OUTER TURRET DOWN
            //BACK OUTER TURRET DOWN
            arrayOfTurretsAtStation.push({ loc: 15, id: id15, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });


            offsetX = -9;
            offsetY = -9;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = -10;
            offsetY = -10;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id16 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id16, 0);
            relations.SetShipFaction(id16, faction);
            arrayOfTurretsAtStation.push({ loc: 16, id: id16, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });




            /*var turretHP = ship.GetCurrentValue(id0, "structure") * 0.15;
            ship.SetCurrentValue(id0, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id1, "structure") * 0.15;
            ship.SetCurrentValue(id1, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id2, "structure") * 0.15;
            ship.SetCurrentValue(id2, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id3, "structure") * 0.15;
            ship.SetCurrentValue(id3, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id4, "structure") * 0.15;
            ship.SetCurrentValue(id4, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id5, "structure") * 0.15;
            ship.SetCurrentValue(id5, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id6, "structure") * 0.15;
            ship.SetCurrentValue(id6, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id7, "structure") * 0.15;
            ship.SetCurrentValue(id7, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id8, "structure") * 0.15;
            ship.SetCurrentValue(id8, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id9, "structure") * 0.15;
            ship.SetCurrentValue(id9, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id10, "structure") * 0.15;
            ship.SetCurrentValue(id10, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id11, "structure") * 0.15;
            ship.SetCurrentValue(id11, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id12, "structure") * 0.15;
            ship.SetCurrentValue(id12, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id13, "structure") * 0.15;
            ship.SetCurrentValue(id13, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id14, "structure") * 0.15;
            ship.SetCurrentValue(id14, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id15, "structure") * 0.15;
            ship.SetCurrentValue(id15, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id16, "structure") * 0.15;
            ship.SetCurrentValue(id16, "structure", turretHP);*/


            storage.SetGlobal("system_" + sys_id + "_base_" + base_id + "_turrets", arrayOfTurretsAtStation);

        }
        else if (base_xml.substring(0, 8) == "business") {
            arrayOfTurretsAtStation = [];
            //OUTTER TURRET RIGHT
            //OUTTER TURRET RIGHT
            offsetX = 18;
            offsetY = 18;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 9;
            offsetY = 9;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id0 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id0, 0);
            relations.SetShipFaction(id0, faction);

            arrayOfTurretsAtStation.push({ loc: 0, id: id0, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });


            offsetX = 18;
            offsetY = 18;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = -11;
            offsetY = -11;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id1 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id1, 0);
            relations.SetShipFaction(id1, faction);
            //OUTTER TURRET RIGHT
            //OUTTER TURRET RIGHT
            arrayOfTurretsAtStation.push({ loc: 1, id: id1, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });

            //OUTTER TURRET LEFT
            //OUTTER TURRET LEFT
            offsetX = 4;
            offsetY = 4;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 9;
            offsetY = 9;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id2 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id2, 0);
            relations.SetShipFaction(id2, faction);
            arrayOfTurretsAtStation.push({ loc: 2, id: id2, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });

            offsetX = 4;
            offsetY = 4;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = -11;
            offsetY = -11;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id3 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id3, 0);
            relations.SetShipFaction(id3, faction);
            //OUTTER TURRET LEFT
            //OUTTER TURRET LEFT
            arrayOfTurretsAtStation.push({ loc: 3, id: id3, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });

            //INNER TURRET RIGHT
            //INNER TURRET RIGHT
            offsetX = 14;
            offsetY = 14;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));  //y
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));  //y

            offsetX = 5;
            offsetY = 5;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1)); //x
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1)); //x

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id4 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id4, 0);
            relations.SetShipFaction(id4, faction);
            arrayOfTurretsAtStation.push({ loc: 4, id: id4, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });


            offsetX = 14;
            offsetY = 14;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = -7;
            offsetY = -7;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id5 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id5, 0);
            relations.SetShipFaction(id5, faction);
            //INNER TURRET RIGHT
            //INNER TURRET RIGHT
            arrayOfTurretsAtStation.push({ loc: 5, id: id5, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });

            //INNER TURRET LEFT
            //INNER TURRET LEFT
            offsetX = 8;
            offsetY = 8;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 5;
            offsetY = 5;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id6 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id6, 0);
            relations.SetShipFaction(id6, faction);
            arrayOfTurretsAtStation.push({ loc: 6, id: id6, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });

            offsetX = 8;
            offsetY = 8;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = -7;
            offsetY = -7;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id7 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id7, 0);
            relations.SetShipFaction(id7, faction);
            //INNER TURRET LEFT
            //INNER TURRET LEFT
            arrayOfTurretsAtStation.push({ loc: 7, id: id7, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });




            /*var turretHP = ship.GetCurrentValue(id0, "structure") * 0.15;
            ship.SetCurrentValue(id0, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id1, "structure") * 0.15;
            ship.SetCurrentValue(id1, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id2, "structure") * 0.15;
            ship.SetCurrentValue(id2, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id3, "structure") * 0.15;
            ship.SetCurrentValue(id3, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id4, "structure") * 0.15;
            ship.SetCurrentValue(id4, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id5, "structure") * 0.15;
            ship.SetCurrentValue(id5, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id6, "structure") * 0.15;
            ship.SetCurrentValue(id6, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id7, "structure") * 0.15;
            ship.SetCurrentValue(id7, "structure", turretHP);*/


            storage.SetGlobal("system_" + sys_id + "_base_" + base_id + "_turrets", arrayOfTurretsAtStation);
        }
        else if (base_xml.substring(0, 7) == "science") {
            arrayOfTurretsAtStation = [];
            //OUTER TURRET RIGHT
            //OUTER TURRET RIGHT
            var offsetX = -12;
            var offsetY = -12;
            var pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            var pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 9;
            offsetY = 9;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            var turretformationWaypoint = { x: pointRightX, y: pointRightY };
            var turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //var turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id0 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id0, 0);
            relations.SetShipFaction(id0, faction);


            arrayOfTurretsAtStation.push({ loc: 0, id: id0, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });

            offsetX = -6;
            offsetY = -6;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = 9;
            offsetY = 9;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id1 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id1, 0);
            relations.SetShipFaction(id1, faction);
            arrayOfTurretsAtStation.push({ loc: 1, id: id1, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });

            offsetX = -9;
            offsetY = -9;
            pointRightX = coords.x + (rightDirPlayer.x * (offsetX + 1));
            pointRightY = coords.y + (rightDirPlayer.y * (offsetY + 1));

            offsetX = -10;
            offsetY = -10;
            pointRightX = pointRightX + (frontDirPlayer.x * (offsetX + 1));
            pointRightY = pointRightY + (frontDirPlayer.y * (offsetY + 1));

            turretformationWaypoint = { x: pointRightX, y: pointRightY };
            turret_coord = { x: turretformationWaypoint.x, y: turretformationWaypoint.y };
            //turret_coord = RotatePoint(pointToRotate, coords, parsedAngle);

            var id2 = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            ship.SetRotation(id2, 0);
            relations.SetShipFaction(id2, faction);

            //OUTER TURRET RIGHT
            //OUTER TURRET RIGHT
            arrayOfTurretsAtStation.push({ loc: 2, id: id2, sys_id: args.sys_id, coordsX: turret_coord.x, coordsY: turret_coord.y, baseID: base_id, spawned: 1, xmltype: xml_ship });



            /*var turretHP = ship.GetCurrentValue(id0, "structure") * 0.15;
            ship.SetCurrentValue(id0, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id1, "structure") * 0.15;
            ship.SetCurrentValue(id1, "structure", turretHP);
            turretHP = ship.GetCurrentValue(id2, "structure") * 0.15;
            ship.SetCurrentValue(id2, "structure", turretHP);*/





            storage.SetGlobal("system_" + sys_id + "_base_" + base_id + "_turrets", arrayOfTurretsAtStation);

        }
        else // if (base_xml.substring(0, 7) == "special")
        {
            var angstep = Math.PI * 2 / quantity;
            var origoffset = MathExt.RandRangeDouble(0, Math.PI);
            for (var ang = origoffset; ang < Math.PI * 2 + origoffset; ang += angstep) {
                var turret_coord =
                {
                    x: coords.x + turret_radius * Math.cos(ang),
                    y: coords.y + turret_radius * Math.sin(ang)
                };

                var id = generator.AddNPCShipToSystem(
                    "", ai_behavior, level, xml_ship,
                    args.sys_id,
                    turret_coord.x,
                    turret_coord.y,
                    tags);

                // avoid random rotation
                ship.SetRotation(id, 0);

                relations.SetShipFaction(id, faction);
            }
        }
    },
    SpawnJumpgateTurretsOfType: function (args, coords, quantity, level, turret_radius, ai_behavior, xml_ship, faction, tags, origoffset) {
        var angstep = Math.PI * 2 / quantity;

        var orientationAngle = origoffset;
        origoffset += Math.PI / 2;

        for (var ang = origoffset; ang < Math.PI * 2 + origoffset; ang += angstep) {
            var turret_coord = {
                x: coords.x + turret_radius * Math.cos(ang),
                y: coords.y + turret_radius * Math.sin(ang)
            };

            var id = generator.AddNPCShipToSystem(
                "", ai_behavior, level, xml_ship,
                args.sys_id,
                turret_coord.x,
                turret_coord.y,
                tags);

            // avoid random rotation
            ship.SetRotation(id, orientationAngle);

            relations.SetShipFaction(id, faction);
        }
    },
    /*
     ====================================================
     Spawns station npcs based on (in order of importance):
     1 - preset
     2 - faction
     3 - base tags
     ====================================================
     */
    SpawnStationPopulation: function (args, base_id, preset) {
        var bases = args.bases;
        var system_id = args.sys_id;
        var inf = args.system_info;

        var chosenBase;
        for (var k = 0; k < bases.length; k++) {
            if (bases[k].id == base_id) {
                chosenBase = bases[k];
            }
        }
        if (typeof (chosenBase) == "undefined") {
            return;
        }

        var tagClass = game.GetTag(base_id, "class");


        //GALAXY MARKET MOD//
        //////////////SPAWN MARKET TERMINAL IN EACH STATIONS///////////
        this.SpawnMarketTerminals(chosenBase, system_id, inf);
        //GALAXY MARKET MOD//


        // common part
        this.SpawnCommonNpc(chosenBase, system_id, inf);
        this.SpawnSpecialNpc(chosenBase, system_id, inf);


        //DRONE STATION MANAGER MOD//
        this.SpawnStationDroneManager(chosenBase, system_id, inf);


        /*//var inf = generator.GetSystemByID(system_id);
        if (inf.name == "Earth")
        {
            this.SpawnTerrainManager(chosenBase, system_id, inf);
        }*/



        //process unique presets
        if (preset == "StartingSystem") // Solar System
        {

            //GALAXY MARKET MOD//
            //////////////SPAWN MARKET BUY INSTANCES IN STATIONS//////////////
            this.SpawnMarketTerminalsBUYTypeHulls(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsBUYTypeEngines(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsBUYTypeFuel(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsBUYTypeGenerators(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsBUYTypeGrapplers(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsBUYTypeRadars(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsBUYTypeRcs(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsBUYTypeShields(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsBUYTypeBoosters(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsBUYTypeWeapons(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsBUYTypeDevices(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsBUYTypeConsumables(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsBUYTypeComponents(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsBUYTypeGoods(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsBUYTypeOres(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsBUYStations(chosenBase, system_id, inf);
            //////////////SPAWN MARKET SELL INSTANCES IN STATIONS//////////////
            this.SpawnMarketTerminalsSELLTypeHulls(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsSELLTypeEngines(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsSELLTypeFuel(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsSELLTypeGenerators(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsSELLTypeGrapplers(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsSELLTypeRadars(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsSELLTypeRcs(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsSELLTypeShields(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsSELLTypeBoosters(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsSELLTypeWeapons(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsSELLTypeDevices(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsSELLTypeConsumables(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsSELLTypeComponents(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsSELLTypeGoods(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsSELLTypeOres(chosenBase, system_id, inf);
            this.SpawnMarketTerminalsSELLStations(chosenBase, system_id, inf);
            //GALAXY MARKET MOD//

            //GALAXY MARKET STATION ITEM GEN MOD//
            //this.SpawnGalaxyMarketStationItemSpawner(chosenBase, system_id, inf);
            //GALAXY MARKET STATION ITEM GEN MOD//

            //DRONE MANAGER MOD//
            this.SpawnDroneManager(chosenBase, system_id, inf);
            this.SpawnDroneManagerCombat(chosenBase, system_id, inf);
            this.SpawnDroneManagerMining(chosenBase, system_id, inf);
            this.SpawnDroneManagerRepair(chosenBase, system_id, inf);
            //DRONE MANAGER MOD//


            this.SpawnStationSpawner(chosenBase, system_id, inf);


            //this.SpawnStationTest(chosenBase, system_id, inf);    
            //this.SpawnTerrainManager(chosenBase, system_id, inf);
            //this.SpawnGalaxyMerchantManager(chosenBase, system_id, inf);

            //i have moved this to corefunctionality.js when any new player comes into the server, they now will have
            //a coms device generated automatically specific to them. I probably will move tons of other stuff from 
            //other scripts there too in the future.

            //this.SpawnCommsManager(chosenBase, system_id, inf);


            this.SpawnStartingSystemPopulation(chosenBase, system_id, inf);
        }
        else {
            //generic presets
            if (inf.faction == "order") {
                if (tagClass == "mining") {
                    this.SpawnOrderMiningPopulation(chosenBase, system_id, inf);
                }
                else if (tagClass == "outpost") {
                    this.SpawnOrderOutpostPopulation(chosenBase, system_id, inf);
                }
                else if (tagClass == "science") {
                    this.SpawnOrderSciencePopulation(chosenBase, system_id, inf);
                }
                else if (tagClass == "business") {
                    this.SpawnOrderBusinessPopulation(chosenBase, system_id, inf);
                }
                else if (tagClass == "military") {
                    this.SpawnOrderMilitaryPopulation(chosenBase, system_id, inf);
                }
            }
            else if (inf.faction == "freedom") {


                if (tagClass == "mining") {
                    this.SpawnFreedomMiningPopulation(chosenBase, system_id, inf);
                }
                else if (tagClass == "outpost") {
                    this.SpawnFreedomOutpostPopulation(chosenBase, system_id, inf);
                }
                else if (tagClass == "science") {
                    this.SpawnFreedomSciencePopulation(chosenBase, system_id, inf);
                }
                else if (tagClass == "military") {
                    this.SpawnFreedomMilitaryPopulation(chosenBase, system_id, inf);
                }
                else if (tagClass == "business") {
                    this.SpawnFreedomBusinessPopulation(chosenBase, system_id, inf);
                }
            }
            else if (inf.faction == "fanatics") {
                if (tagClass == "mining") {
                    this.SpawnFanaticsMiningPopulation(chosenBase, system_id, inf);
                }
                else if (tagClass == "outpost") {
                    this.SpawnFanaticsOutpostPopulation(chosenBase, system_id, inf);
                }
                else if (tagClass == "science") {
                    this.SpawnFanaticsSciencePopulation(chosenBase, system_id, inf);
                }
                else if (tagClass == "military") {
                    this.SpawnFanaticsMilitaryPopulation(chosenBase, system_id, inf);
                }
                //no trade for fanatics...
            }
            else // neutral
            {
                if (tagClass == "mining") {
                    this.SpawnNeutralMiningPopulation(chosenBase, system_id, inf);
                }
                else if (tagClass == "outpost") {
                    this.SpawnNeutralOutpostPopulation(chosenBase, system_id, inf);
                }
                else if (tagClass == "business") {
                    this.SpawnNeutralBusinessPopulation(chosenBase, system_id, inf);
                }
                else if (tagClass == "science") {
                    this.SpawnNeutralSciencePopulation(chosenBase, system_id, inf);
                }
                // no military
            }
        }
    },

    /*
     Spawns NPCs, which are common for every base
     */

    SpawnCommonNpc: function (chosenBase, sys_id, inf) {
        var id;

        // Station commander. Every station has it's own commander.
        id = generator.AddNPCShipToSystem($i0005, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "station.commander", meta: "human", outfit: "officer" }); // Station Commander
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");

        // Crafting Terminal
        var base = generator.GetBaseByID(chosenBase.id);
        if (base.tech_level > 1) //not first system
        {
            if (base.xml_type_id == "outpost_01" || base.xml_type_id == "mining_01") //only outpost and mining stations
            {
                id = generator.AddNPCShipToSystem($i0060, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "manufacturing.terminal", greeting: "terminal" }); // Manufacturing Terminal
                relations.SetShipFaction(id, inf.faction);
                generator.DockShipToBase(id, chosenBase.id);
                ship.SetCurrentArea(id, "offices");
                generator.SetNPCAvatarImg(id, "avatars/unique/terminal_02.png");
            }
        }
    },



    /////////////////////////////////////////////////////GALAXY MARKET///////////////////////////////////////////////////////
    SpawnMarketTerminals: function (chosenBase, sys_id, inf) {
        var id;
        id = generator.AddNPCShipToSystem("Station Terminal", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "StationTerminal", greeting: "terminal", unique_id: "market" + chosenBase.id });
        //id = generator.AddNPCShipToSystem("Station Terminal", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "StationTerminal", greeting: "terminal" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");

    },

    /////////////////////////////////////////////////////GALAXY MARKET BUY///////////////////////////////////////////////////////
    SpawnMarketTerminalsBUYStations: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketBUYStations", greeting: "terminal", unique_id: "marketBUYStations" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsBUYTypeHulls: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketBUYHulls", greeting: "terminal", unique_id: "marketBUYHulls" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsBUYTypeEngines: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketBUYEngines", greeting: "terminal", unique_id: "marketBUYEngines" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsBUYTypeFuel: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketBUYFuel", greeting: "terminal", unique_id: "marketBUYFuel" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsBUYTypeGenerators: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketBUYGenerators", greeting: "terminal", unique_id: "marketBUYGenerators" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsBUYTypeGrapplers: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketBUYGrapplers", greeting: "terminal", unique_id: "marketBUYGrapplers" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsBUYTypeRadars: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketBUYRadars", greeting: "terminal", unique_id: "marketBUYRadars" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsBUYTypeRcs: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketBUYRcs", greeting: "terminal", unique_id: "marketBUYRcs" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsBUYTypeShields: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketBUYShields", greeting: "terminal", unique_id: "marketBUYShields" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsBUYTypeBoosters: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketBUYBoosters", greeting: "terminal", unique_id: "marketBUYBoosters" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },


    SpawnMarketTerminalsBUYTypeWeapons: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketBUYWeapons", greeting: "terminal", unique_id: "marketBUYWeapons" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsBUYTypeDevices: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketBUYDevices", greeting: "terminal", unique_id: "marketBUYDevices" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },


    SpawnMarketTerminalsBUYTypeConsumables: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketBUYConsumables", greeting: "terminal", unique_id: "marketBUYConsumables" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsBUYTypeComponents: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketBUYComponents", greeting: "terminal", unique_id: "marketBUYComponents" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsBUYTypeGoods: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketBUYGoods", greeting: "terminal", unique_id: "marketBUYGoods" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsBUYTypeOres: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketBUYOres", greeting: "terminal", unique_id: "marketBUYOres" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },


    /////////////////////////////////////////////////////GALAXY MARKET SELL///////////////////////////////////////////////////////
    SpawnMarketTerminalsSELLStations: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketSELLStations", greeting: "terminal", unique_id: "marketSELLStations" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsSELLTypeHulls: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketSELLHulls", greeting: "terminal", unique_id: "marketSELLHulls" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsSELLTypeEngines: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketSELLEngines", greeting: "terminal", unique_id: "marketSELLEngines" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsSELLTypeFuel: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketSELLFuel", greeting: "terminal", unique_id: "marketSELLFuel" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsSELLTypeGenerators: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketSELLGenerators", greeting: "terminal", unique_id: "marketSELLGenerators" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsSELLTypeGrapplers: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketSELLGrapplers", greeting: "terminal", unique_id: "marketSELLGrapplers" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsSELLTypeRadars: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketSELLRadars", greeting: "terminal", unique_id: "marketSELLRadars" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsSELLTypeRcs: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketSELLRcs", greeting: "terminal", unique_id: "marketSELLRcs" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsSELLTypeShields: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketSELLShields", greeting: "terminal", unique_id: "marketSELLShields" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsSELLTypeBoosters: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketSELLBoosters", greeting: "terminal", unique_id: "marketSELLBoosters" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },


    SpawnMarketTerminalsSELLTypeWeapons: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketSELLWeapons", greeting: "terminal", unique_id: "marketSELLWeapons" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsSELLTypeDevices: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketSELLDevices", greeting: "terminal", unique_id: "marketSELLDevices" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },


    SpawnMarketTerminalsSELLTypeConsumables: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketSELLConsumables", greeting: "terminal", unique_id: "marketSELLConsumables" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsSELLTypeComponents: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketSELLComponents", greeting: "terminal", unique_id: "marketSELLComponents" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsSELLTypeGoods: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketSELLGoods", greeting: "terminal", unique_id: "marketSELLGoods" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    SpawnMarketTerminalsSELLTypeOres: function (chosenBase, sys_id, inf) {
        var id;
        var basesInfo = generator.GetBaseByID(chosenBase.id);
        id = generator.AddNPCShipToSystem("market", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "marketSELLOres", greeting: "terminal", unique_id: "marketSELLOres" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },

    /*/////////////////////////////////////////////////////SPAWN ITEMS EVERYWHERE IN GALAXY///////////////////////////////////////////////////////
    SpawnGalaxyMarketStationItemSpawner: function (chosenBase, sys_id, inf) {
        var id;
        id = generator.AddNPCShipToSystem("GalaxyMarketStationItemSpawner", "galaxyMarketSpawner", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "GalaxyMarketStationItemSpawner", greeting: "terminal", unique_id: "GalaxyMarketStationItemSpawner" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputer.png");
        npc.SetHiddenInStation(id, true);
    },
    /////////////////////////////////////////////////////GALAXY MARKET END///////////////////////////////////////////////////////
    */


    /////////////////////////////////////////////////////MAIN DRONE MANAGER///////////////////////////////////////////////////////
    SpawnDroneManager: function (chosenBase, sys_id, inf) {
        var id;
        id = generator.AddNPCShipToSystem("OnBoard Computer", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "OnBoardComputer", greeting: "terminal", unique_id: "OnBoardComputer" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputerDialog.png");
        npc.SetHiddenInStation(id, true);
    },

    /////////////////////////////////////////////////////COMBAT DRONE///////////////////////////////////////////////////////
    SpawnDroneManagerCombat: function (chosenBase, sys_id, inf) {
        var id;
        id = generator.AddNPCShipToSystem("drone Manager Combat", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "droneManagerCombat", greeting: "terminal", unique_id: "droneManagerCombat" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputerDialog.png");
        npc.SetHiddenInStation(id, true);
    },

    /////////////////////////////////////////////////////MINING DRONE///////////////////////////////////////////////////////
    SpawnDroneManagerMining: function (chosenBase, sys_id, inf) {
        var id;
        id = generator.AddNPCShipToSystem("drone Manager Mining", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "droneManagerMining", greeting: "terminal", unique_id: "droneManagerMining" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputerDialog.png");
        npc.SetHiddenInStation(id, true);
    },

    /////////////////////////////////////////////////////REPAIR DRONE///////////////////////////////////////////////////////
    SpawnDroneManagerRepair: function (chosenBase, sys_id, inf) {
        var id;
        id = generator.AddNPCShipToSystem("drone Manager Repair", "BaseSitter", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "droneManagerRepair", greeting: "terminal", unique_id: "droneManagerRepair" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputerDialog.png");
        npc.SetHiddenInStation(id, true);
    },

    /////////////////////////////////////////////////////REPAIR DRONE///////////////////////////////////////////////////////
    /*SpawnCommsManager: function (chosenBase, sys_id, inf) {
        var id;
        id = generator.AddNPCShipToSystem("Comms Manager ", "sc_station_comms_systems", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "CommsManager", greeting: "terminal", unique_id: "CommsManager" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputerDialog.png");
        npc.SetHiddenInStation(id, true);
    },*/





    //NOT YET IMPLEMENTED
    SpawnTerrainManager: function (chosenBase, sys_id, inf) {
        var id;
        id = generator.AddNPCShipToSystem("terrain Manager", "terrainGenerator", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "terrainManager", greeting: "terminal", unique_id: "terrainManager" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/terminal_01.png");
        npc.SetHiddenInStation(id, true);
    },




    //NOT YET IMPLEMENTED
    SpawnStationTest: function (chosenBase, sys_id, inf, coordsX, coordsY) {
        var id;
        id = generator.AddNPCShipToSystem("station test", "BaseSitter", 5, "xml_station_test", sys_id, coordsX, coordsY, { class: "stationTest", greeting: "terminal", unique_id: "stationTest" });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/terminal_01.png");
        npc.SetHiddenInStation(id, true);
    },




    //NOT YET IMPLEMENTED
    SpawnStationSpawner: function (chosenBase, sys_id, inf) {

        //sc_gen_stations


        //storage.SetGlobal("stationEXT" + sys_id1, { id: null, x: coord_x, y: coord_y, sys_id: sys_id0, xml_id: arrayOfOutpostStations[randomIndexStationOutpost], DegAngle: parsedAngle, class: tagClass });

        var id;

        id = generator.AddNPCShipToSystem("stationINT", "ai_station", 1, "xml_station_interior", sys_id, chosenBase.coord.x + 20, chosenBase.coord.y, { class: "stationINT", someTag: "stationINT", greeting: "terminal", stationID: "stationINT" }); //, unique_id: "stationDialog"
        //id = generator.AddNPCShipToSystem("station test", "ai_station", 100, "xml_station_exterior", sys_id, chosenBase.coord.x + 20, chosenBase.coord.y, { someTag: "stationEXT", class: "stationEXT", greeting: "terminal", unique_id: "stationTest", stationID: "player base" });
        //id = generator.AddNPCShipToSystem("terrain Manager", "ai_station", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "stationSpawner", greeting: "terminal", unique_id: "stationSpawner" });

        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/terminal_01.png");
        npc.SetHiddenInStation(id, true);
    },





    /////////////////////////////////////////////////////STATION DRONE MANAGER/////////////////////////////////////////////////////// FOR PATHFIND INSIDE STATIONS RADIUS
    SpawnStationDroneManager: function (chosenBase, sys_id, inf) {

        var indexOfStationDroneManager = -1;
        if (storage.IsSetGlobal("StationDroneManagerIndex")) {
            indexOfStationDroneManager = storage.GetGlobal("StationDroneManagerIndex");
            indexOfStationDroneManager++;
            storage.SetGlobal("StationDroneManagerIndex", indexOfStationDroneManager);

        }
        else {
            indexOfStationDroneManager = 0;
            storage.SetGlobal("StationDroneManagerIndex", indexOfStationDroneManager);
        }

        var id;
        id = generator.AddNPCShipToSystem("Station Drone Manager", "SC_Drone_Station_Manager", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "stationDroneManager", greeting: "terminal", unique_id: "stationDroneManager", stationDroneIndex: indexOfStationDroneManager });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputerDialog.png");
        npc.SetHiddenInStation(id, true);
        /*var indexOfStationDroneManager = -1;


        if (!storage.IsSetGlobal("StationDroneManagerIndex")) {
            indexOfStationDroneManager = 0;
            storage.SetGlobal("StationDroneManagerIndex", indexOfStationDroneManager);
            storage.SetGlobal("StationDroneManagerIndex_" + chosenBase.id, indexOfStationDroneManager);
        }
        else {
            indexOfStationDroneManager = storage.GetGlobal("StationDroneManagerIndex");
            indexOfStationDroneManager++;
            storage.SetGlobal("StationDroneManagerIndex", indexOfStationDroneManager);
            storage.SetGlobal("StationDroneManagerIndex_" + chosenBase.id, indexOfStationDroneManager);
        }*/



        /*if (storage.IsSetGlobal("StationDroneManagerIndex" + chosenBase))
        {
            var someGlobalIndex = storage.GetGlobal("StationDroneManagerIndex");
            someGlobalIndex++;
            storage.SetGlobal("GlobalIndex_Player_" + args.name, someGlobalIndex);
            storage.SetGlobal("GlobalIndex_Player", someGlobalIndex);
        }
        else
        {
            var someGlobalIndex = 0;
            storage.SetGlobal("GlobalIndex_Player_" + args.name, someGlobalIndex);
            storage.SetGlobal("GlobalIndex_Player", someGlobalIndex);
        }*/



        /*if (storage.IsSetGlobal("StationDroneManagerIndex"))
        {
            indexOfStationDroneManager = storage.GetGlobal("StationDroneManagerIndex");
            indexOfStationDroneManager++;
            storage.SetGlobal("StationDroneManagerIndex", indexOfStationDroneManager);

        }
        else
        {
            indexOfStationDroneManager = 0;
            storage.SetGlobal("StationDroneManagerIndex", indexOfStationDroneManager);
        }*/



        //later make this maybe the station commander with the ability to leave the station and/or be replaced by another member of the crew if the commander leaves and is destroyed by amy player ships.
        //special_station 
        /*var id;
        id = generator.AddNPCShipToSystem("Station Drone Manager", "SCDroneStationManager", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "stationDroneManager", greeting: "terminal", unique_id: "stationDroneManager", stationDroneIndex: indexOfStationDroneManager });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputerDialog.png");
        npc.SetHiddenInStation(id, true);
        */




        /*var id;
        id = generator.AddNPCShipToSystem("Station Drone Manager", "SC_Drone_Station_Manager", 5, "special_no_ship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "stationDroneManager", greeting: "terminal", unique_id: "stationDroneManager", stationDroneIndex: indexOfStationDroneManager });
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/OnBoardComputerDialog.png");
        npc.SetHiddenInStation(id, false);*/


    },


















    SpawnSpecialNpc: function (chosenBase, sys_id, inf) {
        var id;

        //some extra neutral npcs
        if (inf.faction == "none") {
            var llns = storage.Get("_system_generation", "neutral_stations_special");
            if (llns == null) {
                // generic station
                return;
            }

            if (llns.length == 1 && llns[llns.length - 1] == chosenBase.id) {
                //1st station - farmer station
                id = generator.AddNPCShipToSystem($i0006, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "farmer", unique_id: "farmer.boris", meta: "human", sex: "male", outfit: "casual" }); // Boris
                relations.SetShipFaction(id, inf.faction);
                generator.DockShipToBase(id, chosenBase.id);
                ship.SetCurrentArea(id, "offices");

                storage.Set("_system_special", "farmer.boris", { system: sys_id, base: chosenBase.id });
            }
            else if (llns.length == 2 && llns[llns.length - 1] == chosenBase.id) {
                //2nd station - pirate blockpost station
                id = generator.AddNPCShipToSystem($i0007, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "pirate", unique_id: "pirate.garcia", meta: "human", sex: "male", outfit: "military" }); // Garcia
                relations.SetShipFaction(id, "pirates");
                generator.DockShipToBase(id, chosenBase.id);
                ship.SetCurrentArea(id, "offices");

                storage.Set("_system_special", "pirate.garcia", { system: sys_id, base: chosenBase.id });
            }
            else if (llns.length == 3 && llns[llns.length - 1] == chosenBase.id) {
                //3rd station - barrich the scientist
                id = generator.AddNPCShipToSystem($i0008, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "scientist", unique_id: "barrich", meta: "human", sex: "male" }); // Barrich
                relations.SetShipFaction(id, "pirates");
                generator.DockShipToBase(id, chosenBase.id);
                ship.SetCurrentArea(id, "offices");
                generator.SetNPCAvatarImg(id, "avatars/unique/barrich.png");

                storage.Set("_system_special", "barrich", { system: sys_id, base: chosenBase.id });
            }
        }
        else if (inf.faction == "order") {
            //computer terminal
            id = generator.AddNPCShipToSystem($i0009, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "order.terminal", greeting: "terminal" }); // Public Terminal
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
            generator.SetNPCAvatarImg(id, "avatars/unique/terminal_01.png");
        }
    },
    /*
     ======================================================================
     S P A W N    S T A T I O N S    P O P U L A T I O N S
     based on station type and faction
     ======================================================================
     */

    /*
     ======================================================================
     U N I Q U E
     ======================================================================
     */

    SpawnStartingSystemPopulation: function (chosenBase, sys_id, inf) {
        var id;

        // one-eyed joe
        id = generator.AddNPCShipToSystem($i0010, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "farmer", unique_id: "oneeyedjoe", meta: "human", sex: "male" }); // One-eyed Joe
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
        generator.SetNPCAvatarImg(id, "avatars/unique/one-eyed-joe.png");

        // order recruiter
        id = generator.AddNPCShipToSystem($i0011, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "order.recruiter", unique_id: "order.recruiter", meta: "human", sex: "male", outfit: "military" }); // Order recruiter
        relations.SetShipFaction(id, "order");
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
    },
    /*
     ======================================================================
     N E U T R A L
     ======================================================================
     */
    SpawnNeutralMiningPopulation: function (chosenBase, sys_id, inf) {
        var id;

        // Mining foreman
        id = generator.AddNPCShipToSystem($i0012, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "mining.foreman", meta: "human", outfit: "casual" }); // Mining Foreman
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");

        if (inf.danger_level <= 40) {
            // Order recruiter
            id = generator.AddNPCShipToSystem($i0013, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "order.recruiter", unique_id: "order.recruiter", meta: "human", outfit: "military" }); // Order recruiter
            relations.SetShipFaction(id, "order");
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
        }
    },
    SpawnNeutralOutpostPopulation: function (chosenBase, sys_id, inf) {
        var id;

        // Order Recruiter
        if (inf.danger_level <= 40) {
            id = generator.AddNPCShipToSystem($i0014, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "order.recruiter", unique_id: "order.recruiter", meta: "human", outfit: "military" }); // Order recruiter
            relations.SetShipFaction(id, "order");
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
        }
    },
    SpawnNeutralSciencePopulation: function (chosenBase, sys_id, inf) {
        // Chief Scientist
        var id = generator.AddNPCShipToSystem($i0017, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "chief.scientist", meta: "human", outfit: "science" }); // Chief Scientist
        relations.SetShipFaction(id, "none");
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");
    },
    SpawnNeutralBusinessPopulation: function (chosenBase, sys_id, inf) {
        var id;

        // Broker
        id = generator.AddNPCShipToSystem($i0015, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "broker", meta: "human", outfit: "casual" }); // Broker
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");


        if (inf.danger_level <= 40) {
            // Order recruiter
            id = generator.AddNPCShipToSystem($i0016, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "order.recruiter", unique_id: "order.recruiter", meta: "human", outfit: "military" }); // Order recruiter
            relations.SetShipFaction(id, "order");
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
        }
    },
    /*
     ======================================================================
     O R D E R
     ======================================================================
     */

    SpawnOrderSciencePopulation: function (chosenBase, sys_id, inf) {
        var id;

        // Chief Scientist
        id = generator.AddNPCShipToSystem($i0017, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "chief.scientist", meta: "human", outfit: "science" }); // Chief Scientist
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");

        // if capital
        if (relations.IsCapital(sys_id)) {
            // Officer Threepwood
            id = generator.AddNPCShipToSystem($i0018, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "order.officer", unique_id: "officer.threepwood", meta: "human", sex: "male", outfit: "military" }); // Officer Threepwood
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");

            // Scientist Rosco
            id = generator.AddNPCShipToSystem($i0019, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "order.scientist", unique_id: "scientist.rosco", meta: "human", sex: "male", outfit: "science" }); // Scientist Rosco
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
        }
        else {
            this.SpawnOrderSpecialNpcs(chosenBase, sys_id, inf);
        }
    },
    SpawnOrderMiningPopulation: function (chosenBase, sys_id, inf) {
        var id;

        // Mining foreman
        id = generator.AddNPCShipToSystem($i0020, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "mining.foreman", meta: "human", outfit: "casual" }); // Mining Foreman
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");

        // If capital
        if (relations.IsCapital(sys_id)) {
            //no special npcs
        }
        else {
            this.SpawnOrderSpecialNpcs(chosenBase, sys_id, inf);
        }
    },
    SpawnOrderOutpostPopulation: function (chosenBase, sys_id, inf) {
        var id;
        if (relations.IsCapital(sys_id)) {
            // officer Whistler - in the Capital
            id = generator.AddNPCShipToSystem($i0021, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "order.officer", unique_id: "officer.whistler", meta: "human", sex: "male", outfit: "military" }); // Officer Whistler
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");


            // Reporter Nico Davidson - in the Capital
            id = generator.AddNPCShipToSystem($i0022, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "reporter", unique_id: "nico.davidson", meta: "human", sex: "male", outfit: "casual" }); // Nico Davidson
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
        }
        else {
            this.SpawnOrderSpecialNpcs(chosenBase, sys_id, inf);
        }
    },
    SpawnOrderMilitaryPopulation: function (chosenBase, sys_id, inf) {
        var id;

        // Security officer
        id = generator.AddNPCShipToSystem($i0023, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "security.officer", meta: "human", sex: "male", outfit: "military" }); // Security Officer
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");


        //  C A P I T A L  only
        //-------------------------------------------------------------------
        if (relations.IsCapital(sys_id)) {
            var ids = [];

            // Order leader - General troyden
            id = generator.AddNPCShipToSystem($i0024, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "order.leader", unique_id: "troyden", meta: "", sex: "male" }); // General Troyden
            relations.SetShipFaction(id, inf.faction);
            generator.SetNPCAvatarImg(id, "avatars/unique/order_general_troyden.png");
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "command_deck_order");
            ids.push(id);

            // Sgt. Ragnar
            id = generator.AddNPCShipToSystem($i0025, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "order.officer", unique_id: "sgt.ragnar", meta: "human", sex: "male" }); // Sgt. Ragnar
            relations.SetShipFaction(id, inf.faction);
            generator.SetNPCAvatarImg(id, "avatars/unique/order_sgt_ragnar.png");
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
            ids.push(id);

            storage.Set("_system_special", "sgt.ragnar", { system: sys_id, base: chosenBase.id });

            // Officer Coldridge
            id = generator.AddNPCShipToSystem($i0026, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "order.officer", unique_id: "officer.coldridge", meta: "human", sex: "male", outfit: "military" }); // Officer Coldridge
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
            ids.push(id);

            storage.Set("_system_special", "officer.coldridge", { system: sys_id, base: chosenBase.id });

            //set additional variable for last_stand_quest
            storage.Set("_system_singleplayer", "order_leaders", ids);
        }
        else {
            this.SpawnOrderSpecialNpcs(chosenBase, sys_id, inf);
        }
    },
    SpawnOrderBusinessPopulation: function (chosenBase, sys_id, inf) {
        var id;

        // Broker
        id = generator.AddNPCShipToSystem($i0027, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "broker", meta: "human", outfit: "casual" }); // Broker
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");

        if (relations.IsCapital(sys_id)) {
            //no unique npcs
        }
        else {
            this.SpawnOrderSpecialNpcs(chosenBase, sys_id, inf);
        }
    },
    /*
     These npcs must be randomly distributed across non-capital systems of order
     */
    SpawnOrderSpecialNpcs: function (chosenBase, sys_id, inf) {
        var id;
        var spawned = storage.Get("_system_generation", "order_npcs");
        if (spawned == null) {
            spawned = { step: 0 };
        }

        if (spawned.step == 0) {
            // Officer Kolosov
            id = generator.AddNPCShipToSystem($i0028, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "order.officer", unique_id: "officer.kolosov", meta: "human", sex: "male", outfit: "military" }); // Officer Kolosov
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");

            // Brother Portridge
            id = generator.AddNPCShipToSystem($i0029, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "fanatics.brother", unique_id: "brother.portridge", sex: "male", outfit: "casual" }); // Brother Portridge
            relations.SetShipFaction(id, "none");
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");

            storage.Set("_system_special", "brother.portridge", { system: sys_id, base: chosenBase.id });
        }
        else if (spawned.step == 1) {
            // Officer Aims
            id = generator.AddNPCShipToSystem($i0030, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "order.officer", unique_id: "officer.aims", meta: "human", sex: "male", outfit: "military" }); // Officer Aims
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
        }
        else if (spawned.step == 2) {
            // Officer Gorth
            id = generator.AddNPCShipToSystem($i0031, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "order.officer", unique_id: "officer.gorth", meta: "human", sex: "male", outfit: "military" }); // Officer Gorth
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
        }

        spawned.step++;
        storage.Set("_system_generation", "order_npcs", spawned);
    },
    /*
     ======================================================================
     F R E E D O M
     ======================================================================
     */

    SpawnFreedomSciencePopulation: function (chosenBase, sys_id, inf) {
        var id;

        // Chief Engineer
        id = generator.AddNPCShipToSystem($i0032, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "chief.scientist", meta: "human", outfit: "science" }); // Chief Engineer
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");

        if (relations.IsCapital(sys_id)) {
            //no unique npcs
        }
        else {
            this.SpawnFreedomSpecialNpcs(chosenBase, sys_id, inf);
        }
    },
    SpawnFreedomMiningPopulation: function (chosenBase, sys_id, inf) {
        var id;

        // Mining Foreman
        id = generator.AddNPCShipToSystem($i0033, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "mining.foreman", meta: "human", outfit: "casual" }); // Mining Foreman
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");

        if (relations.IsCapital(sys_id)) {
            // Mercenary Roth
            id = generator.AddNPCShipToSystem($i0034, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "freedom.merc", unique_id: "mercenary.roth", meta: "human", sex: "male", outfit: "casual" }); // Mercenary Roth
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
        }
        else {
            this.SpawnFreedomSpecialNpcs(chosenBase, sys_id, inf);
        }
    },
    SpawnFreedomOutpostPopulation: function (chosenBase, sys_id, inf) {
        var id;

        if (relations.IsCapital(sys_id)) {
            // Merc Aims
            id = generator.AddNPCShipToSystem($i0035, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "freedom.merc", unique_id: "mercenary.aims", meta: "human", sex: "male", outfit: "casual" }); // Mercenary Aims
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
        }
        else {
            this.SpawnFreedomSpecialNpcs(chosenBase, sys_id, inf);
        }
    },
    SpawnFreedomMilitaryPopulation: function (chosenBase, sys_id, inf) {
        var id;

        // Mercenary Leader (security officer)
        id = generator.AddNPCShipToSystem($i0036, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "security.officer", meta: "human", sex: "male", outfit: "military" }); // Mercenary Leader
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");

        if (relations.IsCapital(sys_id)) {
            //no unique npcs
        }
        else {
            this.SpawnFreedomSpecialNpcs(chosenBase, sys_id, inf);
        }
    },
    SpawnFreedomBusinessPopulation: function (chosenBase, sys_id, inf) {
        var id;

        // Broker
        id = generator.AddNPCShipToSystem($i0037, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "broker", meta: "human", outfit: "casual" }); // Broker
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");


        //  C A P I T A L  only
        //---------------------------------------------------
        var capital = relations.GetFactionCapital("freedom");
        if (sys_id == capital) {
            // Order leader - Selek Jeredan
            id = generator.AddNPCShipToSystem($i0038, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "freedom.leader", unique_id: "jeredan", meta: "", sex: "male" }); // Selek Jeredan
            relations.SetShipFaction(id, inf.faction);
            generator.SetNPCAvatarImg(id, "avatars/unique/freedom_selek_jeredan.png");
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "command_deck_freedom");

            // Mercenary Headknot
            id = generator.AddNPCShipToSystem($i0039, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, {
                class: "freedom.merc", unique_id: "mercenary.headknot", meta: "human", sex: "male"
            }); // Mercenary Headknot
            relations.SetShipFaction(id, inf.faction);
            generator.SetNPCAvatarImg(id, "avatars/unique/freedom_headknot.png");
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
            storage.Set("_system_special", "mercenary.headknot", { system: sys_id, base: chosenBase.id });

            // Fence Garret
            id = generator.AddNPCShipToSystem($i0040, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "freedom.fence", unique_id: "fence.garret", meta: "human", sex: "male", outfit: "casual" }); // Fence Garret
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");

            // Fence Kobb
            id = generator.AddNPCShipToSystem($i0041, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "freedom.fence", unique_id: "fence.kobb", meta: "human", sex: "male", outfit: "casual" }); // Fence Kobb
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
        }
        else {
            this.SpawnFreedomSpecialNpcs(chosenBase, sys_id, inf);
        }
    },
    /*
     These npcs must be randomly distributed across non-capital systems of order
     */
    SpawnFreedomSpecialNpcs: function (chosenBase, sys_id, inf) {
        var id;

        var spawned = storage.Get("_system_generation", "freedom_npcs");
        if (spawned == null) {
            spawned = { step: 0 };
        }

        if (spawned.step == 0) {
            // Mechanic Sheldon
            id = generator.AddNPCShipToSystem($i0042, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "freedom.mechanic", unique_id: "mechanic.sheldon", meta: "human", sex: "male", outfit: "casual" }); // Mechanic Sheldon
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");

            // Broker Mannis
            id = generator.AddNPCShipToSystem($i0043, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "broker", unique_id: "broker.mannis", meta: "human", sex: "male", outfit: "casual" }); // Broker Mannis
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
            storage.Set("_system_special", "broker.mannis", { system: sys_id, base: chosenBase.id });
        }
        else if (spawned.step == 1) {
            // Broker Lansel
            id = generator.AddNPCShipToSystem($i0044, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "broker", unique_id: "broker.lansel", meta: "human", sex: "male", outfit: "casual" }); // Broker Lansel
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
        }
        else if (spawned.step == 2) {
            // Mercenary Ricco
            id = generator.AddNPCShipToSystem($i0045, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "freedom.merc", unique_id: "mercenary.ricco", meta: "human", sex: "male", outfit: "casual" }); // Mercenary Ricco
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");

            // Mercenary Batto
            id = generator.AddNPCShipToSystem($i0046, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "freedom.merc", unique_id: "mercenary.batto", meta: "human", sex: "male", outfit: "casual" }); // Mercenary Batto
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
        }
        else if (spawned.step == 3) {
            // Mercenary Thorn
            id = generator.AddNPCShipToSystem($i0047, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "freedom.merc", unique_id: "mercenary.thorn", meta: "human", sex: "male", outfit: "casual" }); // Mercenary Thorn
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");

            storage.Set("_system_special", "mercenary.thorn", { system: sys_id, base: chosenBase.id });
        }

        spawned.step++;
        storage.Set("_system_generation", "freedom_npcs", spawned);
    },
    /*
     ======================================================================
     F A N A T I C S
     ======================================================================
     */

    SpawnFanaticsSciencePopulation: function (chosenBase, sys_id, inf) {
        // Chief archaeologist
        var id = generator.AddNPCShipToSystem($i0048, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "chief.archaeologist", meta: "human", outfit: "science" }); // Chief Archaeologist

        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");

        //  C A P I T A L  only
        //---------------------------------------------------
        var capital = relations.GetFactionCapital("fanatics");
        if (sys_id == capital) {
            // Arthur Melak (fanatics leader)
            id = generator.AddNPCShipToSystem($i0049, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "fanatics.leader", unique_id: "melak", meta: "", sex: "male" }); // Arthur Melak
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "command_deck_fanatics");
            storage.Set("_system_special", "melak", { system: sys_id, base: chosenBase.id });
            generator.SetNPCAvatarImg(id, "avatars/unique/fanatics_arthur_melak.png");

            // Guardian Dostan
            id = generator.AddNPCShipToSystem($i0051, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "fanatics.guardian", unique_id: "guardian.dostan", meta: "human", sex: "male" }); // Guardian Dostan
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
            storage.Set("_system_special", "guardian.dostan", { system: sys_id, base: chosenBase.id });
            generator.SetNPCAvatarImg(id, "avatars/unique/fanatics_guardian_dostan.png");

            // Brother Jebodiah
            id = generator.AddNPCShipToSystem($i0050, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "fanatics.brother", unique_id: "brother.jebodiah", meta: "human", sex: "male", outfit: "casual" }); // Brother Jebodiah
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
        }
        else {
            this.SpawnFanaticsSpecialNpcs(chosenBase, sys_id, inf);
        }
    },
    SpawnFanaticsMiningPopulation: function (chosenBase, sys_id, inf) {
        var id;

        // Foreman
        id = generator.AddNPCShipToSystem($i0052, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "mining.foreman", meta: "human", outfit: "casual" }); // Mining Foreman
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");

        var capital = relations.GetFactionCapital("fanatics");
        if (sys_id == capital) {
            // Brother Derryl
            id = generator.AddNPCShipToSystem($i0053, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "fanatics.brother", unique_id: "brother.derryl", meta: "human", sex: "male", outfit: "casual" }); // Brother Derryl
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");

            //Brother Benedict
            id = generator.AddNPCShipToSystem($i0054, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "fanatics.brother", unique_id: "brother.benedict", meta: "human", sex: "male", outfit: "officer" }); // Brother Benedict
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");

            storage.Set("_system_special", "brother.benedict", { system: sys_id, base: chosenBase.id, meta: "human", sex: "male" });
        }
        else {
            this.SpawnFanaticsSpecialNpcs(chosenBase, sys_id, inf);
        }
    },
    SpawnFanaticsOutpostPopulation: function (chosenBase, sys_id, inf) {
        var id;
        var capital = relations.GetFactionCapital("fanatics");
        if (sys_id == capital) {
            // Brother Maverick
            id = generator.AddNPCShipToSystem($i0055, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "fanatics.brother", unique_id: "brother.maverick", meta: "human", sex: "male", outfit: "casual" }); // Brother Maverick
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");

            // Brother Danny
            id = generator.AddNPCShipToSystem($i0056, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "fanatics.brother", unique_id: "brother.danny", meta: "human", sex: "male", outfit: "casual" }); // Brother Danny
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");

            storage.Set("_system_special", "brother.danny", { system: sys_id, base: chosenBase.id, meta: "human", sex: "male" });
        }
        else {
            this.SpawnFanaticsSpecialNpcs(chosenBase, sys_id, inf);
        }
    },
    SpawnFanaticsMilitaryPopulation: function (chosenBase, sys_id, inf) {
        var id;

        // Cult Guardian (security officer)
        id = generator.AddNPCShipToSystem($i0057, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "security.officer", meta: "human", sex: "male", outfit: "military" }); // Cult Guardian
        relations.SetShipFaction(id, inf.faction);
        generator.DockShipToBase(id, chosenBase.id);
        ship.SetCurrentArea(id, "offices");

        var capital = relations.GetFactionCapital("fanatics");
        if (sys_id == capital) {
            //no unique npc
        }
        else {
            this.SpawnFanaticsSpecialNpcs(chosenBase, sys_id, inf);
        }
    },
    //no trade for fanatics	
    SpawnFanaticsSpecialNpcs: function (chosenBase, sys_id, inf) {
        var id;
        var spawned = storage.Get("_system_generation", "fanatics_npcs");
        if (spawned == null) {
            spawned = { step: 0 };
        }

        if (spawned.step == 0) {
            // Brother Luke
            id = generator.AddNPCShipToSystem($i0058, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "fanatics.brother", unique_id: "brother.luke", meta: "human", sex: "male", outfit: "casual" }); // Brother Luke
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
        }
        else if (spawned.step == 1) {
            // Brother Ramos
            id = generator.AddNPCShipToSystem($i0059, "BaseSitter", 5, "special_human_techship", sys_id, chosenBase.coord.x, chosenBase.coord.y, { class: "fanatics.brother", unique_id: "brother.ramos", meta: "human", sex: "male", outfit: "casual" }); // Brother Ramos
            relations.SetShipFaction(id, inf.faction);
            generator.DockShipToBase(id, chosenBase.id);
            ship.SetCurrentArea(id, "offices");
        }

        spawned.step++;
        storage.Set("_system_generation", "fanatics_npcs", spawned);
    },
    /*
     =================================================================================
     function spawns specified npcs in groups by types
     
     { types: [], max_in_group: [] }
     =================================================================================
     */
    SpawnNpcsStep: function (args, npcs_count, select_tags, faction, name, behaviors, tags) {

        /*var system_id = args.sys_id;
        var system_faction = relations.GetSystemFaction(system_id);
        var inf = args.system_info;
        var step = args.step;

        var bounds = CSGen.GetWorldBounds(args);
        //steps_count *= (bounds.x * bounds.y) / (350 * 350);

        var coords;
        var shipByRandom;
        var level;
        var groupsize;
        var id;
        var ang;
        var attempt = 0;

        //here we pick suitable npcs by their spawning info
        var dangerLevel = inf.danger_level;
        var possibleNpcs = generator.GetNpcsForLevelByTags(select_tags, dangerLevel);
        if (possibleNpcs.length == 0) {
            console.PrintError("Cannot popupate system with NPCs - no suitable NPCs for provided tags and danger_level=" + dangerLevel);
            // cycle is over
            return -1;
        }

        var max_generations_per_iteration = 5;
        var npcs_generated_during_iteration = 0;

        for (; step < npcs_count;) {
            // next step
            //console.PrintError("NPC Gen current step: " + step + " steps count: " + steps_count);

            // try to find suitable coords
            do {
                coords = CSGen.GetRandomCoordWithinWorldBounds();

                if (CSGen.CoordInSafeZones(args, coords)
                    || (system_faction != faction
                        && generator.AreThereAnyObjects(system_id, coords.x, coords.y, 50,
                            { except_ships: true, except_asteroids: true, except_special_objects: true }))) {
                    if (++attempt > 30) {
                        // cannot spawn this npc
                        return step + 1;
                    }

                    continue;
                }

                break;
            }
            while (true);

            // here we decide which size of group we want to create
            groupsize = 1;
            if (MathExt.RandInt() % 20 == 0) {
                // no big groups in non-danger systems (2-3 instead of 2-5)
                groupsize = dangerLevel > 10 ? MathExt.RandRange(2, 6) : MathExt.RandRange(2, 4);
            }

            // set all currents to zero
            for (var i = 0; i < possibleNpcs.length; i++) {
                possibleNpcs[i].currentQuantity = 0;
            }

            // pick members of group
            for (var groupMemberIndex = 0; groupMemberIndex < groupsize; groupMemberIndex++) {
                // pick all available npcs into one array
                var actualGroups = [];
                for (var j = 0; j < possibleNpcs.length; j++) {
                    var possibleNpc = possibleNpcs[j];

                    if (possibleNpc.currentQuantity < possibleNpc.max_in_group) {
                        actualGroups.push(possibleNpc);
                    }
                    else {
                        //console.PrintError("Violation: " + possibleNpcs[k].max_in_group +" , " + possibleNpcs[k].currentQuantity);
                        //console.WaitForUserInteraction();
                        break;
                    }
                }

                if (actualGroups.length == 0)
                {
                    //console.PrintError("No actual npc will be spawned...");
                    step++;
                    break;
                }

                // take random group between picked
                var picked = pickByChance(actualGroups);
                actualGroups[picked].currentQuantity++;
                var shipByRandom = actualGroups[picked];
                level = dangerLevel + Math.round(MathExt.RandDouble() * 4 - 2);
                level = utils.Clamp(level, 1, 100);

                // pick random behavior
                var behavior = "";
                if (typeof (behaviors) == "string") {
                    behavior = behaviors;
                }
                else {
                    picked = pickByChance(behaviors);
                    behavior = behaviors[picked].name;
                }

                //console.PrintError("" + shipByRandom.xml_type);
                tags.xmlnpcship = shipByRandom.xml_type;


                id = generator.AddNPCShipToSystem(
                    name, behavior, level, shipByRandom.xml_type,
                    system_id,
                    coords.x + MathExt.RandRange(3, 9) * (MathExt.RandInt() % 2 == 0 ? -1 : 1),
                    coords.y + MathExt.RandRange(3, 9) * (MathExt.RandInt() % 2 == 0 ? -1 : 1),
                    tags);

                // console.Print("Added " + shipByRandom.xml_type);

                relations.SetShipFaction(id, faction);

                step++;
                npcs_generated_during_iteration++;
            }

            //something was created - return current step
            if (npcs_generated_during_iteration > max_generations_per_iteration) {
                return step;
            }
        }*/
        //cycle is over - return -1
        return -1;
    },
};

//"generic_pirates_shuttle_01",
//"generic_pirates_shuttle_02",
//"generic_pirates_heretic_01",
//"generic_pirates_heretic_02",

//"generic_pirates_orca_01",
//"generic_pirates_orca_02",
//"generic_pirates_orca_03",
//"generic_pirates_scarab_01",
//"generic_pirates_scarab_02",
//"generic_pirates_scarab_mk2_01",
//"generic_pirates_scarab_mk2_02",
//"generic_pirates_scarab_mk2_03"
//"generic_pirates_boomerang_01",
//"generic_pirates_boomerang_02",
//"generic_pirates_banshee_01",
//"generic_pirates_banshee_02",
//"generic_pirates_banshee_03",
//"generic_pirates_banshee_mk2_01",
//"generic_pirates_banshee_mk2_02",
//"special_human_tradership_asteria"
//"generic_pirates_aurora_01",
//"generic_pirates_aurora_02",
//"generic_pirates_aurora_03",
//"generic_pirates_avalanche_01",
//"generic_pirates_avalanche_02",
//"generic_pirates_avalanche_mk2_01",
//"generic_pirates_avalanche_mk2_02",
//"generic_pirates_avalanche_mk2_03",
//"generic_pirates_crab_01",
//"generic_pirates_crab_02",
//"generic_pirates_crab_03"
//"generic_pirates_death_bringer_01",
//"generic_pirates_death_bringer_02",
//"generic_pirates_endeavor_01",
//"generic_pirates_endeavor_02",
//"special_human_tradership_endeavor"



//"generic_guard_hammer_01",
//"generic_guard_hammer_02",
//"generic_pirates_hammer_01",

//"special_human_tradership",
//"generic_pirates_stingray_01",
//"generic_pirates_stingray_02"

//"generic_pirates_stingray_mk2_01",
//"generic_pirates_stingray_mk2_02"


//"generic_pirates_stryker_01",
//"generic_pirates_stryker_02",

//generic_pirates_wasp


var arrayOfPiratesXML =
    [
        "generic_pirates_yamato_01",
        "generic_pirates_yamato_02",
    ];



var arrayOfTurretsXML =
    [
        "special_human_turret",
        "special_human_turret_mk2",
        "special_human_turret_mk3",
    ];








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
