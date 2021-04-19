include(IMCache.js);
include(ShipUpdate.js);

using(generator);
using(relations);
using(spawn);
using(actions);
using(ship);
using(console);
using(game);


//added from 9sMODs
var sc_imcache =
{
    ClearNSetNpcLeveledCache: function (shipID, speedType, player_id) {
        if (speedType == 0) {

            IMCache.Clear();

            var level = ship.GetLevel(player_id); //args.level;
            var maxlevel = 100;
            var lerp = 1 - (maxlevel - level) / maxlevel;

            //hull & booster
            IMCache.AddValue("structure_value", 400 + lerp * 2000);
            IMCache.AddValue("armor_value", 0 + lerp * 25);

            if (ship.GetTag(shipID, "someClass") != null) {
                if (ship.GetTag(shipID, "someClass") == "drone") {
                    console.PrintError("drone");
                    //engine
                    IMCache.AddValue("speed_max_value", 500 + lerp * 200);
                    IMCache.AddValue("acceleration_value", 200 + lerp * 200);
                    IMCache.AddValue("maneuvering_value", 500 + lerp * 300);
                    IMCache.AddValue("speed_reverse_special", 1.75);
                    IMCache.AddValue("speed_strafe_special", 0.5);
                    IMCache.AddValue("deceleration_special", 5);
                }
            }
            else {
                //engine
                IMCache.AddValue("speed_max_value", 500 + lerp * 200);
                IMCache.AddValue("acceleration_value", 200 + lerp * 200);
                IMCache.AddValue("maneuvering_value", 500 + lerp * 300);
                IMCache.AddValue("speed_reverse_special", 0.5);
                IMCache.AddValue("speed_strafe_special", 0.8);
                IMCache.AddValue("deceleration_special", 1);
            }

            //generator
            IMCache.AddValue("energy_production_generator_value", 25 + lerp * 100);
            IMCache.AddValue("energy_capacity_value", 250 + lerp * 1000);

            //shields
            IMCache.AddValue("shield_charge_value", 250 + lerp * 500);
            IMCache.AddValue("shield_regeneration_value", 10 + lerp * 65);
            IMCache.AddValue("shield_deflection_value", 80);
            IMCache.AddValue("shield_reestablishment_value", 100);
            IMCache.AddValue("shield_regeneration_delay_value", 1 + 2 * (1 - lerp));
            IMCache.AddValue("energy_consumption_shield_value", 5 + lerp);

            //targeting
            IMCache.AddValue("radar_targeting_range_value", 20 + lerp * 10); //20-30
            IMCache.AddValue("radar_targeting_speed_value", 400 + lerp * 850); //400-1250
            IMCache.AddValue("radar_targeting_defense", 800 + lerp * 1200);

            // set constants
            //IMCache.AddValue("weapon_tracking_speed_value", 1);
            //IMCache.AddValue("weapon_tracking_angle_value", 1);

            IMCache.AddValue("mining_amount_value", 1);
            IMCache.AddValue("mining_range_value", 1);
            IMCache.AddValue("mining_speed_value", 1);

            //set resulting value	
            IMCache.ApplyCacheFunction(shipID,
                function (ship_id, param_name, param_value) {
                    ship.SetEquipmentCacheValue(ship_id, param_name, param_value);
                });
        }
        else {

            IMCache.Clear();

            var level = ship.GetLevel(player_id); //args.level;
            var maxlevel = 100;
            var lerp = 1 - (maxlevel - level) / maxlevel;

            //hull & booster
            IMCache.AddValue("structure_value", 400 + lerp * 2000);
            IMCache.AddValue("armor_value", 0 + lerp * 25);

            console.PrintError("lerp: " + lerp);

            if (ship.GetTag(shipID, "someClass") != null) {
                if (ship.GetTag(shipID, "someClass") == "drone") {

                 /*console.PrintError("drone");
                   //engine
                   IMCache.AddValue("speed_max_value", 500 + lerp * 200); //500//200//135
                   IMCache.AddValue("acceleration_value", 200 + lerp * 200); //200//200//135
                   IMCache.AddValue("maneuvering_value", 500 + lerp * 300); //500//300//750
                   IMCache.AddValue("speed_reverse_special", 3.5); //1.75
                   IMCache.AddValue("speed_strafe_special", 7.5 + lerp * 150);  //1.75
                   IMCache.AddValue("deceleration_special", 12.5 + lerp * 150);  //1.75*/

                    /*console.PrintError("drone");
                    //engine
                    IMCache.AddValue("speed_max_value", 175 + lerp * 100); //500//200//135
                    IMCache.AddValue("acceleration_value", 250 + lerp * 100); //200//200//135
                    IMCache.AddValue("maneuvering_value", 800 + lerp * 100); //500//300//750/1000 is a bit much for 550xml speed and npc.Stop is making the drone completely stop.
                    IMCache.AddValue("speed_reverse_special", 3.5); //1.75
                    IMCache.AddValue("speed_strafe_special", 7.5 + lerp * 150);  //1.75
                    IMCache.AddValue("deceleration_special", 12.5 + lerp * 150);  //1.75*/

                    console.PrintError("drone");
                    //engine
                    IMCache.AddValue("speed_max_value", 50 + lerp * 100); //500//200//135
                    IMCache.AddValue("acceleration_value", 50 + lerp * 100); //200//200//135
                    IMCache.AddValue("maneuvering_value", 800 + lerp * 100); //500//300//750/1000 is a bit much for 550xml speed and npc.Stop is making the drone completely stop.
                    IMCache.AddValue("speed_reverse_special", 3.5); //1.75
                    IMCache.AddValue("speed_strafe_special", 7.5 + lerp * 150);  //1.75
                    IMCache.AddValue("deceleration_special", 12.5 + lerp * 150);  //1.75
                }
            }
            else {
                //engine
                IMCache.AddValue("speed_max_value", 500 + lerp * 200);
                IMCache.AddValue("acceleration_value", 200 + lerp * 200);
                IMCache.AddValue("maneuvering_value", 500 + lerp * 300);
                IMCache.AddValue("speed_reverse_special", 0.5);
                IMCache.AddValue("speed_strafe_special", 0.8);
                IMCache.AddValue("deceleration_special", 1);
            }



            //generator
            IMCache.AddValue("energy_production_generator_value", 25 + lerp * 100);
            IMCache.AddValue("energy_capacity_value", 250 + lerp * 1000);

            //shields
            IMCache.AddValue("shield_charge_value", 250 + lerp * 500);
            IMCache.AddValue("shield_regeneration_value", 10 + lerp * 65);
            IMCache.AddValue("shield_deflection_value", 80);
            IMCache.AddValue("shield_reestablishment_value", 100);
            IMCache.AddValue("shield_regeneration_delay_value", 1 + 2 * (1 - lerp));
            IMCache.AddValue("energy_consumption_shield_value", 5 + lerp);

            //targeting
            IMCache.AddValue("radar_targeting_range_value", 20 + lerp * 10); //20-30
            IMCache.AddValue("radar_targeting_speed_value", 400 + lerp * 850); //400-1250
            IMCache.AddValue("radar_targeting_defense", 800 + lerp * 1200);

            // set constants
            //IMCache.AddValue("weapon_tracking_speed_value", 1);
            //IMCache.AddValue("weapon_tracking_angle_value", 1);

            IMCache.AddValue("mining_amount_value", 1);
            IMCache.AddValue("mining_range_value", 1);
            IMCache.AddValue("mining_speed_value", 1);

            //set resulting value	
            IMCache.ApplyCacheFunction(shipID,
                function (ship_id, param_name, param_value) {
                    ship.SetEquipmentCacheValue(ship_id, param_name, param_value);
                });
        }
    }
}
