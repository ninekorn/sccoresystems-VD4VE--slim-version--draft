//----------------------------------------------
// small lib to simplify calculations

var ParamUpdater =
    {
        /*
             args:
             energy:
             energy_max:
             shield_max:
             shield_current:
             shield_regeneration_per_frame:
             shield_regeneration_per_frame_cost:
             */
        CalculateShield: function(args)
        {
            var regenerated = args.shield_regeneration_per_frame;

            if (args.shield_current + regenerated > args.shield_max)
            {
                regenerated = args.shield_max - args.shield_current;
            }

            var energy_spent = 0;

            // if need shield regeneration
            if (regenerated > 0)
            {
                energy_spent = args.shield_regeneration_per_frame_cost
                    * (regenerated / args.shield_regeneration_per_frame);

                if (energy_spent > args.energy)
                {
                    // if can't regenerate all shield regenerate as much as it can
                    regenerated *= args.energy / args.shield_regeneration_per_frame_cost;
                    energy_spent = args.energy;
                }
            }

            return {
                shield_regenerated: regenerated,
                energy_spent: energy_spent
            };
        }
    };