/*
 ===============================================================
 Station logics
 ===============================================================
 */

using(ship);
using(console);
using(station);
using(game);
using(generator);
using(storage);

/*
 =========================================================
 Responsible for station and trade basic logics.
 =========================================================
 */

// statuses
var STATUS_NOT_DEFINED = -1;
var STATUS_NEUTRAL = 0;
var STATUS_CONSUMPTION = 1;
var STATUS_PRODUCTION = 2;

// prices
var PRICE_NOT_DEFINED = -1;


/*
 calculates a price of an item on a specific base
 (functions for sell and buy are separate)
 
 in:
 item_id
 base_id
 ship_id
 out:
 double factor
 */

function Clamp(min, current, max)
{
    if (current < min)
    {
        return min;
    }
    if (current > max)
    {
        return max;
    }
    return current;
}

// returns relation coef from -1 to 1 (higher means faction likes player more)
function GetFactionRelationCoef(relation)
{
    relation = Clamp(-150.0, relation, 150.0);
    return relation / 150.0;
}

function CalculateSellPriceOfItemModifier(args, min, max)
{
    var min = 0.4;
    var max = 0.9;
    var trade_margin = -ship.GetFinalCacheValue(args.ship_id, "trade_margin");
    var relation_coef = 0.2 * GetFactionRelationCoef(args.faction_relation);
    var coef = min + trade_margin + relation_coef;
    coef = Clamp(min, coef, max);
    //console.Print("SellPriceCoef: " + coef + " based on tradeMargin=" + trade_margin + " relation_coef=" + relation_coef);
    return coef;
}

function CalculateBuyPriceOfItemModifier(args)
{
    var min = 1;
    var max = 1.25;
    var trade_margin = -ship.GetFinalCacheValue(args.ship_id, "trade_margin");
    var relation_coef = 0.1 * GetFactionRelationCoef(args.faction_relation);
    var coef = max - trade_margin - relation_coef;
    coef = Clamp(min, coef, max);
    //console.Print("BuyPriceCoef: " + coef + " based on tradeMargin=" + trade_margin + " relation_coef=" + relation_coef);
    return coef;
}

/*
 calculates price of cargo on a specific base
 in:
 cargo_id
 base_id
 ship_id
 ship_faction
 base_faction
 base_quantity_neutral
 base_quantity_production
 base_quantity_consumption
 cargo_price
 status
 out:
 double price
 */
function CalculateSellPriceOfCargo(args)
{
    var min = 0.95;
    var max = 0.99;
    var trade_margin = -ship.GetFinalCacheValue(args.ship_id, "trade_margin");
    var relation_coef = 0.2 * GetFactionRelationCoef(args.faction_relation);
    var coef = min + (trade_margin + relation_coef) * 0.04;

    coef = Clamp(min, coef, max);
    //console.PrintError("CARGO SellPriceCoef: " + coef + " based on tradeMargin=" + trade_margin + " relation_coef=" + relation_coef);
    return args.cargo_price * coef;
}

function CalculateBuyPriceOfCargo(args)
{
    var min = 1;
    var max = 1.05;
    var trade_margin = -ship.GetFinalCacheValue(args.ship_id, "trade_margin");
    var relation_coef = 0.2 * GetFactionRelationCoef(args.faction_relation);
    var coef = min - (trade_margin + relation_coef) * 0.05;

    coef = Clamp(min, coef, max);
    //console.PrintError("CARGO BuyPriceCoef: " + coef + " based on tradeMargin=" + trade_margin + " relation_coef=" + relation_coef);
    return args.cargo_price * coef;
}

/*
 in:
 base_id
 faction
 resource_id
 chance_consumption
 chance_production
 current_status
 base_quantity_consumption
 base_quantity_production
 base_quantity_neutral
 out:
 int status (use defines)
 int quantity
 */

function SetResourceStatus(args)
{
    var isConsumption = MathExt.RandDouble() < args.chance_consumption;
    var isProduction = false;
    
    if (!isConsumption)
    {
        isProduction = MathExt.RandDouble() < args.chance_production;
    }
    
    // disallow production-consumption chain in the single star system
    // otherwise it will be too easy to earn money and experience from trading
    var resourceId = args.resource_id;
    if (isProduction)
    {
        if (CheckResourceStatusOnOtherStationsInSystem(args.base_id, resourceId, STATUS_CONSUMPTION))
        {
            isProduction = false;
            //console.PrintError("Disallowed production of " + resourceId + " on station: " + args.base_id);
        }
    }
    else if (isConsumption)
    {
        if (CheckResourceStatusOnOtherStationsInSystem(args.base_id, resourceId, STATUS_PRODUCTION))
        {
            isConsumption = false;
            //console.PrintError("Disallowed consumption of " + resourceId + " on station: " + args.base_id);
        }
    }

    var status = STATUS_NEUTRAL;
    if (isConsumption)
    {
        status = STATUS_CONSUMPTION;
    }
    else if (isProduction)
    {
        status = STATUS_PRODUCTION;
    }

    var quantity = args.base_quantity_neutral;
    if (isConsumption)
    {
        quantity = args.base_quantity_consumption;
    }
    else if (isProduction)
    {
        quantity = args.base_quantity_production;
    }

    var quantity_coef = MathExt.RandRangeDouble(0.8, 1.2);
    quantity = Math.round(quantity * quantity_coef)

    return { status: status, quantity: quantity };
}

function CheckResourceStatusOnOtherStationsInSystem(station_id, resource_id, status)
{
    var system_id = station.GetSystemID(station_id);

    var stations = generator.GetStationsAtSystem(system_id);
    for (var i = 0; i < stations.length; i++)
    {
        var other_station_id = stations[i];
        if (other_station_id == station_id)
        {
            continue;
        }

        if (station.GetResourceStatus(other_station_id, resource_id) == status)
        {
            return true;
        }
    }

    return false;
}

/*
 in:
 base_id
 faction
 resource_id
 status
 current_price
 base_price
 base_quantity_consumption
 base_quantity_production
 base_quantity_neutral
 current_quantity
 out:
 double price
 double quantity
 */
function SetResourceBasePrice(args)
{
    if (args.current_price == PRICE_NOT_DEFINED)
    {
        // calculate initial resource price
        var mean = 0;
        var sigma = 0.05;
        var price_coef_offset = MathExt.RandNormalRangeDouble(-0.1, 0.1, mean, sigma);
        var price_coef = GetCargoPriceCoeffiecientByStatus(args.status) + price_coef_offset;
        var price = Math.max(1, args.base_price * price_coef);

        var quantity = GetQuantityByStatus(args);
        var quantity_coef = MathExt.RandRangeDouble(0.8, 1.2);
        quantity = Math.round(quantity * quantity_coef);

        //GALAXY MARKET MOD//
        var arrayOfPrice = [];
        var globalShopPriceVariable = storage.GetGlobal("globalShopPrice", arrayOfPrice);
        globalShopPriceVariable.push({ price: price, resourceID: args.resource_id, base_id: args.base_id, faction: args.faction, currentPrice: args.current_price });
        storage.SetGlobal("globalShopPrice", globalShopPriceVariable);
        //GALAXY MARKET MOD//

        return { price: price, quantity: quantity };
    }
    else
    {
        return { price: args.current_price, quantity: args.current_quantity };

        //// this algorithm is a black magic and we've decided to not use it anymore
        //
        //// offset algorithm
        //var price_coef = GetCargoPriceCoeffiecientByStatus(args.status);
        //var oldq = args.current_price / args.base_price;
        //var diff = price_coef - oldq;

        //var x = Math.abs(diff) / 0.5;
        //if (x > 1)
        //{
        //    x = 1;
        //}
        //var chance = (1 + Math.min(1, x * 2)) * 0.5;
        //var sign = MathExt.RandDouble() <= chance ? 1 : -1;

        //var mean = 0;
        //var sigma = 0.2;
        //var price_coef_offset = MathExt.RandNormalRangeDouble(0, 1, mean, sigma) * 0.12 + 0.03;
        //var newq = oldq + diff / Math.abs(diff) * price_coef_offset * sign;

        //var quantity = GetQuantityByStatus(args);
        //quantity = args.current_quantity + Math.round((quantity - args.current_quantity) * 0.15);

        //return {
        //    price: Math.max(1, args.base_price * newq),
        //    quantity: quantity
        //};
    }
}

function GetCargoPriceCoeffiecientByStatus(status)
{
    if (status == STATUS_CONSUMPTION)
    {
        return 1.2;
    }
    else if (status == STATUS_PRODUCTION)
    {
        return 0.8;
    }

    return 1;
}

function GetQuantityByStatus(args)
{
    if (args.status == STATUS_CONSUMPTION)
    {
        return args.base_quantity_consumption;
    }
    else if (args.status == STATUS_PRODUCTION)
    {
        return args.base_quantity_production;
    }

    return args.base_quantity_neutral;
}

/*
 ==================================================================
 Calculate prices for additional services
 ==================================================================
 */

/*
 Calculate refuel price
 in:
 base_id
 ship_id
 fuel_amount
 out:
 double price
 */
function CalculateRefuelPrice(args)
{
    var k = ship.GetFinalCacheValue(args.ship_id, "refuel_price");
    var money = args.fuel_amount * (100 - k) / 100;
    return money;
}

/*
 Calculate repair price
 int:	
 base_id
 ship_id
 repair_amount
 repair_amount_relative
 hull_price_total
 out:
 double price
 */
function CalculateRepairPrice(args)
{
    if (args.hull_price_total > 0)
    {
        //basic price
        var basic_price = args.repair_amount * 0.15; //0.15 is based on consumable price, 150d for 500hp, thus 0.3d per hp, so, slightly cheaper to repair at the station

        //discount
        var discount = ship.GetFinalCacheValue(args.ship_id, "repair_price");
        var price = basic_price * (100 - discount) / 100;

        return price;
    }

    //free for shuttle
    return 0;
}