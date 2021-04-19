using(relations);
using(npc);
using(console);
using(ship);
using(generator);
using(game);
using(items);
using(player);
using(storage);
using(station);

var sta;
var pin;

var cusipt;
var scid;

var lowivl = [];
var tloweivl = 0;

var css;
var sbs;

function OnCheckRequirements() {
	var npid = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npid, "class") == "marketSELLWeapons") {
		return true;
	}
	return false;
}

function OnDialogue(args)
{
    var pshp = PLAYER_SHIP; 

    var na = game.GetShipOwner(pshp);

    pin = topic.GetInput();
    sta = topic.GetState();

    if (pin == NO_INPUT)
	{
		var arr1;
        sbs = storage.GetGlobal("marketTerminalChosenStation" + pshp, arr1);
		css = sbs;

		//shopContainerID = items.GetStationShopContainerId(sbs);
        scid = items.GetShipStationStorageContainerId(pshp, css);

        var iicg = items.GetItemsAndCargo(scid);
		lowivl = [];

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 2) == "we") {
                lowivl.push(iicg[i]);
			}
		}

		//topic.AddChoice(140000, "Weapons");
        //topic.AddChoice(10, "Disconnect");

        if (lowivl.length > 0) {
            topic.AddChoice(200000, "Weapons");
        }

        topic.AddChoice(11, "Go back to list of stations");
        topic.AddChoice(10, "Disconnect");






	}

    else if (pin == 10) {
		topic.DialogueBreak();
	}

    else if (pin == 11) {
        player.StartDialogue(na, "marketSELLStations");
	}

	
	/*////////////////////////////WEAPONS/////////////////////////////////////////////
    else if (pin == 140000) {

        var iicg = items.GetItemsAndCargo(scid);
		lowivl = [];

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 2) == "we") {
                lowivl.push(iicg[i]);
			}
		}


	}*/

	/*////////////////////////////ENERGY WEAPONS/////////////////////////////////////////////
    else if (pin == 200000)
	{
		//topic.AddPhrase("Which Energy Weapon do you want to inspect?");	
		var i;
        for (i = 0; i < lowivl.length; i++) {

            var pmd = CalculateSellPriceOfItemModifier(pshp);
            var obj = generator.GetItemByXmlID(lowivl[i].xml_id);
            var spce = Math.ceil(obj.price * pmd);

            var cstg = lowivl[i].xml_id.replace(/_/g, " ");
            topic.AddChoice(2000000 + i, cstg.toUpperCase() + " " + "Price: " + spce + "$");
        }

        tloweivl = i;
		topic.AddChoice(6000 + cusipt, "Go Back!");
		topic.AddChoice(10, "Disconnect");
	}*/

    ////////////////////////////WEAPONS/////////////////////////////////////////////
    else if (pin == 200000) {
        var ii;
        for (ii = 0; ii < lowivl.length; ii++) {

            var pmd = CalculateSellPriceOfItemModifier(pshp);
            var obj = generator.GetItemByXmlID(lowivl[ii].xml_id);
            var shpp = Math.ceil(obj.price * pmd);
            var cstrg = lowivl[ii].xml_id.replace(/_/g, " ");
            topic.AddChoice(2000000 + ii, cstrg.toUpperCase() + " " + "Price: " + "$" + shpp + ".");
        }

        tloweivl = ii;
        topic.AddChoice(11, "Go back to list of stations");
        topic.AddChoice(10, "Disconnect");
    }


	////////////////////////////ENERGY WEAPON INDIVIDUAL SECTION/////////////////////////////////////////////
    else if (pin >= 2000000 && pin < 2000000 + tloweivl) {

        var iidx = pin - 2000000;

		var eid;
		var exmid;

        eid = lowivl[iidx];
        exmid = lowivl[iidx].xml_id;


        var pmd = CalculateSellPriceOfItemModifier(pshp);
        var obj = generator.GetItemByXmlID(exmid);
        var elpce = Math.ceil(obj.price * pmd);

        var cstg = exmid.replace(/_/g, " ");
        topic.AddPhrase("" + cstg.toUpperCase() + "" + " Sell price:" + elpce + "$.");
        topic.AddChoice(20000000 + iidx, "Sell this item?");
		topic.AddChoice(200000, "Go Back!");
		topic.AddChoice(10, "Disconnect");
	}

    else if (pin >= 20000000 && pin < 20000000 + lowivl.length) {
        var iidx = pin - 20000000;

        var eid;
        var exmid;

        eid = lowivl[iidx];
        exmid = lowivl[iidx].xml_id;

		var scid0 = items.GetStationShopContainerId(css);

        var obj = generator.AddItemToSpecifiedContainer(scid0, exmid, 1);

        var spsid = items.GetShipStationStorageContainerId(pshp, css);
        items.RemoveItemQuantity(spsid, eid.item_id, 1);

        var idxtr = lowivl.indexOf(eid);
        lowivl.splice(idxtr, 1);


        var pmd = CalculateSellPriceOfItemModifier(pshp);
        var obj = generator.GetItemByXmlID(exmid);
        var elpce = Math.ceil(obj.price * pmd);

        var na = game.GetShipOwner(pshp);
        player.AddMoney(na, elpce);

        var cstg = exmid.replace(/_/g, " ");
        topic.AddPhrase("You have sold " + cstg.toUpperCase() + " for " + elpce + "$.");
		topic.AddChoice(200000, "Go Back!");
		topic.AddChoice(10, "Disconnect");
	}
}
function CalculateSellPriceOfItemModifier(args) {
    var mi = 0.4;
    var ma = 0.9;
    var stin = station.GetBaseByID(sbs);
    var fact = relations.GetFactionDispositionToShip(stin.faction, args);

    var tdmg = -ship.GetFinalCacheValue(args, "trade_margin");

    var relc = 0.2 * GetFactionRelationCoef(fact);
    var coef = mi + tdmg + relc;
    coef = Clamp(mi, coef, ma);
    return coef;
}

function GetFactionRelationCoef(re) {
    re = Clamp(-150.0, re, 150.0);
    return re / 150.0;
}

function Clamp(mi, cu, ma) {
    if (cu < mi) {
        return mi;
    }
    if (cu > ma) {
        return ma;
    }
    return cu;
}