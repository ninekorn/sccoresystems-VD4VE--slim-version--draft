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

var sa;
var pin;
var scid;
var tnoeivl = 0;
var loeivl = [];
var css;
var sbs;


function OnCheckRequirements() {
	var npid = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npid, "class") == "marketBUYEngines") {
		return true;
	}
	return false;
}

function OnDialogue(args)
{
	var pid = PLAYER_SHIP;
    var name = game.GetShipOwner(pid);

    pin = topic.GetInput();
	sa = topic.GetState();

	var psp = PLAYER_SHIP; 

    if (pin == NO_INPUT)
	{
		var arr1;
        sbs = storage.GetGlobal("marketTerminalChosenStation" + pid, arr1);
		css = sbs;

        scid = items.GetStationShopContainerId(sbs);

		var iicg = items.GetItemsAndCargo(scid);
		loeivl = [];
		var i;
        for (i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 6) == "engine") {
                loeivl.push(iicg[i]);
			}
		}
		topic.AddChoice(60000, "Engines");
		topic.AddChoice(10, "Disconnect");
	}

    else if (pin == 10) {
		topic.DialogueBreak();
	}
    else if (pin == 11) {
		player.StartDialogue(name, "marketBUYStations");
	}
	////////////////////////////ENGINES/////////////////////////////////////////////
    else if (pin == 60000) {

        var iicg = items.GetItemsAndCargo(scid);
		loeivl = [];

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 6) == "engine") {
                loeivl.push(iicg[i]);
			}
		}

		var i;
		for (i = 0; i < loeivl.length; i++) {

            var pmd = CalculateBuyPriceOfItemModifier(pid);
			var obj = generator.GetItemByXmlID(loeivl[i].xml_id);
            var spce = Math.ceil(obj.price * pmd);

            var cstg = loeivl[i].xml_id.replace(/_/g, " ");
            topic.AddChoice(600000 + i, cstg.toUpperCase() + " " + "Price: " + "$" + spce + ".");

		}

		tnoeivl = i;
		topic.AddChoice(11, "Go back to list of stations");
		topic.AddChoice(10, "Disconnect");
	}

	////////////////////////////ENGINES INDIVIDUAL SECTION/////////////////////////////////////////////
    else if (pin >= 600000 && pin < 600000 + tnoeivl) {

        var iidx = pin - 600000;

        var eid;
        var exmid;

        eid = loeivl[iidx];
        exmid = loeivl[iidx].xml_id;

        var pmd = CalculateBuyPriceOfItemModifier(pid);
        var obj = generator.GetItemByXmlID(exmid);
        var spc = Math.ceil(obj.price * pmd);

        var mon = player.GetMoney(name);

        if (mon < spc) {
            var csit = exmid.replace(/_/g, " ");

            topic.AddPhrase("You do not have enough money to buy the " + csit.toUpperCase() + " .Price: " + "$" + spc + " Money: " + "$" + mon);
            topic.AddChoice(60000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
        else {
            var csit = exmid.replace(/_/g, " ");
            topic.AddPhrase("" + csit.toUpperCase() + "" + " Buy price:" + "$" + spc + ".");
            topic.AddChoice(6000000 + iidx, "Buy this item?");
            topic.AddChoice(60000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
	}
    else if (pin >= 6000000 && pin < 6000000 + loeivl.length) {
        var iidx = pin - 6000000;

        var eid;
        var exmid;

        eid = loeivl[iidx];
        exmid = loeivl[iidx].xml_id;


        var spsid = items.GetShipStationStorageContainerId(psp,css);

        var obj = generator.AddItemToSpecifiedContainer(spsid, exmid, 1);

        items.RemoveItemQuantity(scid, eid.item_id, 1);

        var pmd = CalculateBuyPriceOfItemModifier(pid);
        var obj = generator.GetItemByXmlID(exmid);
        var spc = Math.ceil(obj.price * pmd);

        player.RemoveMoney(name, spc);
        var cstg = exmid.replace(/_/g, " ");

        topic.AddPhrase("You have bought " + cstg.toUpperCase() + " for " + "$" + spc + ".");
		topic.AddChoice(60000, "Go Back!");
		topic.AddChoice(10, "Disconnect");
	}

	

}
function CalculateBuyPriceOfItemModifier(args) {
    var mi = 1;
    var ma = 1.25;
    var tmg = -ship.GetFinalCacheValue(args, "trade_margin");
    var sinf = station.GetBaseByID(sbs);

    var fct = relations.GetFactionDispositionToShip(sinf.faction, args);
    var rcoe = 0.1 * GetFactionRelationCoef(fct);
    var coef = ma - tmg - rcoe;
    coef = Clamp(mi, coef, ma);
    return coef;
}

function GetFactionRelationCoef(re) {
    re = Clamp(-150.0, re, 150.0);
    return re / 150.0;
}

function Clamp(mi, cur, ma) {
    if (cur < mi) {
        return mi;
    }
    if (cur > ma) {
        return ma;
    }
    return cur;
}


