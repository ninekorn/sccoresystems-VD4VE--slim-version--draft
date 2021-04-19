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
var scid;
var tnobivl = 0;
var lobivl = [];
var css;
var sb;

function OnCheckRequirements() {
	var npid = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npid, "class") == "marketBUYBoosters") {
		return true;
	}
	return false;
}

function OnDialogue(args) {
	var pid = PLAYER_SHIP;

    var na = game.GetShipOwner(pid);

    pin = topic.GetInput();
    sta = topic.GetState();
    var psh = PLAYER_SHIP; 

    if (pin == NO_INPUT)
	{
		var arr;
		var anotherWhatever1;
        sb = storage.GetGlobal("marketTerminalChosenStation" + pid, anotherWhatever1);
		css = sb;

        scid = items.GetStationShopContainerId(sb);

		var iicg = items.GetItemsAndCargo(scid);
		lobivl = [];
		var i;
        for (i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 7) == "booster") {
                lobivl.push(iicg[i]);
			}
		}

		topic.AddChoice(130000, "Boosters");
		topic.AddChoice(10, "Disconnect");
	}

    else if (pin == 10) {
		topic.DialogueBreak();
	}

    else if (pin == 11) {
        player.StartDialogue(na, "marketBUYStations");
	}


	////////////////////////////BOOSTER/////////////////////////////////////////////
    else if (pin == 130000) {

        var iicg = items.GetItemsAndCargo(scid);
		lobivl = [];
		var i;
        for (i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 7) == "booster") {
                lobivl.push(iicg[i]);
			}
		}

		var ii;
		for (ii = 0; ii < lobivl.length; ii++)
		{

            var pmd = CalculateBuyPriceOfItemModifier(pid);
			var obj = generator.GetItemByXmlID(lobivl[ii].xml_id);
            var sprc = Math.ceil(obj.price * pmd);
            var cstg = lobivl[ii].xml_id.replace(/_/g, " ");
            topic.AddChoice(1300000 + ii, cstg.toUpperCase() + " " + "Price: " + "$" + sprc + ".");
        }

		tnobivl = ii;
		topic.AddChoice(11, "Go back to list of stations");
		topic.AddChoice(10, "Disconnect");
	}


	////////////////////////////BOOSTER INDIVIDUAL SECTION/////////////////////////////////////////////
    else if (pin >= 1300000 && pin < 1300000 + tnobivl)
	{			

        var iind = pin - 1300000;

        var eid;
        var exmid;

        eid = lobivl[iind];
        exmid = lobivl[iind].xml_id;

        var pmd = CalculateBuyPriceOfItemModifier(pid);
        var obj = generator.GetItemByXmlID(exmid);
        var spc = Math.ceil(obj.price * pmd);

        var mon = player.GetMoney(na);

        if (mon < spc) {
            var csit = exmid.replace(/_/g, " ");

            topic.AddPhrase("You do not have enough money to buy the " + csit.toUpperCase() + " .Price: " + "$" + spc + " Money: " + "$" + mon);
            topic.AddChoice(150000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
        else {
            var csit = exmid.replace(/_/g, " ");
            topic.AddPhrase("" + csit.toUpperCase() + "" + " Buy price:" + "$" + spc + ".");
            topic.AddChoice(13000000 + iind, "Buy this item?");
            topic.AddChoice(130000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
		
	}
    else if (pin >= 13000000 && pin < 13000000 +lobivl.length)
	{
        var iind = pin - 13000000;

        var eid;
        var exmid;

        eid = lobivl[iind];
        exmid = lobivl[iind].xml_id;

        var spsid = items.GetShipStationStorageContainerId(pid, css);

        var obj = generator.AddItemToSpecifiedContainer(spsid, exmid, 1);

        items.RemoveItemQuantity(scid, eid.item_id, 1);

        var itr = lobivl.indexOf(eid);
        lobivl.splice(itr, 1);

        var pmd = CalculateBuyPriceOfItemModifier(pid);
        var obj = generator.GetItemByXmlID(exmid);
        var spc = Math.ceil(obj.price * pmd);

        player.RemoveMoney(na, spc);

        var csit = exmid.replace(/_/g, " ");
        topic.AddPhrase("You have bought " + csit.toUpperCase() + " for " + "$" + spc + ".");
        topic.AddChoice(130000, "Go Back!");
        topic.AddChoice(10, "Disconnect");
	}
	
}

function CalculateBuyPriceOfItemModifier(args)
{
	var mi = 1;
	var ma = 1.25;
	var tmg = -ship.GetFinalCacheValue(args, "trade_margin");
	var sinf = station.GetBaseByID(sb);

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