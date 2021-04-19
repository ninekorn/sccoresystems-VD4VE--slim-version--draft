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
var tnosivl = 0;
var losivl = [];
var css;
var sbs;

function OnCheckRequirements() {
	var npid = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npid, "class") == "marketBUYShields") {
		return true;
	}
	return false;
}

function OnDialogue(args)
{
	var pid = PLAYER_SHIP;
    var na = game.GetShipOwner(pid);

    pin = topic.GetInput();
	sa = topic.GetState();
	var pshp = PLAYER_SHIP; 

    if (pin == NO_INPUT)
	{
		var arr1;
        sbs = storage.GetGlobal("marketTerminalChosenStation" + pid, arr1);
		css = sbs;

        scid = items.GetStationShopContainerId(sbs);

		var iicg = items.GetItemsAndCargo(scid);
		losivl = [];
        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 6) == "shield") {
                losivl.push(iicg[i]);
			}
		}

		topic.AddChoice(120000, "Shields");
		topic.AddChoice(10, "Disconnect");
	}

    else if (pin == 10) {
		topic.DialogueBreak();
	}
    else if (pin == 11) {
        player.StartDialogue(na, "marketBUYStations");
	}

	////////////////////////////SHIELD/////////////////////////////////////////////
    else if (pin == 120000) {

        var iicg = items.GetItemsAndCargo(scid);
		losivl = [];
        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 6) == "shield") {
                losivl.push(iicg[i]);
			}
		}

		var i;
		for (i = 0; i < losivl.length; i++) {

            var pmd = CalculateBuyPriceOfItemModifier(pid);
			var obj = generator.GetItemByXmlID(losivl[i].xml_id);
            var shpr = Math.ceil(obj.price * pmd);

            var cstrg = losivl[i].xml_id.replace(/_/g, " ");
            topic.AddChoice(1200000 + i, cstrg.toUpperCase() + " " + "Price: " + "$" + shpr + ".");

		}
		tnosivl = i;
		topic.AddChoice(11, "Go back to list of stations");
		topic.AddChoice(10, "Disconnect");
	}

	
	////////////////////////////SHIELD INDIVIDUAL SECTION/////////////////////////////////////////////
    else if (pin >= 1200000 && pin < 1200000 + tnosivl)
	{			
        var iidx = pin - 1200000;
	
		var eid;
		var exmid;
	
        eid = losivl[iidx];		
        exmid = losivl[iidx].xml_id;

        var pmd = CalculateBuyPriceOfItemModifier(pid);
        var obj = generator.GetItemByXmlID(exmid);
        var shpr = Math.ceil(obj.price * pmd);

        var mon = player.GetMoney(na);

        if (mon < shpr) {
            var csit = exmid.replace(/_/g, " ");

            topic.AddPhrase("You do not have enough money to buy the " + csit.toUpperCase() + " .Price: " + "$" + shpr + " Money: " + "$" + mon);
            topic.AddChoice(120000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
        else {
            var csit = exmid.replace(/_/g, " ");
            topic.AddPhrase("" + csit.toUpperCase() + "" + " Buy price:" + "$" + shpr + ".");
            topic.AddChoice(12000000 + iidx, "Buy this item?");
            topic.AddChoice(120000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
	}

    else if (pin >= 12000000 && pin < 12000000 + losivl.length)
	{
        var iidx = pin - 12000000;
	
        var eid;
        var exmid;
		
        eid = losivl[iidx];		
        exmid = losivl[iidx].xml_id;	

        var spsid = items.GetShipStationStorageContainerId(pshp,css);

        var obj = generator.AddItemToSpecifiedContainer(spsid, exmid, 1); 	
		
        items.RemoveItemQuantity(scid, eid.item_id, 1); 

        var pmd = CalculateBuyPriceOfItemModifier(pid);
        var obj = generator.GetItemByXmlID(exmid);
        var spce = Math.ceil(obj.price * pmd);

        player.RemoveMoney(na, spce);

        var csit = exmid.replace(/_/g, " ");
        topic.AddPhrase("You have bought " + csit.toUpperCase() + " for " + "$" + spce + ".");
		topic.AddChoice(120000, "Go Back!");
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