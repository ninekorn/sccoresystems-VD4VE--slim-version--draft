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

var tnowevl = 0;
var cswvlid;

var tnofweivl = [];

var css;
var sbs;

function OnCheckRequirements() {
	var npid = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npid, "class") == "marketBUYWeapons") {
		return true;
	}
	return false;
}

function OnDialogue(args)
{
	var pid = PLAYER_SHIP;
	//var shipID = args.ship_id;
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
		tnofweivl = [];

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 2) == "we") {
                tnofweivl.push(iicg[i]);
			}
		}

        if (tnofweivl.length > 0) {
            topic.AddChoice(200000, "Weapons");
        }
        topic.AddChoice(11, "Go back to list of stations");
        topic.AddChoice(10, "Disconnect");
	}

    else if (pin == 10) {
		topic.DialogueBreak();
	}

    else if (pin == 11) {
        player.StartDialogue(na, "marketBUYStations");
	}

    ////////////////////////////WEAPONS/////////////////////////////////////////////
    else if (pin == 200000) {
        var ii;
        for (ii = 0; ii < tnofweivl.length; ii++) {

            var pmd = CalculateBuyPriceOfItemModifier(pid);
            var obj = generator.GetItemByXmlID(tnofweivl[ii].xml_id);
            var shpp = Math.ceil(obj.price * pmd);
            var cstrg = tnofweivl[ii].xml_id.replace(/_/g, " ");
            topic.AddChoice(2000000 + ii, cstrg.toUpperCase() + " " + "Price: " + "$" + shpp + ".");
        }

        tnowevl = ii;
        topic.AddChoice(11, "Go back to list of stations");
        topic.AddChoice(10, "Disconnect");
    }

	////////////////////////////WEAPONS INDIVIDUAL SECTION/////////////////////////////////////////////
    else if (pin >= 2000000 && pin < 2000000 + tnowevl)
	{			
        var iidx = pin - 2000000;
	
		var eid;
		var exmid;
	
        eid = tnofweivl[iidx];		
        exmid = tnofweivl[iidx].xml_id;	
		
        var pri = generator.GetItemPrice(exmid);
        var shpp = pri * 1.25;

        var pmd = CalculateBuyPriceOfItemModifier(pid);
        var obj = generator.GetItemByXmlID(exmid);
        var shpp = Math.ceil(obj.price * pmd);	

        var mon = player.GetMoney(na);

        if (mon < shpp) {
            var csit = exmid.replace(/_/g, " ");

            topic.AddPhrase("You do not have enough money to buy the " + csit.toUpperCase() + " .Price: " + "$" + shpp + " Money: " + "$" + mon);
            topic.AddChoice(200000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
        else {
            var csit = exmid.replace(/_/g, " ");
            topic.AddPhrase("" + csit.toUpperCase() + "." + " Price:" + "$" + shpp + ".");
            topic.AddChoice(20000000 + iidx, "Buy this item?");
            topic.AddChoice(200000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
	}

    else if (pin >= 20000000 && pin < 20000000 + tnofweivl.length)
	{
        var iidx = pin - 20000000;
	
        var eid;
        var exmid;
		
        eid = tnofweivl[iidx];		
        exmid = tnofweivl[iidx].xml_id;	

        var spsid = items.GetShipStationStorageContainerId(pshp,css);

        var obj = generator.AddItemToSpecifiedContainer(spsid, exmid, 1); 	
		
        items.RemoveItemQuantity(scid, eid.item_id, 1); 

        var pmd = CalculateBuyPriceOfItemModifier(pid);
        var obj = generator.GetItemByXmlID(exmid);
        var shpp = Math.ceil(obj.price * pmd);

        var na = game.GetShipOwner(pshp);
        player.RemoveMoney(na, shpp);

        var csit = exmid.replace(/_/g, " ");

        topic.AddPhrase("You have bought " + csit.toUpperCase() + " for " + "$" + shpp + ".");
        topic.AddChoice(200000, "Go Back!");
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