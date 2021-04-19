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
var ccidx;
var scid;
var tnoconivl = 0;
var loconivl = [];
var csst;
var cinp;
var stgon = "0";
var sbs;
var cidx;
var quattb = 0;
var topce = 0;
var cipce = 0;
var csitm = "";
var eid;
var exmid;

function OnCheckRequirements() {
    var npid = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npid, "class") == "marketSELLConsumables") {
        return true;
    }
    return false;
}

function OnDialogue(args) {

    var pid = PLAYER_SHIP;
    var na = game.GetShipOwner(pid);

    pin = topic.GetInput();
    sta = topic.GetState();

    if (pin == NO_INPUT) {
        stgon = "0";


        var arr1;
        sbs = storage.GetGlobal("marketTerminalChosenStation" + pid, arr1);
        csst = sbs;

        scid = items.GetShipStationStorageContainerId(pid, csst);

        var iicg = items.GetItemsAndCargo(scid);
        loconivl = [];
        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 10) == "consumable") {
                loconivl.push(iicg[i]);
            }
        }

        topic.AddChoice(190000, "Consumables");
        topic.AddChoice(10, "Disconnect");
    }

    else if (pin == 10)
    {
        topic.DialogueBreak();
    }

    else if (pin == 11)
    {
        player.StartDialogue(na, "marketSELLStations");
    }

    ////////////////////////////ORES/////////////////////////////////////////////
    else if (pin == 190000) {
        stgon = "0";
        var quat = 0;

        var iicg = items.GetItemsAndCargo(scid);
        loconivl = [];

        for (var i = 0; i < iicg.length; i++)
        {
            if (iicg[i].xml_id.slice(0, 10) == "consumable")
            {
                loconivl.push(iicg[i]);
            }
        }

        var ii;
        for (ii = 0; ii < loconivl.length; ii++) {

            var pmd = CalculateSellPriceOfItemModifier(pid);
            var obj = generator.GetItemByXmlID(loconivl[ii].xml_id);
            var shopPrice = Math.ceil(obj.price * pmd);

            quat = loconivl[ii].quantity;

            var crtg = loconivl[ii].xml_id.replace(/_/g, " ");
            //topic.AddChoice(1900000 + ii, currentString.toUpperCase() + " " + "Price: " + shopPrice + "$");
            topic.AddChoice(1900000 + ii, crtg.toUpperCase() + " " + "Quantity:" + " " + quat + " Price per unit: " + shopPrice + "$");
        }

        tnoconivl = i;
        topic.AddChoice(11, "Go back to list of stations");
        topic.AddChoice(10, "Disconnect");
    }

    ////////////////////////////ORES INDIVIDUAL SECTION/////////////////////////////////////////////
    else if (pin >= 1900000 && pin < 1900000 + tnoconivl) {
        cinp = 1900000;
        ccidx = pin - 1900000;

        var quat = 0;

        eid = loconivl[ccidx];
        exmid = loconivl[ccidx].xml_id;

        csitm = exmid.replace(/_/g, " ");

        var spsid = items.GetShipStationStorageContainerId(pid, csst);

        var iicg = items.GetItemsAndCargo(spsid);

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id == exmid) {
                quat = iicg[i].quantity;
                break;
            }
        }

        var pmd = CalculateSellPriceOfItemModifier(pid);
        var obj = generator.GetItemByXmlID(loconivl[ccidx].xml_id);
        var pce = Math.ceil(obj.price * pmd);

        cipce = pce;

        var quatts = parseInt(stgon);


        if (quatts <= quat)
        {
            topic.AddPhrase("Choose a quantity you wish to sell!");
            topic.AddChoice(1000000000 + ccidx, "Add digit 0 to quantity");
            topic.AddChoice(1100000000 + ccidx, "Add digit 1 to quantity");
            topic.AddChoice(1200000000 + ccidx, "Add digit 2 to quantity");
            topic.AddChoice(1300000000 + ccidx, "Add digit 3 to quantity");
            topic.AddChoice(1400000000 + ccidx, "Add digit 4 to quantity");
            topic.AddChoice(1500000000 + ccidx, "Add digit 5 to quantity");
            topic.AddChoice(1600000000 + ccidx, "Add digit 6 to quantity");
            topic.AddChoice(1700000000 + ccidx, "Add digit 7 to quantity");
            topic.AddChoice(1800000000 + ccidx, "Add digit 8 to quantity");
            topic.AddChoice(1900000000 + ccidx, "Add digit 9 to quantity");

            topic.AddChoice(19000000 + ccidx, "Finished setting quantity! Sell " + quatts + " " + csitm.toUpperCase() + " for " + "$" + slpce + ".");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
        else {
            topic.AddPhrase("You don't have that much on you!");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
    }

    else if (pin >= 19000000 && pin < 19000000 + loconivl.length) {
        var quatts = parseInt(stgon);

        var quat = 0;

        var iicg = items.GetItemsAndCargo(scid);

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id == exmid) {
                quat = iicg[i].quantity;
                break;
            }
        }

        if (quatts > quat) {
            stgon = "";
            ccidx = 0;
            cinp = 0;
            topic.AddPhrase("You're trying to sell more than you own! Do you know how to count? Start over.");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
        else {
            var scid0 = items.GetStationShopContainerId(csst);

            var obj = generator.AddItemToSpecifiedContainer(scid0, exmid, quatts);

            var spsid = items.GetShipStationStorageContainerId(pid, csst);

            items.RemoveItemQuantity(spsid, eid.item_id, quatts);

            var slpce = cipce * quatts;

            var na = game.GetShipOwner(pid);
            player.AddMoney(na, slpce);
            stgon = na
            ccidx = 0;
            topic.AddPhrase("You have sold " + quatts + " " + csitm.toUpperCase() + " for " + "$" + slpce + ".");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
    }


    else if (pin == 1000000000 + ccidx) {
        cidx = pin - 1000000000; // index of item,
        stgon += "0";

        quattb = parseInt(stgon);
        topce = quattb * cipce;

        topic.AddChoice((cinp * 10) + cidx, "Finished setting quantity! Sell " + stgon + " " + csitm.toUpperCase() + " for " + "$" + topce + ".");
        topic.AddChoice(cinp + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1100000000 + ccidx) {
        cidx = pin - 1100000000;
        stgon += "1";

        quattb = parseInt(stgon);
        topce = quattb * cipce;
        topic.AddChoice((cinp * 10) + cidx, "Finished setting quantity! Sell " + stgon + " " + csitm.toUpperCase() + " for " + "$" + topce + ".");

        topic.AddChoice(cinp + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1200000000 + ccidx) {
        cidx = pin - 1200000000;
        stgon += "2";

        quattb = parseInt(stgon);
        topce = quattb * cipce;
        topic.AddChoice((cinp * 10) + cidx, "Finished setting quantity! Sell " + stgon + " " + csitm.toUpperCase() + " for " + "$" + topce + ".");

        topic.AddChoice(cinp + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1300000000 + ccidx) {
        cidx = pin - 1300000000;
        stgon += "3";

        quattb = parseInt(stgon);
        topce = quattb * cipce;
        topic.AddChoice((cinp * 10) + cidx, "Finished setting quantity! Sell " + stgon + " " + csitm.toUpperCase() + " for " + "$" + topce + ".");

        topic.AddChoice(cinp + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1400000000 + ccidx) {
        cidx = pin - 1400000000;
        stgon += "4";

        quattb = parseInt(stgon);
        topce = quattb * cipce;
        topic.AddChoice((cinp * 10) + cidx, "Finished setting quantity! Sell " + stgon + " " + csitm.toUpperCase() + " for " + "$" + topce + ".");

        topic.AddChoice(cinp + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1500000000 + ccidx) {
        cidx = pin - 1500000000;
        stgon += "5";

        quattb = parseInt(stgon);
        topce = quattb * cipce;
        topic.AddChoice((cinp * 10) + cidx, "Finished setting quantity! Sell " + stgon + " " + csitm.toUpperCase() + " for " + "$" + topce + ".");

        topic.AddChoice(cinp + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1600000000 + ccidx) {
        cidx = pin - 1600000000;
        stgon += "6";

        quattb = parseInt(stgon);
        topce = quattb * cipce;
        topic.AddChoice((cinp * 10) + cidx, "Finished setting quantity! Sell " + stgon + " " + csitm.toUpperCase() + " for " + "$" + topce + ".");

        topic.AddChoice(cinp + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1700000000 + ccidx) {
        cidx = pin - 1700000000;
        stgon += "7";

        quattb = parseInt(stgon);
        topce = quattb * cipce;
        topic.AddChoice((cinp * 10) + cidx, "Finished setting quantity! Sell " + stgon + " " + csitm.toUpperCase() + " for " + "$" + topce + ".");

        topic.AddChoice(cinp + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1800000000 + ccidx) {
        cidx = pin - 1800000000;
        stgon += "8";

        quattb = parseInt(stgon);
        topce = quattb * cipce;
        topic.AddChoice((cinp * 10) + cidx, "Finished setting quantity! Sell " + stgon + " " + csitm.toUpperCase() + " for " + "$" + topce + ".");

        topic.AddChoice(cinp + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1900000000 + ccidx) {
        cidx = pin - 1900000000;
        stgon += "9";

        quattb = parseInt(stgon);
        topce = quattb * cipce;
        topic.AddChoice((cinp * 10) + cidx, "Finished setting quantity! Sell " + stgon + " " + csitm.toUpperCase() + " for " + "$" + topce + ".");

        topic.AddChoice(cinp + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
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