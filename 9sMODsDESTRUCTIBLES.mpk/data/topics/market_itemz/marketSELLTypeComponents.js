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
var spcid;
var tnocomivl = 0;
var locomivl = [];
var css;
var cinp;
var stgon = "0";
var sbs;
var cuidx;
var quattb = 0;
var tpce = 0;
var cuipce = 0;
var cusi = "";
var eid;
var exmid;

function OnCheckRequirements() {
    var npid = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npid, "class") == "marketSELLComponents") {
        return true;
    }
    return false;
}

function OnDialogue(args) {

    var pshp = PLAYER_SHIP;
    var na = game.GetShipOwner(pshp);

    pin = topic.GetInput();
    sta = topic.GetState();

    if (pin == NO_INPUT) {
        stgon = "0";

        var arr1;
        sbs = storage.GetGlobal("marketTerminalChosenStation" + pshp, arr1);
        css = sbs;

        spcid = items.GetShipStationStorageContainerId(pshp, css);

        var iicg = items.GetItemsAndCargo(spcid);
        locomivl = [];
        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 9) == "component") {
                locomivl.push(iicg[i]);
            }
        }

        topic.AddChoice(190000, "Components");
        topic.AddChoice(10, "Disconnect");
    }

    else if (pin == 10) {
        topic.DialogueBreak();
    }
    else if (pin == 11) {
        player.StartDialogue(na, "marketSELLStations");
    }
    ////////////////////////////ORES/////////////////////////////////////////////
    else if (pin == 190000) {
        stgon = "0";
        var quat;

        var iicg = items.GetItemsAndCargo(spcid);
        locomivl = [];

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 9) == "component") {
                locomivl.push(iicg[i]);
            }
        }

        var arpce = [];
        var gspv = storage.GetGlobal("globalShopPrice", arpce);

        var i;
        for (i = 0; i < locomivl.length; i++) {
            quat = locomivl[i].quantity;

            var pce;
            for (var j = 0; j < gspv.length; j++) {
                if (gspv[j].base_id == sbs && gspv[j].resourceID == locomivl[i].xml_id) {
                    var pmd = CalculateSellPriceOfCargo(pshp);
                    pce = Math.ceil(gspv[j].price * pmd);
                }
            }

            var spce = pce;
            var cstg = locomivl[i].xml_id.replace(/_/g, " ");
            topic.AddChoice(1900000 + i, cstg.toUpperCase() + " " + "Quantity:" + " " + quat + " Price per unit: " + " $" + spce + ".");
        }

        tnocomivl = i;
        topic.AddChoice(11, "Go back to list of stations");
        topic.AddChoice(10, "Disconnect");
    }

    ////////////////////////////ORES INDIVIDUAL SECTION/////////////////////////////////////////////
    else if (pin >= 1900000 && pin < 1900000 + tnocomivl) {
        cinp = 1900000;
        ccidx = pin - 1900000;

        var quat = 0;

        eid = locomivl[ccidx];
        exmid = locomivl[ccidx].xml_id;

        cusi = exmid.replace(/_/g, " ");

        var spsid = items.GetShipStationStorageContainerId(pshp, css);

        var iicg = items.GetItemsAndCargo(spsid);

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id == exmid) {
                quat = iicg[i].quantity;
                break;
            }
        }

        var arpce = [];
        var gspv = storage.GetGlobal("globalShopPrice", arpce);

        var pce;
        for (var j = 0; j < gspv.length; j++) {
            if (gspv[j].base_id == sbs && gspv[j].resourceID == exmid) {

                var pmd = CalculateSellPriceOfCargo(pshp);
                pce = Math.ceil(gspv[j].price * pmd);
            }
        }
        cuipce = pce;
        var quatts = parseInt(stgon);

        if (quatts <= quat) {
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

            topic.AddChoice(19000000 + ccidx, "Finished setting quantity! Sell " + quatts + " " + cusi.toUpperCase() + " for " + "$" + spce + ".");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
        else {
            topic.AddPhrase("You don't have that much on you!");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
    }

    else if (pin >= 19000000 && pin < 19000000 + locomivl.length) {
        var quatts = parseInt(stgon);

        var iidx = pin - 19000000;

        var quat = 0;

        var iicg = items.GetItemsAndCargo(spcid);

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
            var spcid0 = items.GetStationShopContainerId(css);

            var obj = generator.AddItemToSpecifiedContainer(spcid0, exmid, quatts);

            var spsid = items.GetShipStationStorageContainerId(pshp, css);

            items.RemoveCargo(spsid, exmid, quatts);

            var spce = cuipce * quatts;

            var na = game.GetShipOwner(pshp);
            player.AddMoney(na, spce);
            stgon = "";
            ccidx = 0;
            topic.AddPhrase("You have sold " + quatts + " " + cusi.toUpperCase() + " for " + "$" + spce + ".");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
    }


    else if (pin == 1000000000 + ccidx) {
        cuidx = pin - 1000000000; // index of item,
        stgon += "0";

        quattb = parseInt(stgon);
        tpce = quattb * cuipce;

        topic.AddChoice((cinp * 10) + cuidx, "Finished setting quantity! Sell " + stgon + " " + cusi.toUpperCase() + " for " + "$" + tpce + ".");
        topic.AddChoice(cinp + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1100000000 + ccidx) {
        cuidx = pin - 1100000000;
        stgon += "1";

        quattb = parseInt(stgon);
        tpce = quattb * cuipce;
        topic.AddChoice((cinp * 10) + cuidx, "Finished setting quantity! Sell " + stgon + " " + cusi.toUpperCase() + " for " + "$" + tpce + ".");

        topic.AddChoice(cinp + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1200000000 + ccidx) {
        cuidx = pin - 1200000000;
        stgon += "2";

        quattb = parseInt(stgon);
        tpce = quattb * cuipce;
        topic.AddChoice((cinp * 10) + cuidx, "Finished setting quantity! Sell " + stgon + " " + cusi.toUpperCase() + " for " + "$" + tpce + ".");

        topic.AddChoice(cinp + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1300000000 + ccidx) {
        cuidx = pin - 1300000000;
        stgon += "3";

        quattb = parseInt(stgon);
        tpce = quattb * cuipce;
        topic.AddChoice((cinp * 10) + cuidx, "Finished setting quantity! Sell " + stgon + " " + cusi.toUpperCase() + " for " + "$" + tpce + ".");

        topic.AddChoice(cinp + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1400000000 + ccidx) {
        cuidx = pin - 1400000000;
        stgon += "4";

        quattb = parseInt(stgon);
        tpce = quattb * cuipce;
        topic.AddChoice((cinp * 10) + cuidx, "Finished setting quantity! Sell " + stgon + " " + cusi.toUpperCase() + " for " + "$" + tpce + ".");

        topic.AddChoice(cinp + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1500000000 + ccidx) {
        cuidx = pin - 1500000000;
        stgon += "5";

        quattb = parseInt(stgon);
        tpce = quattb * cuipce;
        topic.AddChoice((cinp * 10) + cuidx, "Finished setting quantity! Sell " + stgon + " " + cusi.toUpperCase() + " for " + "$" + tpce + ".");

        topic.AddChoice(cinp + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1600000000 + ccidx) {
        cuidx = pin - 1600000000;
        stgon += "6";

        quattb = parseInt(stgon);
        tpce = quattb * cuipce;
        topic.AddChoice((cinp * 10) + cuidx, "Finished setting quantity! Sell " + stgon + " " + cusi.toUpperCase() + " for " + "$" + tpce + ".");

        topic.AddChoice(cinp + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1700000000 + ccidx) {
        cuidx = pin - 1700000000;
        stgon += "7";

        quattb = parseInt(stgon);
        tpce = quattb * cuipce;
        topic.AddChoice((cinp * 10) + cuidx, "Finished setting quantity! Sell " + stgon + " " + cusi.toUpperCase() + " for " + "$" + tpce + ".");

        topic.AddChoice(cinp + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1800000000 + ccidx) {
        cuidx = pin - 1800000000;
        stgon += "8";

        quattb = parseInt(stgon);
        tpce = quattb * cuipce;
        topic.AddChoice((cinp * 10) + cuidx, "Finished setting quantity! Sell " + stgon + " " + cusi.toUpperCase() + " for " + "$" + tpce + ".");

        topic.AddChoice(cinp + cuidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1900000000 + ccidx) {
        cuidx = pin - 1900000000;
        stgon += "9";

        quattb = parseInt(stgon);
        tpce = quattb * cuipce;
        topic.AddChoice((cinp * 10) + cuidx, "Finished setting quantity! Sell " + stgon + " " + cusi.toUpperCase() + " for " + "$" + tpce + ".");

        topic.AddChoice(cinp + cuidx, "Add a new digit to quantity");
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