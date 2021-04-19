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
var cucidx;
var scid;
var tnoorivl = 0;
var loorivl = [];
var css;
var cuipt;
var ston = "0";
var sbs;
var cidx;
var quattb = 0;
var topce = 0;
var cuipce = 0;
var csit = "";
var eid;
var exmid;

function OnCheckRequirements() {
    var npid = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npid, "class") == "marketSELLOres") {
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
        ston = "0";

        var arr1;
        sbs = storage.GetGlobal("marketTerminalChosenStation" + pshp, arr1);
        css = sbs;

        scid = items.GetShipStationStorageContainerId(pshp, css);

        var iicg = items.GetItemsAndCargo(scid);
        loorivl = [];
        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 3) == "ore") {
                loorivl.push(iicg[i]);
            }
        }

        topic.AddChoice(190000, "Ores");
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
        ston = "0";
        var quat;

        var iicg = items.GetItemsAndCargo(scid);
        loorivl = [];

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 3) == "ore") {
                loorivl.push(iicg[i]);
            }
        }

        var arpce = [];
        var gspcev = storage.GetGlobal("globalShopPrice", arpce);

        var i;
        for (i = 0; i < loorivl.length; i++) {
            quat = loorivl[i].quantity;

            var pce;
            for (var j = 0; j < gspcev.length; j++) {
                if (gspcev[j].base_id == sbs && gspcev[j].resourceID == loorivl[i].xml_id) {
                    var pmd = CalculateSellPriceOfCargo(pshp);
                    pce = Math.ceil(gspcev[j].price * pmd);
                }
            }

            var spce = pce;
            var cstg = loorivl[i].xml_id.replace(/_/g, " ");
            topic.AddChoice(1900000 + i, cstg.toUpperCase() + " " + "Quantity:" + " " + quat + " Price per unit: " + " $" + spce + ".");
        }

        tnoorivl = i;
        topic.AddChoice(11, "Go back to list of stations");
        topic.AddChoice(10, "Disconnect");
    }

    ////////////////////////////ORES INDIVIDUAL SECTION/////////////////////////////////////////////
    else if (pin >= 1900000 && pin < 1900000 + tnoorivl) {
        cuipt = 1900000;
        cucidx = pin - 1900000;

        var quat = 0;

        eid = loorivl[cucidx];
        exmid = loorivl[cucidx].xml_id;

        csit = exmid.replace(/_/g, " ");

        var spsid = items.GetShipStationStorageContainerId(pshp, css);

        var iicg = items.GetItemsAndCargo(spsid);

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id == exmid) {
                quat = iicg[i].quantity;
                break;
            }
        }

        var arpce = [];
        var gspcev = storage.GetGlobal("globalShopPrice", arpce);

        var pce;
        for (var j = 0; j < gspcev.length; j++) {
            if (gspcev[j].base_id == sbs && gspcev[j].resourceID == exmid) {

                var pmd = CalculateSellPriceOfCargo(pshp);
                pce = Math.ceil(gspcev[j].price * pmd);
            }
        }
        cuipce = pce;
        var quatts = parseInt(ston);

        if (quatts <= quat)
        {
            topic.AddPhrase("Choose a quantity you wish to sell!");
            topic.AddChoice(1000000000 + cucidx, "Add digit 0 to quantity");
            topic.AddChoice(1100000000 + cucidx, "Add digit 1 to quantity");
            topic.AddChoice(1200000000 + cucidx, "Add digit 2 to quantity");
            topic.AddChoice(1300000000 + cucidx, "Add digit 3 to quantity");
            topic.AddChoice(1400000000 + cucidx, "Add digit 4 to quantity");
            topic.AddChoice(1500000000 + cucidx, "Add digit 5 to quantity");
            topic.AddChoice(1600000000 + cucidx, "Add digit 6 to quantity");
            topic.AddChoice(1700000000 + cucidx, "Add digit 7 to quantity");
            topic.AddChoice(1800000000 + cucidx, "Add digit 8 to quantity");
            topic.AddChoice(1900000000 + cucidx, "Add digit 9 to quantity");

            topic.AddChoice(19000000 + cucidx, "Finished setting quantity! Sell " + quatts + " " + csit.toUpperCase() + " for " + "$" + spce + ".");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
        else {
            topic.AddPhrase("You don't have that much on you!");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
    }

    else if (pin >= 19000000 && pin < 19000000 + loorivl.length) {
        var quatts = parseInt(ston);

        var itemIndex = pin - 19000000;

        var quat = 0;

        var iicg = items.GetItemsAndCargo(scid);

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id == exmid) {
                quat = iicg[i].quantity;
                break;
            }
        }

        if (quatts > quat)
        {
            ston = "";
            cucidx = 0;
            cuipt = 0;
            topic.AddPhrase("You're trying to sell more than you own! Do you know how to count? Start over.");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
        else {

            var scid0 = items.GetStationShopContainerId(css);

            var obj = generator.AddItemToSpecifiedContainer(scid0, exmid, quatts);

            var spsid = items.GetShipStationStorageContainerId(pshp, css);

            items.RemoveCargo(spsid, exmid, quatts);

            var spce = cuipce * quatts;

            var na = game.GetShipOwner(pshp);
            player.AddMoney(na, spce);
            ston = "";
            cucidx = 0;
            topic.AddPhrase("You have sold " + quatts + " " + csit.toUpperCase() + " for " + "$" + spce + ".");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(10, "Disconnect");
        }
    }


    else if (pin == 1000000000 + cucidx) {
        cidx = pin - 1000000000; // index of item,
        ston += "0";

        quattb = parseInt(ston);
        topce = quattb * cuipce;

        topic.AddChoice((cuipt * 10) + cidx, "Finished setting quantity! Sell " + ston + " " + csit.toUpperCase() + " for " + "$" + topce + ".");
        topic.AddChoice(cuipt + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1100000000 + cucidx) {
        cidx = pin - 1100000000;
        ston += "1";

        quattb = parseInt(ston);
        topce = quattb * cuipce;
        topic.AddChoice((cuipt * 10) + cidx, "Finished setting quantity! Sell " + ston + " " + csit.toUpperCase() + " for " + "$" + topce + ".");

        topic.AddChoice(cuipt + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1200000000 + cucidx) {
        cidx = pin - 1200000000;
        ston += "2";

        quattb = parseInt(ston);
        topce = quattb * cuipce;
        topic.AddChoice((cuipt * 10) + cidx, "Finished setting quantity! Sell " + ston + " " + csit.toUpperCase() + " for " + "$" + topce + ".");

        topic.AddChoice(cuipt + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1300000000 + cucidx) {
        cidx = pin - 1300000000;
        ston += "3";

        quattb = parseInt(ston);
        topce = quattb * cuipce;
        topic.AddChoice((cuipt * 10) + cidx, "Finished setting quantity! Sell " + ston + " " + csit.toUpperCase() + " for " + "$" + topce + ".");

        topic.AddChoice(cuipt + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1400000000 + cucidx) {
        cidx = pin - 1400000000;
        ston += "4";

        quattb = parseInt(ston);
        topce = quattb * cuipce;
        topic.AddChoice((cuipt * 10) + cidx, "Finished setting quantity! Sell " + ston + " " + csit.toUpperCase() + " for " + "$" + topce + ".");

        topic.AddChoice(cuipt + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1500000000 + cucidx) {
        cidx = pin - 1500000000;
        ston += "5";

        quattb = parseInt(ston);
        topce = quattb * cuipce;
        topic.AddChoice((cuipt * 10) + cidx, "Finished setting quantity! Sell " + ston + " " + csit.toUpperCase() + " for " + "$" + topce + ".");

        topic.AddChoice(cuipt + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1600000000 + cucidx) {
        cidx = pin - 1600000000;
        ston += "6";

        quattb = parseInt(ston);
        topce = quattb * cuipce;
        topic.AddChoice((cuipt * 10) + cidx, "Finished setting quantity! Sell " + ston + " " + csit.toUpperCase() + " for " + "$" + topce + ".");

        topic.AddChoice(cuipt + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1700000000 + cucidx) {
        cidx = pin - 1700000000;
        ston += "7";

        quattb = parseInt(ston);
        topce = quattb * cuipce;
        topic.AddChoice((cuipt * 10) + cidx, "Finished setting quantity! Sell " + ston + " " + csit.toUpperCase() + " for " + "$" + topce + ".");

        topic.AddChoice(cuipt + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1800000000 + cucidx) {
        cidx = pin - 1800000000;
        ston += "8";

        quattb = parseInt(ston);
        topce = quattb * cuipce;
        topic.AddChoice((cuipt * 10) + cidx, "Finished setting quantity! Sell " + ston + " " + csit.toUpperCase() + " for " + "$" + topce + ".");

        topic.AddChoice(cuipt + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1900000000 + cucidx) {
        cidx = pin - 1900000000;
        ston += "9";

        quattb = parseInt(ston);
        topce = quattb * cuipce;
        topic.AddChoice((cuipt * 10) + cidx, "Finished setting quantity! Sell " + ston + " " + csit.toUpperCase() + " for " + "$" + topce + ".");

        topic.AddChoice(cuipt + cidx, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
}

function CalculateSellPriceOfCargo(args) {
    var mi = 0.95;
    var ma = 0.99;
    var trmg = -ship.GetFinalCacheValue(args, "trade_margin");

    var stif = station.GetBaseByID(sbs);
    var fact = relations.GetFactionDispositionToShip(stif.faction, args);
    var relco = 0.2 * GetFactionRelationCoef(fact);

    var coef = mi + (trmg + relco) * 0.04;
    coef = Clamp(mi, coef, ma);
    //console.PrintError("CARGO SellPriceCoef: " + coef + " based on tradeMargin=" + trade_margin + " relation_coef=" + relation_coef);
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
