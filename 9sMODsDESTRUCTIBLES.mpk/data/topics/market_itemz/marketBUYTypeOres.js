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
var ccid;
var scid;
var tnooivl = 0;
var looivl = [];
var css;
var cinp;
var son = "";
var sbs;
var cip = 0;
var eid;
var exmid;
var csi;
var qtb = 0;
var tp = 0;
var cind = -1;

function OnCheckRequirements() {
    var npid = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npid, "class") == "marketBUYOres") {
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

    if (pin == NO_INPUT) {
        son = "0";
        cip = 0;

        var arr1;
        sbs = storage.GetGlobal("marketTerminalChosenStation" + pid, arr1);
        css = sbs;

        scid = items.GetStationShopContainerId(sbs);

        var iicg = items.GetItemsAndCargo(scid);
        looivl = [];
        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 3) == "ore") {
                looivl.push(iicg[i]);
            }
        }

        topic.AddChoice(190000, "Ores");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1) {
        topic.DialogueBreak();
    }
    else if (pin == 2) {
        player.StartDialogue(name, "marketBUYStations");
    }
    ////////////////////////////ORES/////////////////////////////////////////////
    else if (pin == 190000) {
        son = "0";
        cip = 0;
        var quat;

        var iicg = items.GetItemsAndCargo(scid);
        looivl = [];
        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 3) == "ore") {
                looivl.push(iicg[i]);
            }
        }

        var i;
        for (i = 0; i < looivl.length; i++) {
            quat = looivl[i].quantity;

            var arprc = [];
            var glpv = storage.GetGlobal("globalShopPrice", arprc);

            var prc;
            for (var j = 0; j < glpv.length; j++) {
                if (glpv[j].base_id == sbs && glpv[j].resourceID == looivl[i].xml_id) {
                    prc = Math.ceil(glpv[j].price);
                }
            }

            var spce = prc;

            var crtg = looivl[i].xml_id.replace(/_/g, " ");
            topic.AddChoice(1900000 + i, crtg.toUpperCase() + " " + "Quantity:" + " " + quat + " Price: " + spce + "$");
        }

        tnooivl = i;
        topic.AddChoice(2, "Go back to list of stations");
        topic.AddChoice(1, "Disconnect");
    }

    ////////////////////////////ORE INDIVIDUAL SECTION/////////////////////////////////////////////
    else if (pin >= 1900000 && pin < 1900000 + tnooivl) {
        cinp = 1900000;
        ccid = pin - 1900000;

        var quat = 0;

        eid = looivl[ccid];
        exmid = looivl[ccid].xml_id;

        csi = exmid.replace(/_/g, " ");

        var iicg = items.GetItemsAndCargo(scid);

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id == exmid) {
                quat = iicg[i].quantity;
                break;
            }
        }

        var arprc = [];
        var glpv = storage.GetGlobal("globalShopPrice", arprc);

        var prc;
        for (var j = 0; j < glpv.length; j++) {
            if (glpv[j].base_id == sbs && glpv[j].resourceID == exmid) {
                prc = Math.ceil(glpv[j].price);
            }
        }

        cip = prc;
        qtb = parseInt(son);

        spce = prc * qtb;

        var mon = player.GetMoney(name);

        if (mon < spce) {
            topic.AddPhrase("You do not have enough money to buy the " + csi.toUpperCase() + " . Price: " + "$" + spce + " Money: " + "$" + mon + ".");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(1, "Disconnect");
        }
        else {
            topic.AddPhrase("Choose a quantity you wish to buy!");
            topic.AddChoice(1000000000 + ccid, "Add digit 0 to quantity");
            topic.AddChoice(1100000000 + ccid, "Add digit 1 to quantity");
            topic.AddChoice(1200000000 + ccid, "Add digit 2 to quantity");
            topic.AddChoice(1300000000 + ccid, "Add digit 3 to quantity");
            topic.AddChoice(1400000000 + ccid, "Add digit 4 to quantity");
            topic.AddChoice(1500000000 + ccid, "Add digit 5 to quantity");
            topic.AddChoice(1600000000 + ccid, "Add digit 6 to quantity");
            topic.AddChoice(1700000000 + ccid, "Add digit 7 to quantity");
            topic.AddChoice(1800000000 + ccid, "Add digit 8 to quantity");
            topic.AddChoice(1900000000 + ccid, "Add digit 9 to quantity");
            topic.AddChoice(19000000 + ccid, "Finished setting quantity? Buy " + qtb + " " + csi.toUpperCase() + " for " + "$" + spce + ".");

            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(1, "Disconnect");
        }
    }

    else if (pin >= 19000000 && pin < 19000000 + looivl.length) {
        qtb = parseInt(son);
        spce = cip * qtb;

        var mon = player.GetMoney(name);

        if (qtb == 0) {
            topic.AddPhrase("Please select a quantity.")
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(1, "Disconnect");
        }
        else {
            if (mon < spce) {
                topic.AddPhrase("You do not have enough money to buy the " + csi.toUpperCase() + " . Price: " + "$" + spce + " Money: " + "$" + mon + ".");
                topic.AddChoice(190000, "Go Back!");
                topic.AddChoice(1, "Disconnect");
            }
            else {
                var itemIndex = pin - 19000000;
                var quat = 0;

                eid = looivl[itemIndex];
                exmid = looivl[itemIndex].xml_id;

                var iicg = items.GetItemsAndCargo(scid);
                for (var i = 0; i < iicg.length; i++) {
                    if (iicg[i].xml_id == exmid) {
                        quat = iicg[i].quantity;
                        break;
                    }
                }

                if (qtb > quat) {
                    son = "0";
                    ccid = 0;

                    cinp = 0;
                    topic.AddPhrase("Youre trying to buy more than the shop owns! Do you know how to count? Start over.");
                    topic.AddChoice(190000, "Go Back!");
                    topic.AddChoice(1, "Disconnect");
                }
                else {

                    var spsid = items.GetShipStationStorageContainerId(pid, css);

                    var iicgop = items.GetItemsAndCargo(spsid);
                    var quatbt = 0;
                    for (var i = 0; i < iicgop.length; i++) {
                        if (iicgop[i].xml_id == exmid) {
                            quatbt = iicgop[i].quantity;
                        }
                    }

                    var obj = generator.AddItemToSpecifiedContainer(spsid, exmid, qtb);

                    var iicgop = items.GetItemsAndCargo(spsid);
                    var quatat = 0;
                    for (var i = 0; i < iicgop.length; i++) {
                        if (iicgop[i].xml_id == exmid) {
                            quatat = iicgop[i].quantity;
                        }
                    }

                    var quattwt = 0;

                    if (quatat > quatbt) {
                        quattwt = quatat - quatbt;
                        items.RemoveCargo(scid, exmid, quattwt);

                        spce = cip * quattwt;

                        var na = game.GetShipOwner(pid);
                        player.RemoveMoney(na, spce);
                        son = "0";

                        ccid = 0;

                        topic.AddPhrase("You have bought " + quattwt + " " + csi.toUpperCase() + " for " + "$" + spce + ".");
                        topic.AddChoice(190000, "Go Back!");
                        topic.AddChoice(1, "Disconnect");
                    }
                    else
                    {
                        son = "0";
                        topic.AddPhrase("There is no more space in your Cargo. Make some space!");
                        topic.AddChoice(190000, "Go Back!");
                        topic.AddChoice(1, "Disconnect");
                    }
                }
            }
        }
    }

    else if (pin == 1000000000 + ccid) {
        cind = pin - 1000000000;
        son += "0";

        qtb = parseInt(son);
        tp = qtb * cip;

        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");
        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1100000000 + ccid) {
        cind = pin - 1100000000;
        son += "1";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1200000000 + ccid) {
        cind = pin - 1200000000;
        son += "2";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1300000000 + ccid) {
        cind = pin - 1300000000;
        son += "3";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1400000000 + ccid) {
        cind = pin - 1400000000;
        son += "4";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1500000000 + ccid) {
        cind = pin - 1500000000;
        son += "5";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1600000000 + ccid) {
        cind = pin - 1600000000;
        son += "6";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1700000000 + ccid) {
        cind = pin - 1700000000;
        son += "7";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1800000000 + ccid) {
        cind = pin - 1800000000;
        son += "8";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1900000000 + ccid) {
        cind = pin - 1900000000;
        son += "9";

        qtb = parseInt(son);
        tp = qtb * cip;
        topic.AddChoice((cinp * 10) + cind, "Finished setting quantity! Buy " + son + " " + csi.toUpperCase() + " for " + "$" + tp + ".");

        topic.AddChoice(cinp + cind, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
}
