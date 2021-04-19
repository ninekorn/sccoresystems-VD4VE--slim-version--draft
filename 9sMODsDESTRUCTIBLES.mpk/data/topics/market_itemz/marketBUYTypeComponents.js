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
var cci;
var scid;
var tnocivl = 0;
var locivl = [];
var css;
var cinp;
var son = "";
var sbs;
var cip = 0;
var eid;
var exmid;
var csit;
var qtb = 0;
var tpr = 0;
var cri = -1;

function OnCheckRequirements()
{
    var npid = topic.GetCurrentNpcShipId();

    if (npc.GetTag(npid, "class") == "marketBUYComponents")
    {
        return true;
    }
    return false;
}

function OnDialogue(args)
{
    var pid = PLAYER_SHIP;
    var name = game.GetShipOwner(pid);

    pin = topic.GetInput();
    sta = topic.GetState();

    if (pin == NO_INPUT) {
        son = "0";
        cip = 0;

        var arr;

        var arr1;
        sbs = storage.GetGlobal("marketTerminalChosenStation" + pid, arr1);
        css = sbs;

        scid = items.GetStationShopContainerId(sbs);

        var iicg = items.GetItemsAndCargo(scid);
        locivl = [];
        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 9) == "component") {
                locivl.push(iicg[i]);
            }
        }

        topic.AddChoice(190000, "Components");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1) {
        topic.DialogueBreak();
    }
    else if (pin == 2) {
        player.StartDialogue(name, "marketBUYStations");
    }
    ////////////////////////////COMPONENTS/////////////////////////////////////////////
    else if (pin == 190000) {
        son = "0";
        cip = 0;
        var quat;

        var iicg = items.GetItemsAndCargo(scid);
        locivl = [];
        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id.slice(0, 9) == "component") {
                locivl.push(iicg[i]);
            }
        }

        var i;
        for (i = 0; i < locivl.length; i++) {
            quat = locivl[i].quantity;

            var arrp = [];
            var gsvp = storage.GetGlobal("globalShopPrice", arrp);

            var prc;
            for (var j = 0; j < gsvp.length; j++) {
                if (gsvp[j].base_id == sbs && gsvp[j].resourceID == locivl[i].xml_id) {
                    prc = Math.ceil(gsvp[j].price);
                }
            }

            var spc = prc;

            var currentString = locivl[i].xml_id.replace(/_/g, " ");
            topic.AddChoice(1900000 + i, currentString.toUpperCase() + " " + "Quantity:" + " " + quat + " Price: " + spc + "$");
        }

        tnocivl = i;
        topic.AddChoice(2, "Go back to list of stations");
        topic.AddChoice(1, "Disconnect");
    }


    ////////////////////////////COMPONENT INDIVIDUAL SECTION/////////////////////////////////////////////
    else if (pin >= 1900000 && pin < 1900000 + tnocivl) {
        cinp = 1900000;
        cci = pin - 1900000;

        var qut = 0;

        eid = locivl[cci];
        exmid = locivl[cci].xml_id;

        csit = exmid.replace(/_/g, " ");

        var iicg = items.GetItemsAndCargo(scid);

        for (var i = 0; i < iicg.length; i++) {
            if (iicg[i].xml_id == exmid) {
                qut = iicg[i].quantity;
                break;
            }
        }

        var arrp = [];
        var gsvp = storage.GetGlobal("globalShopPrice", arrp);

        var price;
        for (var j = 0; j < gsvp.length; j++) {
            if (gsvp[j].base_id == sbs && gsvp[j].resourceID == exmid) {
                price = Math.ceil(gsvp[j].price);
            }
        }

        cip = price;
        qtb = parseInt(son);

        spc = price * qtb;

        var mon = player.GetMoney(name);

        if (mon < spc) {
            topic.AddPhrase("You do not have enough money to buy the " + csit.toUpperCase() + " . Price: " + "$" + spc + " Money: " + "$" + mon + ".");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(1, "Disconnect");
        }
        else {
            topic.AddPhrase("Choose a quantity you wish to buy!");
            topic.AddChoice(1000000000 + cci, "Add digit 0 to quantity");
            topic.AddChoice(1100000000 + cci, "Add digit 1 to quantity");
            topic.AddChoice(1200000000 + cci, "Add digit 2 to quantity");
            topic.AddChoice(1300000000 + cci, "Add digit 3 to quantity");
            topic.AddChoice(1400000000 + cci, "Add digit 4 to quantity");
            topic.AddChoice(1500000000 + cci, "Add digit 5 to quantity");
            topic.AddChoice(1600000000 + cci, "Add digit 6 to quantity");
            topic.AddChoice(1700000000 + cci, "Add digit 7 to quantity");
            topic.AddChoice(1800000000 + cci, "Add digit 8 to quantity");
            topic.AddChoice(1900000000 + cci, "Add digit 9 to quantity");
            topic.AddChoice(19000000 + cci, "Finished setting quantity? Buy " + qtb + " " + csit.toUpperCase() + " for " + "$" + spc + ".");

            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(1, "Disconnect");
        }
    }

    else if (pin >= 19000000 && pin < 19000000 + locivl.length)
    {
        qtb = parseInt(son);
        spc = cip * qtb;

        var mon = player.GetMoney(name);

        if (qtb == 0) {
            topic.AddPhrase("Please select a quantity.");
            topic.AddChoice(190000, "Go Back!");
            topic.AddChoice(1, "Disconnect");
        }
        else
        {
            if (mon < spc)
            {
                topic.AddPhrase("You do not have enough money to buy the " + csit.toUpperCase() + " . Price: " + "$" + spc + " Money: " + "$" + mon + ".");
                topic.AddChoice(190000, "Go Back!");
                topic.AddChoice(1, "Disconnect");
            }
            else
            {
                var iidx = pin - 19000000;
                var quat = 0;

                eid = locivl[iidx];
                exmid = locivl[iidx].xml_id;

                var iicg = items.GetItemsAndCargo(scid);
                for (var i = 0; i < iicg.length; i++) {
                    if (iicg[i].xml_id == exmid) {
                        quat = iicg[i].quantity;
                        break;
                    }
                }

                if (qtb > qut) {
                    son = "0";
                    cci = 0;

                    cinp = 0;
                    topic.AddPhrase("Youre trying to buy more than the shop owns! Do you know how to count? Start over.");
                    topic.AddChoice(190000, "Go Back!");
                    topic.AddChoice(1, "Disconnect");
                }
                else {

                    var spsid = items.GetShipStationStorageContainerId(pid, css);

                    var iicgop = items.GetItemsAndCargo(spsid);
                    var qutbt = 0;
                    for (var i = 0; i < iicgop.length; i++) {
                        if (iicgop[i].xml_id == exmid) {
                            qutbt = iicgop[i].quantity;
                        }
                    }

                    var obj = generator.AddItemToSpecifiedContainer(spsid, exmid, qtb);

                    var iicgop = items.GetItemsAndCargo(spsid);
                    var qat = 0;
                    for (var i = 0; i < iicgop.length; i++) {
                        if (iicgop[i].xml_id == exmid) {
                            qat = iicgop[i].quantity;
                        }
                    }

                    var qtwt = 0;

                    if (qat > qutbt) {
                        qtwt = qat - qutbt;
                        items.RemoveCargo(scid, exmid, qtwt);

                        spc = cip * qtwt;

                        var na = game.GetShipOwner(pid);
                        player.RemoveMoney(na, spc);
                        son = "0";

                        cci = 0;

                        topic.AddPhrase("You have bought " + qtwt + " " + csit.toUpperCase() + " for " + "$" + spc + ".");
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

    else if (pin == 1000000000 + cci) {
        cri = pin - 1000000000;
        son += "0";

        qtb = parseInt(son);
        tpr = qtb * cip;

        topic.AddChoice((cinp * 10) + cri, "Finished setting quantity! Buy " + son + " " + csit.toUpperCase() + " for " + "$" + tpr + ".");
        topic.AddChoice(cinp + cri, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1100000000 + cci) {
        cri = pin - 1100000000;
        son += "1";

        qtb = parseInt(son);
        tpr = qtb * cip;
        topic.AddChoice((cinp * 10) + cri, "Finished setting quantity! Buy " + son + " " + csit.toUpperCase() + " for " + "$" + tpr + ".");

        topic.AddChoice(cinp + cri, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1200000000 + cci) {
        cri = pin - 1200000000;
        son += "2";

        qtb = parseInt(son);
        tpr = qtb * cip;
        topic.AddChoice((cinp * 10) + cri, "Finished setting quantity! Buy " + son + " " + csit.toUpperCase() + " for " + "$" + tpr + ".");

        topic.AddChoice(cinp + cri, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1300000000 + cci) {
        cri = pin - 1300000000;
        son += "3";

        qtb = parseInt(son);
        tpr = qtb * cip;
        topic.AddChoice((cinp * 10) + cri, "Finished setting quantity! Buy " + son + " " + csit.toUpperCase() + " for " + "$" + tpr + ".");

        topic.AddChoice(cinp + cri, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1400000000 + cci) {
        cri = pin - 1400000000;
        son += "4";

        qtb = parseInt(son);
        tpr = qtb * cip;
        topic.AddChoice((cinp * 10) + cri, "Finished setting quantity! Buy " + son + " " + csit.toUpperCase() + " for " + "$" + tpr + ".");

        topic.AddChoice(cinp + cri, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1500000000 + cci) {
        cri = pin - 1500000000;
        son += "5";

        qtb = parseInt(son);
        tpr = qtb * cip;
        topic.AddChoice((cinp * 10) + cri, "Finished setting quantity! Buy " + son + " " + csit.toUpperCase() + " for " + "$" + tpr + ".");

        topic.AddChoice(cinp + cri, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }

    else if (pin == 1600000000 + cci) {
        cri = pin - 1600000000;
        son += "6";

        qtb = parseInt(son);
        tpr = qtb * cip;
        topic.AddChoice((cinp * 10) + cri, "Finished setting quantity! Buy " + son + " " + csit.toUpperCase() + " for " + "$" + tpr + ".");

        topic.AddChoice(cinp + cri, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1700000000 + cci) {
        cri = pin - 1700000000;
        son += "7";

        qtb = parseInt(son);
        tpr = qtb * cip;
        topic.AddChoice((cinp * 10) + cri, "Finished setting quantity! Buy " + son + " " + csit.toUpperCase() + " for " + "$" + tpr + ".");

        topic.AddChoice(cinp + cri, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1800000000 + cci) {
        cri = pin - 1800000000;
        son += "8";

        qtb = parseInt(son);
        tpr = qtb * cip;
        topic.AddChoice((cinp * 10) + cri, "Finished setting quantity! Buy " + son + " " + csit.toUpperCase() + " for " + "$" + tpr + ".");

        topic.AddChoice(cinp + cri, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
    else if (pin == 1900000000 + cci) {
        cri = pin - 1900000000;
        son += "9";

        qtb = parseInt(son);
        tpr = qtb * cip;
        topic.AddChoice((cinp * 10) + cri, "Finished setting quantity! Buy " + son + " " + csit.toUpperCase() + " for " + "$" + tpr + ".");

        topic.AddChoice(cinp + cri, "Add a new digit to quantity");
        topic.AddChoice(190000, "Go Back!");
        topic.AddChoice(1, "Disconnect");
    }
}