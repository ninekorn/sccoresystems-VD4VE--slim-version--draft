using(npc);
using(console);
using(player);
using(ship);
using(game);
using(generator);
using(storage);
using(station);
using(items);
using(timer);
using(server);


function OnAIInited()
{
    npc.SetDecisionsPerSecond(SHIP_ID, 1);
    /*npc.SetBehavior(SHIP_ID, "avoid_asteroids", false);
    npc.SetBehavior(SHIP_ID, "avoid_ships", false);
    npc.SetBehavior(SHIP_ID, "avoid_bases", false);
    npc.SetBehavior(SHIP_ID, "avoid_debris", false);*/
}

var someSwitch = true;
var mainFrameCounter = 0;
var someOtherSwitch = 0;
var correctSystem;

function Decision(args)
{
    npc.InstantStop(SHIP_ID);
    //var npcCoord = ship.GetCoordinates(SHIP_ID);

    //npc.StickToPoint(SHIP_ID, npcCoord.x, npcCoord.y, 0);
    //mainFrameCounter++;
}